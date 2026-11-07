#pragma once

#include <vins/common/sensor_data_type.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <mutex>

namespace vins {

// Carries the last accepted metric epoch through a short estimator
// reinitialization using only the already-calibrated IMU state.  It never
// approves a new visual epoch by itself: the caller must compare the
// reinitialized state with sample() and enforce the continuity limits.
class WarmReinitializationBridge {
 public:
  bool begin(const OdomData &anchor, const IMUData &anchor_imu,
             const Eigen::Vector3d &accel_bias,
             const Eigen::Vector3d &gyro_bias,
             const Eigen::Vector3d &gravity,
             double maximum_duration_s = 5.0) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!finite(anchor) || !finite(anchor_imu) ||
        !accel_bias.allFinite() || !gyro_bias.allFinite() ||
        !gravity.allFinite() || gravity.norm() < 8.0 ||
        gravity.norm() > 11.0 || !std::isfinite(maximum_duration_s) ||
        maximum_duration_s <= 0.0 ||
        std::abs(anchor.timestamp - anchor_imu.timestamp) > 0.1) {
      clearLocked();
      return false;
    }
    active_ = true;
    expired_ = false;
    maximum_duration_s_ = maximum_duration_s;
    start_timestamp_ = anchor_imu.timestamp;
    latest_ = anchor;
    latest_.timestamp = anchor_imu.timestamp;
    latest_imu_ = anchor_imu;
    accel_bias_ = accel_bias;
    gyro_bias_ = gyro_bias;
    gravity_ = gravity;
    history_.clear();
    history_.push_back(latest_);
    return true;
  }

  bool propagate(const IMUData &imu) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_ || !finite(imu)) return false;
    const double dt = imu.timestamp - latest_imu_.timestamp;
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.1) {
      clearLocked();
      return false;
    }
    if (imu.timestamp - start_timestamp_ > maximum_duration_s_) {
      active_ = false;
      expired_ = true;
      return false;
    }

    const Eigen::Vector3d un_acc_0 =
        latest_.orientation *
            (latest_imu_.linear_acceleration - accel_bias_) -
        gravity_;
    const Eigen::Vector3d un_gyr =
        0.5 * (latest_imu_.angular_velocity + imu.angular_velocity) -
        gyro_bias_;
    const Eigen::Vector3d delta_angle = un_gyr * dt;
    const double angle = delta_angle.norm();
    Eigen::Quaterniond delta_q = Eigen::Quaterniond::Identity();
    if (angle > 1e-12) {
      delta_q =
          Eigen::Quaterniond(Eigen::AngleAxisd(angle, delta_angle / angle));
    }
    latest_.orientation = (latest_.orientation * delta_q).normalized();
    const Eigen::Vector3d un_acc_1 =
        latest_.orientation * (imu.linear_acceleration - accel_bias_) -
        gravity_;
    const Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    latest_.position +=
        dt * latest_.velocity + 0.5 * dt * dt * un_acc;
    latest_.velocity += dt * un_acc;
    latest_.timestamp = imu.timestamp;
    latest_imu_ = imu;
    history_.push_back(latest_);
    while (history_.size() > 2 &&
           history_.front().timestamp <
               latest_.timestamp - maximum_duration_s_) {
      history_.pop_front();
    }
    return finite(latest_);
  }

  bool sample(double timestamp, OdomData *result,
              double maximum_gap_s = 0.02) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!result || history_.empty() || !std::isfinite(timestamp) ||
        !std::isfinite(maximum_gap_s) || maximum_gap_s < 0.0) {
      return false;
    }
    auto upper = std::lower_bound(
        history_.begin(), history_.end(), timestamp,
        [](const OdomData &odom, double value) {
          return odom.timestamp < value;
        });
    const OdomData *best = nullptr;
    if (upper != history_.end()) best = &*upper;
    if (upper != history_.begin()) {
      const OdomData &lower = *std::prev(upper);
      if (!best ||
          std::abs(lower.timestamp - timestamp) <
              std::abs(best->timestamp - timestamp)) {
        best = &lower;
      }
    }
    if (!best || std::abs(best->timestamp - timestamp) > maximum_gap_s) {
      return false;
    }
    *result = *best;
    return true;
  }

  bool latest(OdomData *result) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!result || history_.empty()) return false;
    *result = latest_;
    return true;
  }

  bool active() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_;
  }

  bool expired() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return expired_;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    clearLocked();
  }

 private:
  static bool finite(const OdomData &odom) {
    return std::isfinite(odom.timestamp) && odom.position.allFinite() &&
           odom.velocity.allFinite() &&
           odom.orientation.coeffs().allFinite() &&
           std::abs(odom.orientation.norm() - 1.0) < 1e-3;
  }

  static bool finite(const IMUData &imu) {
    return std::isfinite(imu.timestamp) &&
           imu.linear_acceleration.allFinite() &&
           imu.angular_velocity.allFinite();
  }

  void clearLocked() {
    active_ = false;
    expired_ = false;
    maximum_duration_s_ = 0.0;
    start_timestamp_ = std::numeric_limits<double>::quiet_NaN();
    history_.clear();
  }

  mutable std::mutex mutex_;
  bool active_ = false;
  bool expired_ = false;
  double maximum_duration_s_ = 0.0;
  double start_timestamp_ = std::numeric_limits<double>::quiet_NaN();
  OdomData latest_;
  IMUData latest_imu_;
  Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity_ = Eigen::Vector3d::Zero();
  std::deque<OdomData, Eigen::aligned_allocator<OdomData>> history_;
};

}  // namespace vins
