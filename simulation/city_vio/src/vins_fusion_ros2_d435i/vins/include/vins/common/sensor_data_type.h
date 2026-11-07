
#ifndef SENSOR_DATA_TYPE_H
#define SENSOR_DATA_TYPE_H
#include <vins/common/data_type.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cstdint>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>

template <typename T>
class SafeClass {
 public:
  SafeClass() = default;
  ~SafeClass() = default;

  void set(const T &value) {
    std::lock_guard<std::mutex> lock(mutex_);
    data_ = value;
    has_value = true;
  }

  bool get(T &value, bool force = false) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_value && !force) return false;
    value = data_;
    has_value = false;
    return true;
  }

  bool check() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_value;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    has_value = false;
  }

 private:
  mutable std::mutex mutex_;
  bool has_value = false;
  T data_;
};

typedef double TimeStampSec;

struct SensorDataBase {
  TimeStampSec timestamp = 0.0;
};

struct OdomData : SensorDataBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

struct PoseData : SensorDataBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

struct IMUData : SensorDataBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Populated only by the replay ledger instrumentation.  These values are
  // never consumed by estimation or preintegration.
  uint64_t source_sequence = 0;
  uint64_t payload_fingerprint = 0;
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

struct RangeData : SensorDataBase {
  uint64_t source_sequence = 0;
  uint64_t payload_fingerprint = 0;
  double range = 0.0;
  bool valid = false;
};

struct MetricVelocityData : SensorDataBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector2d horizontal_velocity = Eigen::Vector2d::Zero();
  Eigen::Vector2d horizontal_position = Eigen::Vector2d::Zero();
  uint64_t integration_segment = 0;
  double sigma = 0.0;
  bool valid = false;
};

struct RotorDynamicsData : SensorDataBase {
  // Source-neutral physical input. Effective rotor rad/s is supplied by the
  // frozen provenance adapter and is never reconstructed in the estimator.
  double total_thrust_newton = 0.0;
  double summed_rotor_speed_rad_s = 0.0;
  uint64_t source_sequence = 0;
  std::string source_hash;
  std::string quality_state;
  double factor_weight_scale = 0.0;
  bool valid = false;
};

struct ImageData : SensorDataBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t sequence = 0;
  uint64_t left_fingerprint = 0;
  uint64_t right_fingerprint = 0;
  cv::Mat image0 = cv::Mat();
  cv::Mat image1 = cv::Mat();
  ImageData(const ImageData &other) {
    timestamp = other.timestamp;
    sequence = other.sequence;
    left_fingerprint = other.left_fingerprint;
    right_fingerprint = other.right_fingerprint;
    image0 = other.image0.clone();
    image1 = other.image1.clone();
  }

  ImageData &operator=(const ImageData &other) {
    if (this != &other) {
      timestamp = other.timestamp;
      sequence = other.sequence;
      left_fingerprint = other.left_fingerprint;
      right_fingerprint = other.right_fingerprint;
      image0 = other.image0.clone();
      image1 = other.image1.clone();
    }
    return *this;
  }

  ImageData(ImageData &&other) noexcept {
    timestamp = other.timestamp;
    sequence = other.sequence;
    left_fingerprint = other.left_fingerprint;
    right_fingerprint = other.right_fingerprint;
    image0 = std::move(other.image0);
    image1 = std::move(other.image1);
  }

  ImageData &operator=(ImageData &&other) noexcept {
    if (this != &other) {
      timestamp = other.timestamp;
      sequence = other.sequence;
      left_fingerprint = other.left_fingerprint;
      right_fingerprint = other.right_fingerprint;
      image0 = std::move(other.image0);
      image1 = std::move(other.image1);
    }
    return *this;
  }

  ImageData() = default;
  ~ImageData() = default;
};

struct PoseSequenceData : SensorDataBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<Eigen::Vector3d> poses;
};

struct ChannelFloat {
  std::string name;
  std::vector<float> values;
};

struct PointCloudData : SensorDataBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<Eigen::Vector3d> points;
  std::vector<ChannelFloat> channels;
};

struct StateData : SensorDataBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();

  void clear() {
    position = Eigen::Vector3d::Zero();
    velocity = Eigen::Vector3d::Zero();
    rotation = Eigen::Matrix3d::Identity();
    accel_bias = Eigen::Vector3d::Zero();
    gyro_bias = Eigen::Vector3d::Zero();
  }

  void swap(StateData &other) {
    position.swap(other.position);
    velocity.swap(other.velocity);
    rotation.swap(other.rotation);
    accel_bias.swap(other.accel_bias);
    gyro_bias.swap(other.gyro_bias);
  }

  void toPoseArray(double *pose) const {
    pose[0] = position.x();
    pose[1] = position.y();
    pose[2] = position.z();
    Eigen::Quaterniond q(rotation);
    pose[3] = q.x();
    pose[4] = q.y();
    pose[5] = q.z();
    pose[6] = q.w();
  }

  void fromPoseArray(const double *pose) {
    position = Eigen::Vector3d(pose[0], pose[1], pose[2]);
    Eigen::Quaterniond q(pose[6], pose[3], pose[4], pose[5]);  // w, x, y, z
    rotation = q.normalized().toRotationMatrix();
  }

  void toSpeedBiasArray(double *sb) const {
    sb[0] = velocity.x();
    sb[1] = velocity.y();
    sb[2] = velocity.z();
    sb[3] = accel_bias.x();
    sb[4] = accel_bias.y();
    sb[5] = accel_bias.z();
    sb[6] = gyro_bias.x();
    sb[7] = gyro_bias.y();
    sb[8] = gyro_bias.z();
  }

  void fromSpeedBiasArray(const double *sb) {
    velocity = Eigen::Vector3d(sb[0], sb[1], sb[2]);
    accel_bias = Eigen::Vector3d(sb[3], sb[4], sb[5]);
    gyro_bias = Eigen::Vector3d(sb[6], sb[7], sb[8]);
  }
};

#endif  //
