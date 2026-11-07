#pragma once

#include <ceres/ceres.h>
#include <Eigen/Geometry>

// Unary vertical-position constraint derived from a downward range sensor.
// Attitude/lever-arm compensation is evaluated by the estimator at the
// current linearization point and passed as target_z.  The residual itself
// deliberately has zero orientation and horizontal Jacobians, preventing a
// 1-D altitude sensor from driving roll/pitch, IMU bias, and horizontal scale.
struct RangeFactor {
  RangeFactor(double target_z, double sigma)
      : target_z_(target_z), sigma_(sigma) {}

  template <typename T>
  bool operator()(const T *const pose, T *residual) const {
    residual[0] = (pose[2] - T(target_z_)) / T(sigma_);
    return true;
  }

  static ceres::CostFunction *Create(double target_z, double sigma) {
    return new ceres::AutoDiffCostFunction<RangeFactor, 1, 7>(
        new RangeFactor(target_z, sigma));
  }

 private:
  double target_z_;
  double sigma_;
};
