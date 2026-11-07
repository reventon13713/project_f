#pragma once

#include <ceres/ceres.h>

#include <algorithm>

struct HorizontalVelocityFactor {
  HorizontalVelocityFactor(double target_velocity_x,
                           double target_velocity_y,
                           double sigma)
      : target_velocity_x_(target_velocity_x),
        target_velocity_y_(target_velocity_y),
        inverse_sigma_(1.0 / std::max(0.005, sigma)) {}

  template <typename T>
  bool operator()(const T *const speed_bias, T *residuals) const {
    // VINS world Z is gravity-aligned. The configured launch contract maps
    // GPS east/north velocity to VINS world X/Y, so this factor constrains
    // both metric scale and accumulated horizontal course without importing
    // GPS position.
    residuals[0] =
        (speed_bias[0] - T(target_velocity_x_)) * T(inverse_sigma_);
    residuals[1] =
        (speed_bias[1] - T(target_velocity_y_)) * T(inverse_sigma_);
    return true;
  }

  static ceres::CostFunction *Create(double target_velocity_x,
                                     double target_velocity_y,
                                     double sigma) {
    return new ceres::AutoDiffCostFunction<
        HorizontalVelocityFactor, 2, 9>(
        new HorizontalVelocityFactor(
            target_velocity_x, target_velocity_y, sigma));
  }

 private:
  double target_velocity_x_;
  double target_velocity_y_;
  double inverse_sigma_;
};
