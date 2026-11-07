#pragma once

#include <ceres/ceres.h>

#include <algorithm>

struct HorizontalDisplacementFactor {
  HorizontalDisplacementFactor(double target_position_x,
                               double target_position_y,
                               double sigma)
      : target_position_x_(target_position_x),
        target_position_y_(target_position_y),
        inverse_sigma_(1.0 / std::max(0.01, sigma)) {}

  template <typename T>
  bool operator()(const T *const pose, T *residuals) const {
    residuals[0] =
        (pose[0] - T(target_position_x_)) * T(inverse_sigma_);
    residuals[1] =
        (pose[1] - T(target_position_y_)) * T(inverse_sigma_);
    return true;
  }

  static ceres::CostFunction *Create(double target_position_x,
                                     double target_position_y,
                                     double sigma) {
    return new ceres::AutoDiffCostFunction<
        HorizontalDisplacementFactor, 2, 7>(
        new HorizontalDisplacementFactor(
            target_position_x, target_position_y, sigma));
  }

 private:
  double target_position_x_;
  double target_position_y_;
  double inverse_sigma_;
};
