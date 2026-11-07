#pragma once

#include <ceres/ceres.h>

struct HorizontalSpeedUpperBoundFactor {
  HorizontalSpeedUpperBoundFactor(double maximum_speed, double sigma)
      : maximum_speed_(maximum_speed), sigma_(sigma) {}

  template <typename T>
  bool operator()(const T *const speed_bias, T *residual) const {
    constexpr double kSmoothHingeMetersPerSecond = 0.005;
    const T speed =
        ceres::sqrt(speed_bias[0] * speed_bias[0] +
                    speed_bias[1] * speed_bias[1] + T(1e-12));
    const T excess = speed - T(maximum_speed_);
    const T positive_excess =
        T(0.5) *
        (excess +
         ceres::sqrt(excess * excess +
                     T(kSmoothHingeMetersPerSecond *
                       kSmoothHingeMetersPerSecond)));
    residual[0] = positive_excess / T(sigma_);
    return true;
  }

  static ceres::CostFunction *Create(double maximum_speed, double sigma) {
    return new ceres::AutoDiffCostFunction<
        HorizontalSpeedUpperBoundFactor, 1, 9>(
        new HorizontalSpeedUpperBoundFactor(maximum_speed, sigma));
  }

 private:
  double maximum_speed_;
  double sigma_;
};

struct VerticalSpeedUpperBoundFactor {
  VerticalSpeedUpperBoundFactor(double maximum_speed, double sigma)
      : maximum_speed_(maximum_speed), sigma_(sigma) {}

  template <typename T>
  bool operator()(const T *const speed_bias, T *residual) const {
    constexpr double kSmoothAbsoluteMetersPerSecond = 0.001;
    constexpr double kSmoothHingeMetersPerSecond = 0.005;
    const T speed =
        ceres::sqrt(speed_bias[2] * speed_bias[2] +
                    T(kSmoothAbsoluteMetersPerSecond *
                      kSmoothAbsoluteMetersPerSecond));
    const T excess = speed - T(maximum_speed_);
    const T positive_excess =
        T(0.5) *
        (excess +
         ceres::sqrt(excess * excess +
                     T(kSmoothHingeMetersPerSecond *
                       kSmoothHingeMetersPerSecond)));
    residual[0] = positive_excess / T(sigma_);
    return true;
  }

  static ceres::CostFunction *Create(double maximum_speed, double sigma) {
    return new ceres::AutoDiffCostFunction<
        VerticalSpeedUpperBoundFactor, 1, 9>(
        new VerticalSpeedUpperBoundFactor(maximum_speed, sigma));
  }

 private:
  double maximum_speed_;
  double sigma_;
};
