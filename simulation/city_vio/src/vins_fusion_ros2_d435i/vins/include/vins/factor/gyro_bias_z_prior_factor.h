#pragma once

#include <ceres/ceres.h>

#include <cmath>

// Causal GPS-dropout prior on the slowly varying vertical gyro bias. The
// target is captured from the last solve that still contained fresh GPS
// position factors; no GPS position or velocity is extrapolated.
class GyroBiasZPriorFactor : public ceres::SizedCostFunction<1, 9> {
 public:
  GyroBiasZPriorFactor(double target_bias_rad_s, double sigma_rad_s)
      : target_bias_rad_s_(target_bias_rad_s),
        inverse_sigma_(1.0 / sigma_rad_s) {}

  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const override {
    residuals[0] =
        (parameters[0][8] - target_bias_rad_s_) * inverse_sigma_;
    if (jacobians && jacobians[0]) {
      for (int column = 0; column < 9; ++column) {
        jacobians[0][column] = 0.0;
      }
      jacobians[0][8] = inverse_sigma_;
    }
    return std::isfinite(residuals[0]);
  }

 private:
  double target_bias_rad_s_;
  double inverse_sigma_;
};
