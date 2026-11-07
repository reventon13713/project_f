#include <vins/factor/gyro_bias_z_prior_factor.h>

#include <cmath>
#include <cstdlib>
#include <iostream>

int main() {
  constexpr double target = 4.0e-5;
  constexpr double sigma = 5.0e-5;
  GyroBiasZPriorFactor factor(target, sigma);
  double speed_bias[9] = {};
  speed_bias[8] = target + 2.0 * sigma;
  double const *parameters[] = {speed_bias};
  double residual[1] = {};
  double jacobian_storage[9] = {};
  double *jacobians[] = {jacobian_storage};
  if (!factor.Evaluate(parameters, residual, jacobians) ||
      std::abs(residual[0] - 2.0) > 1e-12 ||
      std::abs(jacobian_storage[8] - 1.0 / sigma) > 1e-9) {
    std::cerr << "FAILED: gyro z-bias residual or Jacobian\n";
    return EXIT_FAILURE;
  }
  for (int index = 0; index < 8; ++index) {
    if (jacobian_storage[index] != 0.0) {
      std::cerr << "FAILED: factor constrains a non-z-bias state\n";
      return EXIT_FAILURE;
    }
  }
  std::cout << "gyro_bias_z_prior_factor_test: PASS\n";
  return EXIT_SUCCESS;
}
