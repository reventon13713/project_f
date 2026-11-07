#include <vins/factor/rotor_drag_dynamics_factor.h>

#include <ceres/ceres.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>

namespace {

bool near(double value, double expected, double tolerance,
          const char *message) {
  if (std::abs(value - expected) <= tolerance) return true;
  std::cerr << "FAILED: " << message << ": " << value
            << " expected " << expected << '\n';
  return false;
}

}  // namespace

int main() {
  constexpr double dt = 0.1;
  constexpr double mass = 1.57334;
  constexpr double drag = 0.000175;
  constexpr double rotor_speed_sum = 4000.0;
  constexpr double sigma = 0.2;
  const double lambda = drag * rotor_speed_sum / mass;
  // Exact trapezoidal solution of dv/dt = -lambda*v for one interval.
  const double velocity_j =
      (1.0 - 0.5 * lambda * dt) /
      (1.0 + 0.5 * lambda * dt);

  std::unique_ptr<ceres::CostFunction> factor(
      RotorDragDynamicsFactor::Create(
          dt, 0.0, rotor_speed_sum, 0.0, rotor_speed_sum, mass, drag,
          sigma));
  double pose_i[7] = {0, 0, 0, 0, 0, 0, 1};
  double pose_j[7] = {0, 0, 0, 0, 0, 0, 1};
  double speed_i[9] = {1.0, 0.0, 7.0, 0, 0, 0, 0, 0, 0};
  double speed_j[9] = {velocity_j, 0.0, -9.0, 0, 0, 0, 0, 0, 0};
  double const *parameters[] = {pose_i, speed_i, pose_j, speed_j};
  double residuals[2] = {};
  if (!factor->Evaluate(parameters, residuals, nullptr) ||
      !near(residuals[0], 0.0, 1e-10,
            "metric velocity satisfying rotor drag") ||
      !near(residuals[1], 0.0, 1e-10,
            "zero cross-axis acceleration")) {
    return EXIT_FAILURE;
  }

  // A scale-collapsed constant-velocity solution contradicts rotor drag.
  speed_j[0] = speed_i[0];
  if (!factor->Evaluate(parameters, residuals, nullptr) ||
      !(residuals[0] > 2.0) ||
      !near(residuals[1], 0.0, 1e-10,
            "vertical velocity excluded from horizontal residual")) {
    std::cerr << "FAILED: dynamics factor did not reject constant "
                 "horizontal velocity under rotor drag\n";
    return EXIT_FAILURE;
  }

  std::cout << "rotor_drag_dynamics_factor_test: PASS\n";
  return EXIT_SUCCESS;
}
