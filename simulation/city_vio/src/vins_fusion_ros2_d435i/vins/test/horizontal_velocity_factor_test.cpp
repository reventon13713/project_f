#include <vins/factor/horizontal_velocity_factor.h>
#include <vins/factor/horizontal_displacement_factor.h>
#include <vins/factor/horizontal_speed_upper_bound_factor.h>
#include <vins/estimator/trusted_metric_correction_gate.h>

#include <ceres/ceres.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
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
  std::unique_ptr<ceres::CostFunction> factor(
      HorizontalVelocityFactor::Create(0.6, -0.2, 0.1));
  double speed_bias[9] = {0.8, -0.5, 1.7, 0, 0, 0, 0, 0, 0};
  double const *parameters[] = {speed_bias};
  double residuals[2] = {};
  if (!factor->Evaluate(parameters, residuals, nullptr)) {
    std::cerr << "FAILED: factor evaluation\n";
    return EXIT_FAILURE;
  }
  if (!near(residuals[0], 2.0, 1e-9,
            "east velocity and sigma scaling") ||
      !near(residuals[1], -3.0, 1e-9,
            "north velocity and sigma scaling")) {
    return EXIT_FAILURE;
  }

  speed_bias[2] = -20.0;
  if (!factor->Evaluate(parameters, residuals, nullptr) ||
      !near(residuals[0], 2.0, 1e-9,
            "vertical velocity does not enter east residual") ||
      !near(residuals[1], -3.0, 1e-9,
            "vertical velocity does not enter north residual")) {
    return EXIT_FAILURE;
  }

  std::unique_ptr<ceres::CostFunction> displacement_factor(
      HorizontalDisplacementFactor::Create(12.0, -4.0, 0.2));
  double pose[7] = {12.1, -4.3, 9.0, 0.0, 0.0, 0.0, 1.0};
  double const *pose_parameters[] = {pose};
  if (!displacement_factor->Evaluate(
          pose_parameters, residuals, nullptr) ||
      !near(residuals[0], 0.5, 1e-9,
            "integrated east displacement and sigma scaling") ||
      !near(residuals[1], -1.5, 1e-9,
            "integrated north displacement and sigma scaling")) {
    return EXIT_FAILURE;
  }
  pose[2] = -30.0;
  if (!displacement_factor->Evaluate(
          pose_parameters, residuals, nullptr) ||
      !near(residuals[0], 0.5, 1e-9,
            "vertical position does not enter east residual") ||
      !near(residuals[1], -1.5, 1e-9,
            "vertical position does not enter north residual")) {
    return EXIT_FAILURE;
  }

  TrustedMetricCorrectionEvidence evidence;
  evidence.gps_fresh = true;
  evidence.range_fresh = true;
  evidence.gps_position_error_m = 0.08;
  evidence.gps_velocity_error_mps = 0.20;
  evidence.range_position_error_m = 0.15;
  evidence.output_step_m = 0.40;
  if (!IsTrustedMetricCorrection(evidence)) {
    std::cerr << "FAILED: bounded GPS/range correction was rejected\n";
    return EXIT_FAILURE;
  }
  evidence.output_step_m = 0.51;
  if (IsTrustedMetricCorrection(evidence)) {
    std::cerr << "FAILED: excessive output step was accepted\n";
    return EXIT_FAILURE;
  }
  evidence.output_step_m = 0.40;
  evidence.gps_position_error_m =
      std::numeric_limits<double>::quiet_NaN();
  if (IsTrustedMetricCorrection(evidence)) {
    std::cerr << "FAILED: non-finite GPS evidence was accepted\n";
    return EXIT_FAILURE;
  }
  if (!ShouldUseAnchoredMetricGauge(true, true, true, true) ||
      ShouldUseAnchoredMetricGauge(true, true, true, false) ||
      ShouldUseAnchoredMetricGauge(true, false, true, true)) {
    std::cerr << "FAILED: anchored gauge freshness contract\n";
    return EXIT_FAILURE;
  }
  TrustedGpsDropoutCorrectionEvidence dropout_evidence;
  dropout_evidence.anchored_epoch = true;
  dropout_evidence.gps_position_factor_active = false;
  dropout_evidence.range_factor_active = true;
  dropout_evidence.horizontal_speed_mps = 1.10;
  dropout_evidence.maximum_speed_mps = 1.10;
  dropout_evidence.optimization_position_correction_m = 0.14;
  dropout_evidence.horizontal_output_step_m = 0.40;
  dropout_evidence.vertical_output_step_m = 0.09;
  dropout_evidence.vertical_output_innovation_m = 0.09;
  dropout_evidence.range_position_error_m = 0.14;
  if (!IsTrustedGpsDropoutCorrection(dropout_evidence)) {
    std::cerr << "FAILED: bounded GPS-off correction was rejected\n";
    return EXIT_FAILURE;
  }
  dropout_evidence.horizontal_speed_mps = 1.16;
  if (IsTrustedGpsDropoutCorrection(dropout_evidence)) {
    std::cerr << "FAILED: excessive GPS-off speed was accepted\n";
    return EXIT_FAILURE;
  }
  dropout_evidence.horizontal_speed_mps = 1.10;
  dropout_evidence.vertical_output_step_m = 0.16;
  if (IsTrustedGpsDropoutCorrection(dropout_evidence)) {
    std::cerr << "FAILED: excessive GPS-off vertical step was accepted\n";
    return EXIT_FAILURE;
  }
  dropout_evidence.vertical_output_step_m = 0.09;
  dropout_evidence.vertical_output_innovation_m = 0.11;
  if (!IsTrustedGpsDropoutCorrection(dropout_evidence)) {
    std::cerr << "FAILED: range-bounded GPS-off correction was rejected\n";
    return EXIT_FAILURE;
  }
  dropout_evidence.vertical_output_innovation_m = 0.09;
  dropout_evidence.range_position_error_m = 0.16;
  if (IsTrustedGpsDropoutCorrection(dropout_evidence)) {
    std::cerr << "FAILED: excessive GPS-off range error was accepted\n";
    return EXIT_FAILURE;
  }
  dropout_evidence.range_position_error_m = 0.14;
  dropout_evidence.optimization_position_correction_m = 0.16;
  if (IsTrustedGpsDropoutCorrection(dropout_evidence)) {
    std::cerr << "FAILED: excessive GPS-off optimizer correction was accepted\n";
    return EXIT_FAILURE;
  }
  dropout_evidence.optimization_position_correction_m = 0.14;
  dropout_evidence.gps_position_factor_active = true;
  if (IsTrustedGpsDropoutCorrection(dropout_evidence)) {
    std::cerr << "FAILED: dropout gate overlapped a GPS position factor\n";
    return EXIT_FAILURE;
  }

  std::unique_ptr<ceres::CostFunction> speed_upper_bound(
      HorizontalSpeedUpperBoundFactor::Create(1.10, 0.05));
  double bounded_speed_bias[9] = {0.80, 0.0, 3.0, 0, 0, 0, 0, 0, 0};
  double const *bounded_speed_parameters[] = {bounded_speed_bias};
  double speed_residual[1] = {};
  if (!speed_upper_bound->Evaluate(
          bounded_speed_parameters, speed_residual, nullptr) ||
      std::abs(speed_residual[0]) > 0.01) {
    std::cerr << "FAILED: speed below upper bound was constrained\n";
    return EXIT_FAILURE;
  }
  bounded_speed_bias[0] = 1.60;
  if (!speed_upper_bound->Evaluate(
          bounded_speed_parameters, speed_residual, nullptr) ||
      !near(speed_residual[0], 10.0, 0.01,
            "speed above upper bound")) {
    return EXIT_FAILURE;
  }
  std::unique_ptr<ceres::CostFunction> vertical_speed_upper_bound(
      VerticalSpeedUpperBoundFactor::Create(0.50, 0.05));
  bounded_speed_bias[0] = 0.0;
  bounded_speed_bias[2] = 0.20;
  if (!vertical_speed_upper_bound->Evaluate(
          bounded_speed_parameters, speed_residual, nullptr) ||
      std::abs(speed_residual[0]) > 0.01) {
    std::cerr << "FAILED: vertical speed below upper bound was constrained\n";
    return EXIT_FAILURE;
  }
  bounded_speed_bias[2] = -1.00;
  if (!vertical_speed_upper_bound->Evaluate(
          bounded_speed_parameters, speed_residual, nullptr) ||
      !near(speed_residual[0], 10.0, 0.01,
            "vertical speed above upper bound")) {
    return EXIT_FAILURE;
  }

  std::cout << "horizontal_velocity_factor_test: PASS\n";
  return EXIT_SUCCESS;
}
