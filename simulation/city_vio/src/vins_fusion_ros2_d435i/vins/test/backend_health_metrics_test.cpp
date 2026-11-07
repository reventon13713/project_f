#include <vins/estimator/backend_health_metrics.h>

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    return false;
  }
  return true;
}

bool uniformMetricScaleIsMeasuredAcrossComponents() {
  constexpr int kFrames = 5;
  constexpr double kScale = 1.02;
  StateData entry[WINDOW_SIZE + 1];
  StateData optimized[WINDOW_SIZE + 1];
  IntegrationBase::Ptr preintegrations[WINDOW_SIZE + 1];
  ImuOptions options;
  IMUData first;
  for (int i = 0; i <= kFrames; ++i) {
    entry[i].timestamp = 0.1 * i;
    entry[i].position.x() = 0.1 * i;
    entry[i].velocity.x() = 1.0;
    optimized[i] = entry[i];
    optimized[i].position *= kScale;
    optimized[i].velocity *= kScale;
    if (i > 0) {
      preintegrations[i] = std::make_shared<IntegrationBase>(
          first, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
          options);
      IMUData sample = first;
      sample.timestamp = 0.1;
      preintegrations[i]->push_back(sample);
    }
  }
  std::vector<double> entry_inverse_depths{0.2, 0.25, 0.4};
  double optimized_inverse_depths[3][SIZE_FEATURE] = {
      {0.2 / kScale}, {0.25 / kScale}, {0.4 / kScale}};
  const auto metrics = vins::BackendHealthMetricCalculator::compute(
      entry, optimized, kFrames, preintegrations,
      entry_inverse_depths, optimized_inverse_depths, 3);
  return expect(metrics.finite, "uniform scale metrics are finite") &&
         expect(std::abs(metrics.pose_scale_ratio - kScale) < 1e-12,
                "pose scale ratio is exact") &&
         expect(std::abs(metrics.velocity_scale_ratio - kScale) < 1e-12,
                "velocity scale ratio is exact") &&
         expect(std::abs(metrics.landmark_depth_scale_ratio - kScale) <
                    1e-12,
                "landmark depth scale ratio is exact") &&
         expect(metrics.scale_log_spread < 1e-12,
                "uniform scale has no component spread") &&
         expect(std::abs(metrics.window_kinematic_rms_mps) < 1e-12,
                "uniform position and velocity scale remains kinematic") &&
         expect(metrics.usable_scale_components == 3,
                "all scale components are usable");
}

bool inconsistentScaleAndNonfiniteStateAreVisible() {
  StateData entry[WINDOW_SIZE + 1];
  StateData optimized[WINDOW_SIZE + 1];
  entry[0].timestamp = optimized[0].timestamp = 0.0;
  entry[1].timestamp = optimized[1].timestamp = 0.1;
  entry[1].position.x() = 1.0;
  optimized[1].position.x() = 0.9;
  entry[1].velocity.x() = 1.0;
  optimized[1].velocity.x() = 1.1;
  std::vector<double> entry_inverse_depths{0.2};
  double optimized_inverse_depths[1][SIZE_FEATURE] = {{0.2}};
  auto metrics = vins::BackendHealthMetricCalculator::compute(
      entry, optimized, 1, nullptr, entry_inverse_depths,
      optimized_inverse_depths, 1);
  if (!expect(metrics.scale_log_spread > 0.19,
              "opposing component scales produce a spread")) {
    return false;
  }
  optimized[1].position.x() =
      std::numeric_limits<double>::quiet_NaN();
  metrics = vins::BackendHealthMetricCalculator::compute(
      entry, optimized, 1, nullptr, entry_inverse_depths,
      optimized_inverse_depths, 1);
  return expect(!metrics.finite, "non-finite candidate state is rejected");
}

}  // namespace

int main() {
  if (!uniformMetricScaleIsMeasuredAcrossComponents() ||
      !inconsistentScaleAndNonfiniteStateAreVisible()) {
    return EXIT_FAILURE;
  }
  std::cout << "backend_health_metrics_test: PASS\n";
  return EXIT_SUCCESS;
}
