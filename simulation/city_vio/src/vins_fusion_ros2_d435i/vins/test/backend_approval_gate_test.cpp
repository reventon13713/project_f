#include <vins/estimator/backend_approval_gate.h>

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

vins::BackendHealthMetrics healthyMetrics() {
  vins::BackendHealthMetrics metrics;
  metrics.finite = true;
  metrics.position_innovation_m = 0.05;
  metrics.velocity_innovation_mps = 0.10;
  metrics.window_kinematic_rms_mps = 0.04;
  metrics.maximum_scale_log_change = 0.06;
  metrics.scale_log_spread = 0.03;
  metrics.kinematic_intervals = 10;
  metrics.landmark_depths = 100;
  metrics.usable_scale_components = 3;
  return metrics;
}

bool completeHealthyEvidencePasses() {
  const vins::BackendApprovalGate gate;
  const auto result = gate.evaluate(healthyMetrics(), true, 0, 0);
  return expect(result.healthy, "complete bounded evidence passes");
}

bool everyIndependentBoundaryFailsClosed() {
  const vins::BackendApprovalGate gate;
  auto check = [&](vins::BackendHealthMetrics metrics,
                   bool solver_usable, int previous_rejects,
                   int current_rejects, const char *message) {
    return expect(
        !gate.evaluate(metrics, solver_usable, previous_rejects,
                       current_rejects)
             .healthy,
        message);
  };

  auto metrics = healthyMetrics();
  metrics.position_innovation_m = 0.100001;
  if (!check(metrics, true, 0, 0, "position overflow fails")) return false;
  metrics = healthyMetrics();
  metrics.velocity_innovation_mps = 0.150001;
  if (!check(metrics, true, 0, 0, "velocity overflow fails")) return false;
  metrics = healthyMetrics();
  metrics.window_kinematic_rms_mps = 0.050001;
  if (!check(metrics, true, 0, 0, "kinematic overflow fails")) return false;
  metrics = healthyMetrics();
  metrics.maximum_scale_log_change = 0.070001;
  if (!check(metrics, true, 0, 0, "scale change overflow fails"))
    return false;
  metrics = healthyMetrics();
  metrics.scale_log_spread = 0.040001;
  if (!check(metrics, true, 0, 0, "scale spread overflow fails"))
    return false;
  metrics = healthyMetrics();
  metrics.landmark_depths = 14;
  if (!check(metrics, true, 0, 0, "insufficient depth support fails"))
    return false;
  metrics = healthyMetrics();
  if (!check(metrics, false, 0, 0, "unusable solver fails")) return false;
  return check(healthyMetrics(), true, 0, 1,
               "worsening reject chain fails");
}

}  // namespace

int main() {
  if (!completeHealthyEvidencePasses() ||
      !everyIndependentBoundaryFailsClosed()) {
    return EXIT_FAILURE;
  }
  std::cout << "backend_approval_gate_test: PASS\n";
  return EXIT_SUCCESS;
}
