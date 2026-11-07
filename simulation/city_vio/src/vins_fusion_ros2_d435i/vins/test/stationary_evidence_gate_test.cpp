#include <vins/estimator/stationary_evidence_gate.h>

#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char *message) {
  if (!condition) std::cerr << "FAILED: " << message << '\n';
  return condition;
}

vins::StationaryEvidence stationary() {
  vins::StationaryEvidence result;
  result.visual_tracks = 80;
  result.median_flow_px_s = 2.0;
  result.angular_rate_rad_s = 0.01;
  result.acceleration_norm_error_mps2 = 0.05;
  result.range_required = true;
  result.range_pair_valid = true;
  result.range_delta_m = 0.01;
  return result;
}

}  // namespace

int main() {
  vins::StationaryEvidenceGate gate;
  auto evidence = stationary();
  if (!expect(gate.evaluate(evidence), "all stationary evidence passes"))
    return EXIT_FAILURE;

  evidence = stationary();
  evidence.median_flow_px_s = 15.01;
  if (!expect(!gate.evaluate(evidence), "moving visual flow fails"))
    return EXIT_FAILURE;

  evidence = stationary();
  evidence.angular_rate_rad_s = 0.031;
  if (!expect(!gate.evaluate(evidence), "rotation fails")) return EXIT_FAILURE;

  evidence = stationary();
  evidence.acceleration_norm_error_mps2 = 0.201;
  if (!expect(!gate.evaluate(evidence), "dynamic acceleration fails"))
    return EXIT_FAILURE;

  evidence = stationary();
  evidence.range_delta_m = 0.051;
  if (!expect(!gate.evaluate(evidence), "unstable range fails"))
    return EXIT_FAILURE;

  evidence = stationary();
  evidence.range_required = false;
  evidence.range_pair_valid = false;
  if (!expect(gate.evaluate(evidence), "range-free configuration passes"))
    return EXIT_FAILURE;

  std::cout << "stationary_evidence_gate_test: PASS\n";
  return EXIT_SUCCESS;
}
