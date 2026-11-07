#include <vins/estimator/backend_recovery_candidate_selector.h>

#include <cstdlib>
#include <iostream>
#include <limits>

namespace {

bool expect(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    return false;
  }
  return true;
}

vins::BackendRecoveryCandidateEvidence evidence(int phase) {
  vins::BackendRecoveryCandidateEvidence result;
  result.downsample_phase = phase;
  result.replayed_through_sequence = 200;
  result.healthy_updates_after_corridor = 3;
  result.maximum_scale_log_change = 0.03;
  result.maximum_kinematic_rms = 0.02;
  result.published_output_correction_m = 0.2;
  return result;
}

bool failClosedBoundaries() {
  vins::BackendRecoveryCandidateSelector selector(3);
  auto phase0 = evidence(0);
  auto phase1 = evidence(1);
  phase0.replayed_through_sequence = 199;
  phase1.healthy_updates_after_corridor = 2;
  auto selection = selector.select(phase0, phase1, 200);
  if (!expect(selection.selected_phase == -1,
              "candidate must cover corridor and approval horizon")) {
    return false;
  }
  phase0 = evidence(0);
  phase0.published_output_correction_m = 0.500001;
  phase1.fatal_processing_error = true;
  selection = selector.select(phase0, phase1, 200);
  if (!expect(selection.selected_phase == -1,
              "output correction and fatal error fail closed")) {
    return false;
  }
  phase0 = evidence(0);
  phase0.maximum_scale_log_change =
      std::numeric_limits<double>::quiet_NaN();
  phase1.protocol_error = true;
  selection = selector.select(phase0, phase1, 200);
  return expect(selection.selected_phase == -1,
                "nonfinite metric and protocol error fail closed");
}

bool onlyEligiblePhaseWins() {
  vins::BackendRecoveryCandidateSelector selector(3);
  auto phase0 = evidence(0);
  auto phase1 = evidence(1);
  phase0.fatal_processing_error = true;
  auto selection = selector.select(phase0, phase1, 200);
  return expect(!selection.phase0_eligible &&
                    selection.phase1_eligible &&
                    selection.selected_phase == 1,
                "single eligible phase is selected");
}

bool lowerScaleThenKinematicsWins() {
  vins::BackendRecoveryCandidateSelector selector(3);
  auto phase0 = evidence(0);
  auto phase1 = evidence(1);
  phase0.maximum_scale_log_change = 0.04;
  phase1.maximum_scale_log_change = 0.03;
  auto selection = selector.select(phase0, phase1, 200);
  if (!expect(selection.selected_phase == 1,
              "lower maximum scale change wins")) {
    return false;
  }
  phase0.maximum_scale_log_change = 0.03;
  phase1.maximum_scale_log_change = 0.03;
  phase0.maximum_kinematic_rms = 0.015;
  phase1.maximum_kinematic_rms = 0.02;
  selection = selector.select(phase0, phase1, 200);
  return expect(selection.selected_phase == 0,
                "kinematic RMS breaks equal-scale tie");
}

bool invalidRoleDoesNotCrossWirePhases() {
  vins::BackendRecoveryCandidateSelector selector(3);
  auto phase0 = evidence(1);
  auto phase1 = evidence(0);
  return expect(selector.select(phase0, phase1, 200).selected_phase == -1,
                "phase roles cannot be swapped accidentally");
}

}  // namespace

int main() {
  if (!failClosedBoundaries() ||
      !onlyEligiblePhaseWins() ||
      !lowerScaleThenKinematicsWins() ||
      !invalidRoleDoesNotCrossWirePhases()) {
    return EXIT_FAILURE;
  }
  std::cout << "backend_recovery_candidate_selector_test: PASS\n";
  return EXIT_SUCCESS;
}
