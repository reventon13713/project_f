#include <vins/estimator/motion_onset_warm_reinitialization_gate.h>

#include <cassert>
#include <iostream>

namespace {

vins::MotionOnsetWindowEvidence validEvidence() {
  vins::MotionOnsetWindowEvidence evidence;
  evidence.maximum_imu_interval_s = 13.284;
  evidence.maximum_interval_would_be_cut_off = true;
  evidence.old_slots_before_gap = 8;
  evidence.recent_slots_after_gap = 3;
  for (int i = 0; i < 11; ++i) {
    vins::MotionOnsetFrameEvidence frame;
    frame.timestamp = 37.844 + 0.1 * i;
    frame.position_step_speed_mps = i < 7 ? 0.005 : 0.08;
    frame.median_flow_px_s = i < 7 ? 2.0 : 30.0;
    frame.visual_tracks = 100;
    evidence.recent_frames.push_back(frame);
  }
  return evidence;
}

}  // namespace

int main() {
  const vins::MotionOnsetWarmReinitializationGate gate;

  auto evidence = validEvidence();
  assert(gate.evaluate(evidence));

  evidence = validEvidence();
  evidence.maximum_imu_interval_s = 9.999;
  assert(!gate.evaluate(evidence));

  evidence = validEvidence();
  evidence.maximum_interval_would_be_cut_off = false;
  assert(!gate.evaluate(evidence));

  evidence = validEvidence();
  evidence.recent_slots_after_gap = 2;
  assert(!gate.evaluate(evidence));

  evidence = validEvidence();
  evidence.recent_frames[9].position_step_speed_mps = 0.01;
  evidence.recent_frames[9].median_flow_px_s = 3.0;
  assert(!gate.evaluate(evidence));

  evidence = validEvidence();
  evidence.recent_frames[7].timestamp =
      evidence.recent_frames[6].timestamp + 0.6;
  assert(!gate.evaluate(evidence));

  const vins::MotionOnsetWarmReleaseGate release_gate;
  vins::MotionOnsetWarmReleaseEvidence release;
  release.prior_free_solve_count = 10;
  release.horizontal_speed_mps = 0.4;
  release.vertical_speed_mps = 0.1;
  release.median_flow_px_s = 20.0;
  release.visual_tracks = 20;
  assert(release_gate.evaluate(release));

  auto weak_release = release;
  weak_release.prior_free_solve_count = 9;
  assert(!release_gate.evaluate(weak_release));

  weak_release = release;
  weak_release.horizontal_speed_mps = 0.399;
  assert(!release_gate.evaluate(weak_release));

  weak_release = release;
  weak_release.vertical_speed_mps = -0.101;
  assert(!release_gate.evaluate(weak_release));

  weak_release = release;
  weak_release.median_flow_px_s = 19.999;
  assert(!release_gate.evaluate(weak_release));

  weak_release = release;
  weak_release.visual_tracks = 19;
  assert(!release_gate.evaluate(weak_release));

  std::cout << "motion_onset_warm_reinitialization_gate_test: PASS\n";
  return 0;
}
