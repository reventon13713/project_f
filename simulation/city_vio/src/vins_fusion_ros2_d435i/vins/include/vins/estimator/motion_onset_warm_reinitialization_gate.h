#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

namespace vins {

struct MotionOnsetFrameEvidence {
  double timestamp = 0.0;
  double position_step_speed_mps = 0.0;
  double median_flow_px_s = 0.0;
  int visual_tracks = 0;
};

struct MotionOnsetWindowEvidence {
  double maximum_imu_interval_s = 0.0;
  bool maximum_interval_would_be_cut_off = false;
  int old_slots_before_gap = 0;
  int recent_slots_after_gap = 0;
  std::vector<MotionOnsetFrameEvidence> recent_frames;
};

struct MotionOnsetWarmReleaseEvidence {
  int prior_free_solve_count = 0;
  double horizontal_speed_mps = 0.0;
  double vertical_speed_mps = 0.0;
  double median_flow_px_s = 0.0;
  int visual_tracks = 0;
};

// This gate intentionally recognizes the transition, not generic motion.
// Requiring a stationary prefix followed by three moving frames prevents a
// stale window from being rebuilt repeatedly during a long moving segment.
class MotionOnsetWarmReinitializationGate {
 public:
  bool evaluate(const MotionOnsetWindowEvidence &evidence) const {
    constexpr double kImuFactorCutoffS = 10.0;
    constexpr std::size_t kContinuousWindowStates = 11;
    constexpr std::size_t kMotionFrames = 3;
    constexpr std::size_t kMinimumStationaryFrames = 3;
    constexpr double kMaximumRecentFrameGapS = 0.5;
    constexpr double kStationarySpeedMps = 0.025;
    constexpr double kStationaryFlowPxS = 15.0;
    constexpr double kMotionSpeedMps = 0.04;
    constexpr double kMotionFlowPxS = 20.0;

    if (!std::isfinite(evidence.maximum_imu_interval_s) ||
        evidence.maximum_imu_interval_s <= kImuFactorCutoffS ||
        !evidence.maximum_interval_would_be_cut_off ||
        evidence.old_slots_before_gap < 2 ||
        evidence.recent_slots_after_gap < 3 ||
        evidence.recent_frames.size() != kContinuousWindowStates) {
      return false;
    }

    for (std::size_t i = 1; i < evidence.recent_frames.size(); ++i) {
      const double dt = evidence.recent_frames[i].timestamp -
                        evidence.recent_frames[i - 1].timestamp;
      if (!std::isfinite(dt) || dt <= 0.0 ||
          dt > kMaximumRecentFrameGapS) {
        return false;
      }
    }

    const auto is_moving = [&](const MotionOnsetFrameEvidence &frame) {
      return
          (std::isfinite(frame.position_step_speed_mps) &&
           frame.position_step_speed_mps >= kMotionSpeedMps) ||
          (frame.visual_tracks >= 20 &&
           std::isfinite(frame.median_flow_px_s) &&
           frame.median_flow_px_s >= kMotionFlowPxS);
    };
    std::size_t motion_start = evidence.recent_frames.size();
    while (motion_start > 0 &&
           is_moving(evidence.recent_frames[motion_start - 1])) {
      --motion_start;
    }
    if (evidence.recent_frames.size() - motion_start < kMotionFrames) {
      return false;
    }

    std::size_t stationary_frames = 0;
    for (std::size_t i = 0; i < motion_start; ++i) {
      const auto &frame = evidence.recent_frames[i];
      const bool stationary =
          std::isfinite(frame.position_step_speed_mps) &&
          frame.position_step_speed_mps <= kStationarySpeedMps &&
          frame.visual_tracks >= 20 &&
          std::isfinite(frame.median_flow_px_s) &&
          frame.median_flow_px_s <= kStationaryFlowPxS;
      if (stationary) {
        ++stationary_frames;
      } else {
        stationary_frames = 0;
      }
    }
    if (stationary_frames < kMinimumStationaryFrames) return false;

    return true;
  }
};

// A rebuilt window stays prior-free until it has traversed a full window and
// horizontal translation makes accelerometer bias observable. The caller
// requires this evidence on three consecutive frames before scheduling the
// release solve.
class MotionOnsetWarmReleaseGate {
 public:
  bool evaluate(const MotionOnsetWarmReleaseEvidence &evidence) const {
    constexpr int kMinimumPriorFreeSolves = 10;
    constexpr double kMinimumHorizontalSpeedMps = 0.4;
    constexpr double kMaximumVerticalSpeedMps = 0.1;
    constexpr double kMinimumFlowPxS = 20.0;
    constexpr int kMinimumVisualTracks = 20;

    return evidence.prior_free_solve_count >= kMinimumPriorFreeSolves &&
           std::isfinite(evidence.horizontal_speed_mps) &&
           evidence.horizontal_speed_mps >= kMinimumHorizontalSpeedMps &&
           std::isfinite(evidence.vertical_speed_mps) &&
           std::abs(evidence.vertical_speed_mps) <=
               kMaximumVerticalSpeedMps &&
           evidence.visual_tracks >= kMinimumVisualTracks &&
           std::isfinite(evidence.median_flow_px_s) &&
           evidence.median_flow_px_s >= kMinimumFlowPxS;
  }
};

}  // namespace vins
