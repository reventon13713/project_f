/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 * and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0.
 *******************************************************/

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace vins {

struct BackendRecoveryCandidateEvidence {
  int downsample_phase = -1;
  std::uint64_t replayed_through_sequence = 0;
  std::size_t healthy_updates_after_corridor = 0;
  bool fatal_processing_error = false;
  bool protocol_error = false;
  double maximum_scale_log_change =
      std::numeric_limits<double>::infinity();
  double maximum_kinematic_rms =
      std::numeric_limits<double>::infinity();
  double published_output_correction_m =
      std::numeric_limits<double>::infinity();
};

struct BackendRecoverySelection {
  bool phase0_eligible = false;
  bool phase1_eligible = false;
  int selected_phase = -1;
};

class BackendRecoveryCandidateSelector {
 public:
  explicit BackendRecoveryCandidateSelector(
      std::size_t required_healthy_updates,
      double maximum_output_correction_m = 0.5)
      : required_healthy_updates_(required_healthy_updates),
        maximum_output_correction_m_(maximum_output_correction_m) {}

  bool valid() const {
    return required_healthy_updates_ > 0 &&
           std::isfinite(maximum_output_correction_m_) &&
           maximum_output_correction_m_ > 0.0;
  }

  BackendRecoverySelection select(
      const BackendRecoveryCandidateEvidence &phase0,
      const BackendRecoveryCandidateEvidence &phase1,
      std::uint64_t anomaly_corridor_end_sequence) const {
    BackendRecoverySelection result;
    if (!valid() || phase0.downsample_phase != 0 ||
        phase1.downsample_phase != 1 ||
        anomaly_corridor_end_sequence == 0) {
      return result;
    }
    result.phase0_eligible =
        eligible(phase0, anomaly_corridor_end_sequence);
    result.phase1_eligible =
        eligible(phase1, anomaly_corridor_end_sequence);
    if (result.phase0_eligible && !result.phase1_eligible) {
      result.selected_phase = 0;
    } else if (!result.phase0_eligible && result.phase1_eligible) {
      result.selected_phase = 1;
    } else if (result.phase0_eligible && result.phase1_eligible) {
      if (phase0.maximum_scale_log_change <
          phase1.maximum_scale_log_change) {
        result.selected_phase = 0;
      } else if (phase1.maximum_scale_log_change <
                 phase0.maximum_scale_log_change) {
        result.selected_phase = 1;
      } else {
        result.selected_phase =
            phase0.maximum_kinematic_rms <=
                    phase1.maximum_kinematic_rms
                ? 0
                : 1;
      }
    }
    return result;
  }

 private:
  bool eligible(const BackendRecoveryCandidateEvidence &candidate,
                std::uint64_t corridor_end) const {
    return !candidate.fatal_processing_error &&
           !candidate.protocol_error &&
           candidate.replayed_through_sequence >= corridor_end &&
           candidate.healthy_updates_after_corridor >=
               required_healthy_updates_ &&
           std::isfinite(candidate.maximum_scale_log_change) &&
           std::isfinite(candidate.maximum_kinematic_rms) &&
           std::isfinite(candidate.published_output_correction_m) &&
           candidate.published_output_correction_m <=
               maximum_output_correction_m_;
  }

  std::size_t required_healthy_updates_;
  double maximum_output_correction_m_;
};

}  // namespace vins
