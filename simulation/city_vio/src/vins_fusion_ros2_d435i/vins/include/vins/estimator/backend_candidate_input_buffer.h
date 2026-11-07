/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 * and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0.
 *******************************************************/

#pragma once

#include <vins/common/sensor_data_type.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace vins {

struct BackendImuStep {
  IMUData data;
  double delta_time = 0.0;
};

// Owns IMU input needed to rebuild after discard.  The fail-closed policy
// omits every camera update from the failed generation.  Its IMU steps remain
// pending and are applied once to the next generation before its first image.
class BackendCandidateInputBuffer {
 public:
  bool begin(std::uint64_t sequence, double timestamp) {
    if (active_ || sequence == 0 || !std::isfinite(timestamp) ||
        (has_last_sequence_ && sequence <= last_sequence_)) {
      return false;
    }
    active_ = true;
    first_sequence_ = sequence;
    last_sequence_ = sequence - 1;
    has_last_sequence_ = true;
    last_timestamp_ = std::numeric_limits<double>::quiet_NaN();
    active_steps_ = pending_steps_;
    return true;
  }

  bool append(std::uint64_t sequence, double timestamp,
              const std::vector<BackendImuStep> &steps) {
    if (!active_ || sequence != last_sequence_ + 1 ||
        !std::isfinite(timestamp) ||
        (std::isfinite(last_timestamp_) && timestamp < last_timestamp_)) {
      return false;
    }
    for (const auto &step : steps) {
      if (!std::isfinite(step.data.timestamp) ||
          !std::isfinite(step.delta_time) || step.delta_time < 0.0) {
        return false;
      }
    }
    active_steps_.insert(active_steps_.end(), steps.begin(), steps.end());
    last_sequence_ = sequence;
    last_timestamp_ = timestamp;
    return true;
  }

  void commit() {
    pending_steps_.clear();
    pending_end_timestamp_ =
        std::numeric_limits<double>::quiet_NaN();
    active_steps_.clear();
    finish();
  }

  bool checkpoint() {
    if (!active_) return false;
    checkpoint_step_index_ = active_steps_.size();
    has_checkpoint_ = true;
    return true;
  }

  void discard() {
    const std::size_t keep_from =
        has_checkpoint_ ? checkpoint_step_index_ : 0;
    pending_steps_.assign(active_steps_.begin() + keep_from,
                          active_steps_.end());
    pending_end_timestamp_ = last_timestamp_;
    active_steps_.clear();
    finish();
  }

  void reset() {
    pending_steps_.clear();
    active_steps_.clear();
    active_ = false;
    first_sequence_ = 0;
    last_timestamp_ = std::numeric_limits<double>::quiet_NaN();
    pending_end_timestamp_ =
        std::numeric_limits<double>::quiet_NaN();
  }

  bool active() const { return active_; }
  const std::vector<BackendImuStep> &stepsForActiveGeneration() const {
    return active_steps_;
  }
  std::size_t pendingStepCount() const { return pending_steps_.size(); }
  double pendingEndTimestamp() const { return pending_end_timestamp_; }
  std::uint64_t firstSequence() const { return first_sequence_; }
  std::uint64_t lastSequence() const { return last_sequence_; }

 private:
  void finish() {
    active_ = false;
    first_sequence_ = 0;
    checkpoint_step_index_ = 0;
    has_checkpoint_ = false;
    last_timestamp_ = std::numeric_limits<double>::quiet_NaN();
  }

  bool active_ = false;
  bool has_last_sequence_ = false;
  std::uint64_t first_sequence_ = 0;
  std::uint64_t last_sequence_ = 0;
  double last_timestamp_ = std::numeric_limits<double>::quiet_NaN();
  double pending_end_timestamp_ =
      std::numeric_limits<double>::quiet_NaN();
  std::size_t checkpoint_step_index_ = 0;
  bool has_checkpoint_ = false;
  std::vector<BackendImuStep> pending_steps_;
  std::vector<BackendImuStep> active_steps_;
};

}  // namespace vins
