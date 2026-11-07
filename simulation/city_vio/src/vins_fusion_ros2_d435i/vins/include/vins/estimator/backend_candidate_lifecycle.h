/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 * and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0.
 *******************************************************/

#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

namespace vins {

// This class owns no estimator data.  It is the fail-closed lifecycle contract
// used by the future dual-state backend:
//
//   committed state --clone--> candidate --N healthy image updates--> commit
//                                      \--first unhealthy update----> discard
//
// The caller must move or destroy the corresponding backend state only after
// receiving kCommit or kDiscard.  Keeping this policy independent of estimator
// storage makes ownership transitions testable before the deep snapshot and
// replay implementation is connected to production.
class BackendCandidateLifecycle {
 public:
  enum class Phase { kCommittedOnly, kEvaluatingCandidate };
  enum class Decision {
    kContinue,
    kCommit,
    kDiscard,
    kProtocolError,
  };

  explicit BackendCandidateLifecycle(std::size_t required_healthy_updates)
      : required_healthy_updates_(required_healthy_updates) {}

  bool valid() const { return required_healthy_updates_ > 0; }

  void reset(std::uint64_t committed_generation = 0) {
    phase_ = Phase::kCommittedOnly;
    committed_generation_ = committed_generation;
    candidate_generation_ = 0;
    healthy_updates_ = 0;
    suspect_ = false;
    last_input_sequence_ = 0;
    has_last_input_sequence_ = false;
    candidate_start_timestamp_ =
        std::numeric_limits<double>::quiet_NaN();
    last_candidate_timestamp_ =
        std::numeric_limits<double>::quiet_NaN();
  }

  // A candidate is cloned from the current committed generation immediately
  // before the first candidate image update is applied.
  bool begin(std::uint64_t input_sequence, double timestamp) {
    if (!valid() || phase_ != Phase::kCommittedOnly ||
        (has_last_input_sequence_ &&
         input_sequence <= last_input_sequence_) ||
        !isFinite(timestamp)) {
      return false;
    }
    phase_ = Phase::kEvaluatingCandidate;
    candidate_generation_ = committed_generation_ + 1;
    healthy_updates_ = 0;
    suspect_ = false;
    last_input_sequence_ = input_sequence - 1;
    has_last_input_sequence_ = true;
    candidate_start_timestamp_ = timestamp;
    last_candidate_timestamp_ =
        std::numeric_limits<double>::quiet_NaN();
    return true;
  }

  // Exactly one observation is required for every candidate image update.
  // Sequence gaps are rejected because they would make the sensor replay
  // contract ambiguous.  Timestamp equality is allowed for synthetic tests;
  // production camera timestamps are expected to increase.
  Decision observe(std::uint64_t input_sequence, double timestamp,
                   bool healthy) {
    if (!acceptInput(input_sequence, timestamp)) {
      return Decision::kProtocolError;
    }

    if (!healthy) {
      finishCandidate();
      return Decision::kDiscard;
    }

    ++healthy_updates_;
    if (healthy_updates_ < required_healthy_updates_) {
      return Decision::kContinue;
    }

    committed_generation_ = candidate_generation_;
    finishCandidate();
    return Decision::kCommit;
  }

  // A raw solve can be rejected and rebuilt from its solve-entry propagated
  // state.  Retain that safe backend, but restart the approval prefix because
  // the rejected visual update is not evidence for commit.
  Decision observeSafeAdvance(std::uint64_t input_sequence,
                              double timestamp) {
    if (!acceptInput(input_sequence, timestamp)) {
      return Decision::kProtocolError;
    }
    healthy_updates_ = 0;
    return Decision::kContinue;
  }

  // The first failed metric starts delayed approval without publishing the
  // suspect candidate.  A second failed metric before N healthy follow-ups
  // discards it.  This lets a persistent visual correction demonstrate
  // stability over future frames while the committed backend stays frozen.
  Decision observeSuspect(std::uint64_t input_sequence,
                          double timestamp) {
    if (!acceptInput(input_sequence, timestamp)) {
      return Decision::kProtocolError;
    }
    healthy_updates_ = 0;
    if (suspect_) {
      finishCandidate();
      return Decision::kDiscard;
    }
    suspect_ = true;
    return Decision::kContinue;
  }

  Phase phase() const { return phase_; }
  std::size_t healthyUpdates() const { return healthy_updates_; }
  std::size_t requiredHealthyUpdates() const {
    return required_healthy_updates_;
  }
  std::uint64_t committedGeneration() const {
    return committed_generation_;
  }
  std::uint64_t candidateGeneration() const {
    return candidate_generation_;
  }
  bool suspect() const { return suspect_; }
  std::uint64_t lastInputSequence() const { return last_input_sequence_; }

 private:
  static bool isFinite(double value) {
    return value == value &&
           value != std::numeric_limits<double>::infinity() &&
           value != -std::numeric_limits<double>::infinity();
  }

  bool acceptInput(std::uint64_t input_sequence, double timestamp) {
    if (phase_ != Phase::kEvaluatingCandidate ||
        input_sequence != last_input_sequence_ + 1 ||
        !isFinite(timestamp) || timestamp < candidate_start_timestamp_ ||
        (isFinite(last_candidate_timestamp_) &&
         timestamp < last_candidate_timestamp_)) {
      return false;
    }
    last_input_sequence_ = input_sequence;
    last_candidate_timestamp_ = timestamp;
    return true;
  }

  void finishCandidate() {
    phase_ = Phase::kCommittedOnly;
    candidate_generation_ = 0;
    healthy_updates_ = 0;
    suspect_ = false;
    candidate_start_timestamp_ =
        std::numeric_limits<double>::quiet_NaN();
    last_candidate_timestamp_ =
        std::numeric_limits<double>::quiet_NaN();
  }

  const std::size_t required_healthy_updates_;
  Phase phase_ = Phase::kCommittedOnly;
  std::uint64_t committed_generation_ = 0;
  std::uint64_t candidate_generation_ = 0;
  std::size_t healthy_updates_ = 0;
  bool suspect_ = false;
  std::uint64_t last_input_sequence_ = 0;
  bool has_last_input_sequence_ = false;
  double candidate_start_timestamp_ =
      std::numeric_limits<double>::quiet_NaN();
  double last_candidate_timestamp_ =
      std::numeric_limits<double>::quiet_NaN();
};

}  // namespace vins
