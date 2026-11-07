/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 * and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0.
 *******************************************************/

#pragma once

#include <vins/estimator/backend_candidate_lifecycle.h>
#include <vins/estimator/backend_state.h>

#include <memory>
#include <string>

namespace vins {

// Couples the storage-independent lifecycle to complete owning backends.
// Optimization is allowed only through candidate().  committed() is const and
// cannot be replaced until the lifecycle returns kCommit.
class BackendCandidateStore {
 public:
  explicit BackendCandidateStore(std::size_t required_healthy_updates)
      : lifecycle_(required_healthy_updates) {}

  bool initialize(BackendState::Ptr committed,
                  std::string *error = nullptr) {
    if (!lifecycle_.valid()) {
      return fail("candidate lifecycle horizon is invalid", error);
    }
    if (!committed || !committed->priorAddressesAreOwned()) {
      return fail("committed backend violates ownership", error);
    }
    lifecycle_.reset(committed->generation);
    committed_ = std::move(committed);
    candidate_.reset();
    rebuild_seed_.reset();
    if (error) error->clear();
    return true;
  }

  bool begin(std::uint64_t sequence, double timestamp,
             std::string *error = nullptr) {
    if (!committed_ || candidate_) {
      return fail("candidate store is not ready to begin", error);
    }
    if (!lifecycle_.begin(sequence, timestamp)) {
      return fail("candidate lifecycle rejected begin", error);
    }
    const BackendState *seed =
        rebuild_seed_ ? rebuild_seed_.get() : committed_.get();
    candidate_ = seed->clone(error);
    if (!candidate_) {
      lifecycle_.reset(committed_->generation);
      return false;
    }
    candidate_->generation = lifecycle_.candidateGeneration();
    if (error) error->clear();
    return true;
  }

  BackendCandidateLifecycle::Decision observe(
      std::uint64_t sequence, double timestamp, bool healthy,
      std::string *error = nullptr) {
    if (!candidate_) {
      fail("candidate store has no active candidate", error);
      return BackendCandidateLifecycle::Decision::kProtocolError;
    }
    const auto decision = lifecycle_.observe(sequence, timestamp, healthy);
    if (decision == BackendCandidateLifecycle::Decision::kCommit) {
      candidate_->generation = lifecycle_.committedGeneration();
      committed_ = std::move(candidate_);
      rebuild_seed_.reset();
    } else if (decision ==
               BackendCandidateLifecycle::Decision::kDiscard) {
      candidate_.reset();
    } else if (decision ==
               BackendCandidateLifecycle::Decision::kProtocolError) {
      fail("candidate lifecycle protocol error", error);
      return decision;
    }
    if (error) error->clear();
    return decision;
  }

  bool checkpointCandidate(std::string *error = nullptr) {
    if (!candidate_) {
      return fail("candidate checkpoint requires an active candidate",
                  error);
    }
    auto checkpoint = candidate_->clone(error);
    if (!checkpoint) return false;
    rebuild_seed_ = std::move(checkpoint);
    if (error) error->clear();
    return true;
  }

  BackendCandidateLifecycle::Decision observeSafeAdvance(
      std::uint64_t sequence, double timestamp,
      std::string *error = nullptr) {
    if (!candidate_) {
      fail("candidate store has no active candidate", error);
      return BackendCandidateLifecycle::Decision::kProtocolError;
    }
    const auto decision =
        lifecycle_.observeSafeAdvance(sequence, timestamp);
    if (decision ==
        BackendCandidateLifecycle::Decision::kProtocolError) {
      fail("candidate lifecycle safe-advance protocol error", error);
      return decision;
    }
    if (error) error->clear();
    return decision;
  }

  BackendCandidateLifecycle::Decision observeSuspect(
      std::uint64_t sequence, double timestamp,
      std::string *error = nullptr) {
    if (!candidate_) {
      fail("candidate store has no active candidate", error);
      return BackendCandidateLifecycle::Decision::kProtocolError;
    }
    const auto decision =
        lifecycle_.observeSuspect(sequence, timestamp);
    if (decision ==
        BackendCandidateLifecycle::Decision::kDiscard) {
      candidate_.reset();
    } else if (
        decision ==
        BackendCandidateLifecycle::Decision::kProtocolError) {
      fail("candidate lifecycle suspect protocol error", error);
    }
    if (error &&
        decision !=
            BackendCandidateLifecycle::Decision::kProtocolError) {
      error->clear();
    }
    return decision;
  }

  bool replaceCandidate(BackendState::Ptr candidate,
                        std::string *error = nullptr) {
    if (!candidate_ || !candidate) {
      return fail("candidate replacement requires an active candidate",
                  error);
    }
    if (!candidate->priorAddressesAreOwned()) {
      return fail("replacement candidate violates ownership", error);
    }
    if (candidate->generation != lifecycle_.candidateGeneration()) {
      return fail("replacement candidate generation mismatch", error);
    }
    candidate_ = std::move(candidate);
    if (error) error->clear();
    return true;
  }

  bool adoptRecoveredCommitted(BackendState::Ptr recovered,
                               std::string *error = nullptr) {
    if (!committed_ || !recovered ||
        !recovered->priorAddressesAreOwned()) {
      return fail("recovered committed backend violates ownership",
                  error);
    }
    recovered->generation = committed_->generation + 1;
    lifecycle_.reset(recovered->generation);
    committed_ = std::move(recovered);
    candidate_.reset();
    rebuild_seed_.reset();
    if (error) error->clear();
    return true;
  }

  const BackendState *committed() const { return committed_.get(); }
  BackendState *candidate() { return candidate_.get(); }
  const BackendState *candidate() const { return candidate_.get(); }
  const BackendState *rebuildSeed() const {
    return rebuild_seed_.get();
  }
  const BackendCandidateLifecycle &lifecycle() const { return lifecycle_; }

 private:
  static bool fail(const char *message, std::string *error) {
    if (error) *error = message;
    return false;
  }

  BackendCandidateLifecycle lifecycle_;
  BackendState::Ptr committed_;
  BackendState::Ptr candidate_;
  BackendState::Ptr rebuild_seed_;
};

}  // namespace vins
