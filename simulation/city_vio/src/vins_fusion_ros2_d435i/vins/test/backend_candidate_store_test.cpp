#include <vins/estimator/backend_candidate_store.h>

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

bool expect(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    return false;
  }
  return true;
}

vins::BackendState::Ptr state(std::uint64_t generation, double x) {
  auto result =
      vins::BackendState::Ptr(new vins::BackendState);
  result->generation = generation;
  result->states[0].position.x() = x;
  result->pose_parameters[0][0] = x;
  return result;
}

bool discardDestroysCandidateAndPreservesCommitted() {
  vins::BackendCandidateStore store(2);
  std::string error;
  if (!expect(store.initialize(state(5, 1.0), &error), error.c_str()) ||
      !expect(store.begin(10, 1.0, &error), error.c_str())) {
    return false;
  }
  store.candidate()->states[0].position.x() = 9.0;
  if (!expect(store.committed()->states[0].position.x() == 1.0,
              "candidate mutation cannot reach committed") ||
      !expect(store.observe(10, 1.0, false, &error) ==
                  vins::BackendCandidateLifecycle::Decision::kDiscard,
              "unhealthy candidate is discarded")) {
    return false;
  }
  return expect(store.candidate() == nullptr,
                "discard destroys candidate ownership") &&
         expect(store.committed()->generation == 5,
                "discard preserves committed generation") &&
         expect(store.committed()->states[0].position.x() == 1.0,
                "discard preserves committed value");
}

bool commitMovesWholeCandidateOwnership() {
  vins::BackendCandidateStore store(2);
  std::string error;
  if (!expect(store.initialize(state(2, 2.0), &error), error.c_str()) ||
      !expect(store.begin(20, 2.0, &error), error.c_str())) {
    return false;
  }
  auto *candidate_address = store.candidate();
  candidate_address->states[0].position.x() = 3.0;
  if (!expect(store.observe(20, 2.0, true, &error) ==
                  vins::BackendCandidateLifecycle::Decision::kContinue,
              "first healthy update retains candidate") ||
      !expect(store.committed()->states[0].position.x() == 2.0,
              "healthy prefix does not mutate committed") ||
      !expect(store.observe(21, 2.1, true, &error) ==
                  vins::BackendCandidateLifecycle::Decision::kCommit,
              "second healthy update commits")) {
    return false;
  }
  return expect(store.candidate() == nullptr,
                "commit consumes candidate slot") &&
         expect(store.committed() == candidate_address,
                "commit moves complete candidate ownership") &&
         expect(store.committed()->generation == 3,
                "commit advances generation") &&
         expect(store.committed()->states[0].position.x() == 3.0,
                "commit publishes candidate value");
}

bool protocolErrorDoesNotMoveOwnership() {
  vins::BackendCandidateStore store(2);
  std::string error;
  if (!expect(store.initialize(state(0, 1.0), &error), error.c_str()) ||
      !expect(store.begin(1, 0.0, &error), error.c_str())) {
    return false;
  }
  const auto *committed_address = store.committed();
  const auto *candidate_address = store.candidate();
  return expect(store.observe(2, 0.0, true, &error) ==
                    vins::BackendCandidateLifecycle::Decision::kProtocolError,
                "sequence gap is a protocol error") &&
         expect(store.committed() == committed_address,
                "protocol error preserves committed ownership") &&
         expect(store.candidate() == candidate_address,
                "protocol error preserves candidate for fail-closed handling");
}

bool replacementRequiresOwnedMatchingGeneration() {
  vins::BackendCandidateStore store(2);
  std::string error;
  if (!expect(store.initialize(state(4, 1.0), &error), error.c_str()) ||
      !expect(store.begin(30, 3.0, &error), error.c_str())) {
    return false;
  }
  auto replacement = state(5, 8.0);
  auto *replacement_address = replacement.get();
  if (!expect(store.replaceCandidate(std::move(replacement), &error),
              error.c_str()) ||
      !expect(store.candidate() == replacement_address,
              "replacement transfers complete ownership") ||
      !expect(store.committed()->states[0].position.x() == 1.0,
              "replacement cannot mutate committed")) {
    return false;
  }

  auto wrong_generation = state(9, 3.0);
  return expect(!store.replaceCandidate(std::move(wrong_generation), &error),
                "replacement rejects a mismatched generation") &&
         expect(store.candidate() == replacement_address,
                "failed replacement preserves active candidate");
}

bool safeAdvanceRetainsCandidateOwnership() {
  vins::BackendCandidateStore store(2);
  std::string error;
  if (!expect(store.initialize(state(3, 1.0), &error), error.c_str()) ||
      !expect(store.begin(50, 5.0, &error), error.c_str()) ||
      !expect(store.observe(50, 5.0, true, &error) ==
                  vins::BackendCandidateLifecycle::Decision::kContinue,
              "healthy prefix starts")) {
    return false;
  }
  auto *candidate_address = store.candidate();
  return expect(store.observeSafeAdvance(51, 5.1, &error) ==
                    vins::BackendCandidateLifecycle::Decision::kContinue,
                "safe advance continues generation") &&
         expect(store.candidate() == candidate_address,
                "safe advance retains candidate ownership") &&
         expect(store.committed()->generation == 3,
                "safe advance does not commit") &&
         expect(store.lifecycle().healthyUpdates() == 0,
                "safe advance resets healthy prefix");
}

bool discardRebuildsFromLastCandidateCheckpoint() {
  vins::BackendCandidateStore store(3);
  std::string error;
  if (!expect(store.initialize(state(6, 1.0), &error), error.c_str()) ||
      !expect(store.begin(60, 6.0, &error), error.c_str())) {
    return false;
  }
  store.candidate()->states[0].position.x() = 4.0;
  if (!expect(store.observe(60, 6.0, true, &error) ==
                  vins::BackendCandidateLifecycle::Decision::kContinue,
              "checkpoint candidate remains active") ||
      !expect(store.checkpointCandidate(&error), error.c_str())) {
    return false;
  }
  store.candidate()->states[0].position.x() = 9.0;
  if (!expect(store.observe(61, 6.1, false, &error) ==
                  vins::BackendCandidateLifecycle::Decision::kDiscard,
              "unhealthy suffix is discarded") ||
      !expect(store.committed()->states[0].position.x() == 1.0,
              "checkpoint does not publish committed output") ||
      !expect(store.rebuildSeed() != nullptr,
              "checkpoint remains as rebuild seed") ||
      !expect(store.begin(62, 6.2, &error), error.c_str())) {
    return false;
  }
  return expect(store.candidate()->states[0].position.x() == 4.0,
                "next generation clones the healthy checkpoint") &&
         expect(store.committed()->states[0].position.x() == 1.0,
                "rebuild still protects committed state");
}

bool suspectStoreCommitsOnlyAfterHealthyFuture() {
  vins::BackendCandidateStore store(2);
  std::string error;
  if (!expect(store.initialize(state(10, 1.0), &error), error.c_str()) ||
      !expect(store.begin(90, 9.0, &error), error.c_str())) {
    return false;
  }
  store.candidate()->states[0].position.x() = 5.0;
  if (!expect(store.observeSuspect(90, 9.0, &error) ==
                  vins::BackendCandidateLifecycle::Decision::kContinue,
              "store retains first suspect") ||
      !expect(store.committed()->states[0].position.x() == 1.0,
              "suspect is not published") ||
      !expect(store.observe(91, 9.1, true, &error) ==
                  vins::BackendCandidateLifecycle::Decision::kContinue,
              "first healthy follow-up continues") ||
      !expect(store.observe(92, 9.2, true, &error) ==
                  vins::BackendCandidateLifecycle::Decision::kCommit,
              "second healthy follow-up commits")) {
    return false;
  }
  return expect(store.committed()->states[0].position.x() == 5.0,
                "only validated suspect becomes committed");
}

bool recoveredBackendCanAtomicallyReplaceCommitted() {
  vins::BackendCandidateStore store(3);
  std::string error;
  if (!expect(store.initialize(state(7, 1.0), &error), error.c_str()) ||
      !expect(store.begin(100, 10.0, &error), error.c_str())) {
    return false;
  }
  auto recovered = state(2, 8.0);
  auto *recovered_address = recovered.get();
  if (!expect(store.adoptRecoveredCommitted(
                  std::move(recovered), &error),
              error.c_str())) {
    return false;
  }
  return expect(store.committed() == recovered_address,
                "recovery moves complete backend ownership") &&
         expect(store.committed()->generation == 8,
                "recovery advances from current committed generation") &&
         expect(store.committed()->states[0].position.x() == 8.0,
                "recovered state becomes committed") &&
         expect(store.candidate() == nullptr &&
                    store.rebuildSeed() == nullptr,
                "recovery clears stale candidate and checkpoint") &&
         expect(store.lifecycle().phase() ==
                    vins::BackendCandidateLifecycle::Phase::kCommittedOnly,
                "recovery resets lifecycle to committed-only");
}

}  // namespace

int main() {
  if (!discardDestroysCandidateAndPreservesCommitted() ||
      !commitMovesWholeCandidateOwnership() ||
      !protocolErrorDoesNotMoveOwnership() ||
      !replacementRequiresOwnedMatchingGeneration() ||
      !safeAdvanceRetainsCandidateOwnership() ||
      !discardRebuildsFromLastCandidateCheckpoint() ||
      !suspectStoreCommitsOnlyAfterHealthyFuture() ||
      !recoveredBackendCanAtomicallyReplaceCommitted()) {
    return EXIT_FAILURE;
  }
  std::cout << "backend_candidate_store_test: PASS\n";
  return EXIT_SUCCESS;
}
