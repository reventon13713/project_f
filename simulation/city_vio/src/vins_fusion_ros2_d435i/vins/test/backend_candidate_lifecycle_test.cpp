#include <vins/estimator/backend_candidate_lifecycle.h>

#include <cstdlib>
#include <iostream>
#include <limits>

namespace {

using Lifecycle = vins::BackendCandidateLifecycle;

bool expect(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    return false;
  }
  return true;
}

bool commitAfterRequiredHealthyUpdates() {
  Lifecycle lifecycle(3);
  lifecycle.reset(7);
  return expect(lifecycle.begin(10, 1.0), "candidate begins") &&
         expect(lifecycle.candidateGeneration() == 8,
                "candidate derives from committed generation") &&
         expect(lifecycle.observe(10, 1.0, true) ==
                    Lifecycle::Decision::kContinue,
                "first healthy update continues") &&
         expect(lifecycle.observe(11, 1.1, true) ==
                    Lifecycle::Decision::kContinue,
                "second healthy update continues") &&
         expect(lifecycle.observe(12, 1.2, true) ==
                    Lifecycle::Decision::kCommit,
                "third healthy update commits") &&
         expect(lifecycle.committedGeneration() == 8,
                "commit advances generation") &&
         expect(lifecycle.phase() == Lifecycle::Phase::kCommittedOnly,
                "commit closes candidate");
}

bool discardPreservesCommittedGeneration() {
  Lifecycle lifecycle(3);
  lifecycle.reset(4);
  return expect(lifecycle.begin(20, 2.0), "candidate begins") &&
         expect(lifecycle.observe(20, 2.0, true) ==
                    Lifecycle::Decision::kContinue,
                "healthy prefix continues") &&
         expect(lifecycle.observe(21, 2.1, false) ==
                    Lifecycle::Decision::kDiscard,
                "unhealthy update discards") &&
         expect(lifecycle.committedGeneration() == 4,
                "discard preserves committed generation") &&
         expect(lifecycle.phase() == Lifecycle::Phase::kCommittedOnly,
                "discard closes candidate");
}

bool protocolErrorsFailClosed() {
  Lifecycle lifecycle(2);
  lifecycle.reset();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  return expect(lifecycle.observe(1, 0.0, true) ==
                    Lifecycle::Decision::kProtocolError,
                "observe without candidate fails") &&
         expect(!lifecycle.begin(1, nan), "non-finite start is rejected") &&
         expect(lifecycle.begin(1, 0.0), "valid candidate begins") &&
         expect(lifecycle.observe(2, 0.0, true) ==
                    Lifecycle::Decision::kProtocolError,
                "sequence gap fails") &&
         expect(lifecycle.observe(1, 0.0, true) ==
                    Lifecycle::Decision::kContinue,
                "state is unchanged after protocol error") &&
         expect(lifecycle.observe(2, -0.1, true) ==
                    Lifecycle::Decision::kProtocolError,
                "timestamp regression fails") &&
         expect(lifecycle.observe(2, 0.1, true) ==
                    Lifecycle::Decision::kCommit,
                "valid retry can commit");
}

bool invalidConfigurationNeverBegins() {
  Lifecycle lifecycle(0);
  lifecycle.reset();
  return expect(!lifecycle.valid(), "zero horizon is invalid") &&
         expect(!lifecycle.begin(1, 0.0),
                "invalid lifecycle cannot begin");
}

bool safeAdvanceRetainsGenerationAndResetsApprovalPrefix() {
  Lifecycle lifecycle(3);
  lifecycle.reset(2);
  return expect(lifecycle.begin(40, 4.0),
                "safe-advance generation begins") &&
         expect(lifecycle.observe(40, 4.0, true) ==
                    Lifecycle::Decision::kContinue,
                "first healthy update continues") &&
         expect(lifecycle.healthyUpdates() == 1,
                "healthy prefix starts") &&
         expect(lifecycle.observeSafeAdvance(41, 4.1) ==
                    Lifecycle::Decision::kContinue,
                "transaction rollback retains candidate") &&
         expect(lifecycle.healthyUpdates() == 0,
                "rollback resets approval prefix") &&
         expect(lifecycle.candidateGeneration() == 3,
                "rollback preserves candidate generation") &&
         expect(lifecycle.observe(42, 4.2, true) ==
                    Lifecycle::Decision::kContinue,
                "approval restarts after safe advance");
}

bool suspectNeedsHealthyFutureOrSecondFailureDiscards() {
  Lifecycle recovering(3);
  recovering.reset(8);
  if (!expect(recovering.begin(70, 7.0),
              "suspect candidate begins") ||
      !expect(recovering.observeSuspect(70, 7.0) ==
                  Lifecycle::Decision::kContinue,
              "first suspect is retained") ||
      !expect(recovering.suspect(),
              "candidate is marked suspect") ||
      !expect(recovering.observe(71, 7.1, true) ==
                  Lifecycle::Decision::kContinue,
              "first healthy follow-up continues") ||
      !expect(recovering.observe(72, 7.2, true) ==
                  Lifecycle::Decision::kContinue,
              "second healthy follow-up continues") ||
      !expect(recovering.observe(73, 7.3, true) ==
                  Lifecycle::Decision::kCommit,
              "third healthy follow-up commits suspect")) {
    return false;
  }

  Lifecycle failing(3);
  failing.reset(8);
  return expect(failing.begin(80, 8.0),
                "failing suspect begins") &&
         expect(failing.observeSuspect(80, 8.0) ==
                    Lifecycle::Decision::kContinue,
                "first failure starts delayed approval") &&
         expect(failing.observeSuspect(81, 8.1) ==
                    Lifecycle::Decision::kDiscard,
                "second failure discards suspect") &&
         expect(failing.committedGeneration() == 8,
                "suspect discard preserves committed generation");
}

}  // namespace

int main() {
  if (!commitAfterRequiredHealthyUpdates() ||
      !discardPreservesCommittedGeneration() ||
      !protocolErrorsFailClosed() ||
      !invalidConfigurationNeverBegins() ||
      !safeAdvanceRetainsGenerationAndResetsApprovalPrefix() ||
      !suspectNeedsHealthyFutureOrSecondFailureDiscards()) {
    return EXIT_FAILURE;
  }
  std::cout << "backend_candidate_lifecycle_test: PASS\n";
  return EXIT_SUCCESS;
}
