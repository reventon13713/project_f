#include <vins/estimator/backend_candidate_input_buffer.h>

#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    return false;
  }
  return true;
}

vins::BackendImuStep step(double timestamp, double dt) {
  vins::BackendImuStep result;
  result.data.timestamp = timestamp;
  result.delta_time = dt;
  return result;
}

bool discardCarriesOnlyImuIntoNextGeneration() {
  vins::BackendCandidateInputBuffer buffer;
  if (!expect(buffer.begin(10, 1.0), "first generation begins") ||
      !expect(buffer.append(10, 1.0,
                            {step(0.9, 0.1), step(1.0, 0.1)}),
              "first frame is retained") ||
      !expect(buffer.append(11, 1.1, {step(1.1, 0.1)}),
              "second frame is retained")) {
    return false;
  }
  buffer.discard();
  if (!expect(buffer.pendingStepCount() == 3,
              "discard retains every consumed IMU step") ||
      !expect(buffer.pendingEndTimestamp() == 1.1,
              "discard retains the replay endpoint") ||
      !expect(buffer.begin(12, 1.2), "next generation begins") ||
      !expect(buffer.stepsForActiveGeneration().size() == 3,
              "next generation starts with discarded IMU backlog") ||
      !expect(buffer.append(12, 1.2, {step(1.2, 0.1)}),
              "new frame appends after backlog") ||
      !expect(buffer.stepsForActiveGeneration().size() == 4,
              "each retained step appears exactly once")) {
    return false;
  }
  buffer.commit();
  return expect(buffer.pendingStepCount() == 0,
                "commit releases replay backlog") &&
         expect(!std::isfinite(buffer.pendingEndTimestamp()),
                "commit releases the replay endpoint") &&
         expect(!buffer.active(), "commit closes generation");
}

bool protocolChecksFailClosed() {
  vins::BackendCandidateInputBuffer buffer;
  if (!expect(buffer.begin(1, 1.0), "generation begins") ||
      !expect(!buffer.append(2, 1.0, {step(1.0, 0.1)}),
              "sequence gap is rejected") ||
      !expect(!buffer.append(1, 1.0, {step(1.0, -0.1)}),
              "negative dt is rejected") ||
      !expect(buffer.append(1, 1.0, {step(1.0, 0.1)}),
              "valid first frame is accepted") ||
      !expect(!buffer.append(2, 0.9, {step(1.1, 0.1)}),
              "timestamp regression is rejected")) {
    return false;
  }
  buffer.reset();
  return expect(!buffer.begin(1, 2.0),
                "reset does not rewind the backend sequence") &&
         expect(buffer.begin(2, 2.0),
                "reset accepts the next global sequence");
}

bool checkpointTrimsDiscardedPrefix() {
  vins::BackendCandidateInputBuffer buffer;
  if (!expect(buffer.begin(20, 2.0), "checkpoint generation begins") ||
      !expect(buffer.append(20, 2.0,
                            {step(1.9, 0.1), step(2.0, 0.1)}),
              "healthy prefix IMU is retained") ||
      !expect(buffer.checkpoint(), "healthy prefix is checkpointed") ||
      !expect(buffer.append(21, 2.1, {step(2.1, 0.1)}),
              "unhealthy suffix is retained")) {
    return false;
  }
  buffer.discard();
  if (!expect(buffer.pendingStepCount() == 1,
              "discard keeps only IMU after the checkpoint") ||
      !expect(buffer.begin(22, 2.2),
              "rebuild generation begins") ||
      !expect(buffer.stepsForActiveGeneration().size() == 1,
              "rebuild replays only the discarded suffix")) {
    return false;
  }
  return true;
}

}  // namespace

int main() {
  if (!discardCarriesOnlyImuIntoNextGeneration() ||
      !protocolChecksFailClosed() ||
      !checkpointTrimsDiscardedPrefix()) {
    return EXIT_FAILURE;
  }
  std::cout << "backend_candidate_input_buffer_test: PASS\n";
  return EXIT_SUCCESS;
}
