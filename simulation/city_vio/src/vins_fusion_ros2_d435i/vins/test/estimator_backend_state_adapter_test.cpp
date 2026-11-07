#include <vins/estimator/estimator.h>

#include <cstdlib>
#include <iostream>
#include <string>

class EstimatorBackendStateTestAccess {
 public:
  static void populate(Estimator *estimator) {
    estimator->options = std::make_shared<VINSOptions>();
    estimator->options->RIC = {Eigen::Matrix3d::Identity(),
                               Eigen::Matrix3d::Identity()};
    estimator->options->TIC = {Eigen::Vector3d::Zero(),
                               Eigen::Vector3d::Zero()};
    estimator->options->num_of_camera = 1;
    estimator->options->time_delay = 0.003;
    estimator->featureManager.setOptions(estimator->options);
    estimator->backendGeneration = 12;
    estimator->frameCount = 4;
    estimator->solver_flag = SolverState::NON_LINEAR;
    estimator->estimator_state[2].position =
        Eigen::Vector3d(1.0, 2.0, 3.0);
    estimator->estimator_state[2].velocity =
        Eigen::Vector3d(0.4, 0.5, 0.6);
    estimator->estimator_state[2].toPoseArray(estimator->poseArray[2]);
    estimator->estimator_state[2].toSpeedBiasArray(
        estimator->speedBiasArray[2]);
    estimator->para_Ex_Pose[0][6] = 1.0;
    estimator->para_Td[0][0] = 0.003;

    FeaturePerId feature(7, 1);
    Eigen::Matrix<double, 7, 1> observation;
    observation << 0.1, 0.2, 1.0, 20.0, 30.0, 0.5, 0.6;
    feature.feature_per_frame.emplace_back(observation, 0.0);
    observation(0) = 0.2;
    feature.feature_per_frame.emplace_back(observation, 0.0);
    feature.estimated_depth = 4.5;
    estimator->featureManager.feature.push_back(feature);

    ImuOptions imu_options;
    imu_options.accelNoiseDensity = 0.02;
    imu_options.gyroNoiseDensity = 0.01;
    IMUData first;
    first.linear_acceleration.z() = 9.8;
    estimator->pre_integrations[2] =
        std::make_shared<IntegrationBase>(
            first, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
            imu_options);
    IMUData update = first;
    update.timestamp = 0.01;
    update.linear_acceleration.x() = 0.2;
    estimator->pre_integrations[2]->push_back(update);

    estimator->last_marginalization_parameter_blocks = {
        estimator->poseArray[2], estimator->speedBiasArray[2],
        estimator->para_Ex_Pose[0], estimator->para_Td[0]};
    const int sizes[] = {SIZE_POSE, SIZE_SPEEDBIAS, SIZE_POSE, 1};
    const int indices[] = {0, 6, 15, 21};
    auto prior = std::make_shared<MarginalizationInfo>();
    prior->m = 0;
    prior->n = 22;
    prior->sum_block_size = 24;
    prior->keep_block_size.assign(std::begin(sizes), std::end(sizes));
    prior->keep_block_idx.assign(std::begin(indices), std::end(indices));
    prior->linearized_jacobians =
        Eigen::MatrixXd::Identity(22, 22);
    prior->linearized_residuals = Eigen::VectorXd::Zero(22);
    for (std::size_t i = 0;
         i < estimator->last_marginalization_parameter_blocks.size();
         ++i) {
      double *block =
          estimator->last_marginalization_parameter_blocks[i];
      const long key = reinterpret_cast<long>(block);
      auto data = std::make_unique<double[]>(sizes[i]);
      std::copy(block, block + sizes[i], data.get());
      prior->parameter_block_size[key] = sizes[i];
      prior->parameter_block_idx[key] = indices[i];
      prior->keep_block_data.push_back(data.get());
      prior->parameter_block_data[key] = std::move(data);
    }
    estimator->last_marginalization_info = std::move(prior);
  }

  static std::unique_ptr<vins::BackendState> capture(
      const Estimator &estimator, std::string *error) {
    return estimator.captureBackendState(error);
  }

  static bool apply(Estimator *estimator,
                    const vins::BackendState &state,
                    std::string *error) {
    return estimator->applyBackendState(state, error);
  }

  static double livePositionX(const Estimator &estimator) {
    return estimator.estimator_state[2].position.x();
  }

  static double liveFeatureDepth(const Estimator &estimator) {
    return estimator.featureManager.feature.front().estimated_depth;
  }

  static bool livePriorUsesOwnedArrays(const Estimator &estimator) {
    if (!estimator.last_marginalization_info ||
        estimator.last_marginalization_parameter_blocks.size() != 4) {
      return false;
    }
    const std::vector<double *> expected{
        const_cast<double *>(estimator.poseArray[2]),
        const_cast<double *>(estimator.speedBiasArray[2]),
        const_cast<double *>(estimator.para_Ex_Pose[0]),
        const_cast<double *>(estimator.para_Td[0])};
    return estimator.last_marginalization_parameter_blocks == expected &&
           estimator.last_marginalization_info
               ->ownsLinearizationDataFor(expected);
  }

  static void installUnknownPriorBlock(Estimator *estimator,
                                       double *external) {
    estimator->last_marginalization_parameter_blocks[0] = external;
  }

  static std::vector<std::uint64_t> queuedSequences(Estimator *estimator) {
    std::vector<std::uint64_t> result;
    auto queue = estimator->featureBuffer;
    while (!queue.empty()) {
      result.push_back(queue.front().sequence);
      queue.pop();
    }
    return result;
  }

  static void setCandidateWorkspaceActive(Estimator *estimator,
                                          bool active) {
    estimator->backendCandidateWorkspaceActive = active;
  }

  static bool candidateProcessingFailed(const Estimator &estimator) {
    return estimator.backendCandidateProcessingFailed;
  }

  static int frameCount(const Estimator &estimator) {
    return estimator.frameCount;
  }

  static void updateCameraX(Estimator *estimator, double x) {
    estimator->cameraTranslation[0].x() = x;
    estimator->updateCameraPose(0);
  }
};

namespace {

bool expect(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    return false;
  }
  return true;
}

bool candidateWorkspaceCanRestoreCommittedState() {
  Estimator estimator;
  EstimatorBackendStateTestAccess::populate(&estimator);
  std::string error;
  auto committed =
      EstimatorBackendStateTestAccess::capture(estimator, &error);
  if (!expect(committed != nullptr, error.c_str())) return false;
  auto candidate = committed->clone(&error);
  if (!expect(candidate != nullptr, error.c_str())) return false;

  candidate->states[2].position.x() = 9.0;
  candidate->pose_parameters[2][0] = 9.0;
  candidate->feature_manager.feature.front().estimated_depth = 8.0;
  candidate->preintegrations[2]
      ->imu_buffer.front().linear_acceleration.x() = 7.0;
  candidate->marginalization_prior->keep_block_data[0][0] = 6.0;

  if (!expect(
          EstimatorBackendStateTestAccess::apply(
              &estimator, *candidate, &error),
          error.c_str()) ||
      !expect(EstimatorBackendStateTestAccess::livePositionX(estimator) ==
                  9.0,
              "candidate state is applied to live workspace") ||
      !expect(EstimatorBackendStateTestAccess::liveFeatureDepth(estimator) ==
                  8.0,
              "candidate feature state is applied") ||
      !expect(
          EstimatorBackendStateTestAccess::livePriorUsesOwnedArrays(
              estimator),
          "applied candidate prior is rebound to live arrays")) {
    return false;
  }

  if (!expect(
          EstimatorBackendStateTestAccess::apply(
              &estimator, *committed, &error),
          error.c_str()) ||
      !expect(EstimatorBackendStateTestAccess::livePositionX(estimator) ==
                  1.0,
              "committed window state is restored") ||
      !expect(EstimatorBackendStateTestAccess::liveFeatureDepth(estimator) ==
                  4.5,
              "committed feature state is restored") ||
      !expect(
          EstimatorBackendStateTestAccess::livePriorUsesOwnedArrays(
              estimator),
          "restored prior is rebound to live arrays") ||
      !expect(candidate->states[2].position.x() == 9.0,
              "restoring committed does not mutate candidate") ||
      !expect(committed->states[2].position.x() == 1.0,
              "candidate workspace did not mutate committed")) {
    return false;
  }

  auto restored =
      EstimatorBackendStateTestAccess::capture(estimator, &error);
  return expect(restored != nullptr, error.c_str()) &&
         expect(restored->states[2].position.x() == 1.0,
                "round-trip capture preserves committed state") &&
         expect(restored->priorAddressesAreOwned(),
                "round-trip prior is BackendState-owned");
}

bool captureRejectsUnknownLivePriorAddress() {
  Estimator estimator;
  EstimatorBackendStateTestAccess::populate(&estimator);
  double external[SIZE_POSE] = {};
  EstimatorBackendStateTestAccess::installUnknownPriorBlock(
      &estimator, external);
  std::string error;
  auto snapshot =
      EstimatorBackendStateTestAccess::capture(estimator, &error);
  return expect(snapshot == nullptr,
                "unknown live prior block rejects capture") &&
         expect(error.find("unknown") != std::string::npos,
                "unknown live prior block reports error");
}

bool featureQueueAssignsStrictSequences() {
  Estimator estimator;
  EstimatorBackendStateTestAccess::populate(&estimator);
  FeatureFrame frame;
  estimator.inputFeature(1.0, frame);
  estimator.inputFeature(1.1, frame);
  estimator.resetState();
  estimator.inputFeature(1.2, frame);
  const auto sequences =
      EstimatorBackendStateTestAccess::queuedSequences(&estimator);
  return expect(sequences.size() == 1,
                "reset clears queued feature frames") &&
         expect(sequences[0] == 3,
                "feature sequence remains monotonic across reset");
}

bool candidateResetIsDeferredWithoutClearingInputs() {
  Estimator estimator;
  EstimatorBackendStateTestAccess::populate(&estimator);
  FeatureFrame frame;
  estimator.inputFeature(2.0, frame);
  EstimatorBackendStateTestAccess::setCandidateWorkspaceActive(
      &estimator, true);
  estimator.resetState();
  const auto sequences =
      EstimatorBackendStateTestAccess::queuedSequences(&estimator);
  EstimatorBackendStateTestAccess::setCandidateWorkspaceActive(
      &estimator, false);
  return expect(
             EstimatorBackendStateTestAccess::candidateProcessingFailed(
                 estimator),
             "candidate reset becomes a discard signal") &&
         expect(EstimatorBackendStateTestAccess::frameCount(estimator) == 4,
                "candidate reset preserves live workspace for restore") &&
         expect(sequences.size() == 1 && sequences[0] == 1,
                "candidate reset does not clear global feature input");
}

bool candidateCameraPoseIsWithheld() {
  Estimator estimator;
  EstimatorBackendStateTestAccess::populate(&estimator);
  EstimatorBackendStateTestAccess::updateCameraX(&estimator, 1.0);
  PoseData pose;
  if (!expect(estimator.getCameraPose(0, pose),
              "committed camera pose is available") ||
      !expect(pose.position.x() == 1.0,
              "committed camera pose is published")) {
    return false;
  }
  EstimatorBackendStateTestAccess::setCandidateWorkspaceActive(
      &estimator, true);
  EstimatorBackendStateTestAccess::updateCameraX(&estimator, 9.0);
  EstimatorBackendStateTestAccess::setCandidateWorkspaceActive(
      &estimator, false);
  return expect(estimator.getCameraPose(0, pose),
                "withheld camera pose keeps prior output") &&
         expect(pose.position.x() == 1.0,
                "candidate camera pose cannot overwrite safe output");
}

}  // namespace

int main() {
  if (!candidateWorkspaceCanRestoreCommittedState() ||
      !captureRejectsUnknownLivePriorAddress() ||
      !featureQueueAssignsStrictSequences() ||
      !candidateResetIsDeferredWithoutClearingInputs() ||
      !candidateCameraPoseIsWithheld()) {
    return EXIT_FAILURE;
  }
  std::cout << "estimator_backend_state_adapter_test: PASS\n";
  return EXIT_SUCCESS;
}
