#include <vins/estimator/backend_state.h>

#include <cmath>
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

IMUData imu(double dt, double accel_x) {
  IMUData sample;
  sample.timestamp = dt;
  sample.linear_acceleration = Eigen::Vector3d(accel_x, 0.1, 9.8);
  sample.angular_velocity = Eigen::Vector3d(0.01, 0.02, 0.03);
  return sample;
}

IntegrationBase::Ptr populatedPreintegration() {
  ImuOptions options;
  options.accelNoiseDensity = 0.02;
  options.gyroNoiseDensity = 0.01;
  options.accelRandomWalk = 0.001;
  options.gyroRandomWalk = 0.001;
  auto result = std::make_shared<IntegrationBase>(
      imu(0.0, 0.0), Eigen::Vector3d(0.01, 0.02, 0.03),
      Eigen::Vector3d(0.001, 0.002, 0.003), options);
  result->push_back(imu(0.01, 0.2));
  result->push_back(imu(0.02, 0.4));
  return result;
}

void addFeature(FeatureManager *manager) {
  FeaturePerId feature(42, 1);
  Eigen::Matrix<double, 7, 1> observation;
  observation << 0.1, 0.2, 1.0, 100.0, 200.0, 1.0, 2.0;
  feature.feature_per_frame.emplace_back(observation, 0.0);
  observation(0) = 0.15;
  feature.feature_per_frame.emplace_back(observation, 0.0);
  feature.estimated_depth = 3.25;
  feature.solve_flag = FeaturePerId::SOLVED_SUCCESS;
  manager->feature.push_back(feature);
  manager->last_track_num = 1;
  manager->last_average_parallax = 4.0;
  manager->new_feature_num = 0;
  manager->long_track_num = 1;
}

void addPrior(vins::BackendState *state) {
  state->prior_parameter_blocks = {
      state->poseBlock(2), state->speedBiasBlock(2),
      state->extrinsicBlock(0), state->timeDelayBlock()};
  const int sizes[] = {SIZE_POSE, SIZE_SPEEDBIAS, SIZE_POSE, 1};
  const int indices[] = {0, 6, 15, 21};
  auto prior = std::make_shared<MarginalizationInfo>();
  prior->m = 0;
  prior->n = 22;
  prior->sum_block_size = 24;
  prior->keep_block_size.assign(std::begin(sizes), std::end(sizes));
  prior->keep_block_idx.assign(std::begin(indices), std::end(indices));
  prior->linearized_jacobians =
      Eigen::MatrixXd::Identity(prior->n, prior->n);
  prior->linearized_residuals =
      Eigen::VectorXd::LinSpaced(prior->n, 0.0, 2.1);
  for (std::size_t i = 0; i < state->prior_parameter_blocks.size(); ++i) {
    const long key =
        reinterpret_cast<long>(state->prior_parameter_blocks[i]);
    auto data = std::make_unique<double[]>(sizes[i]);
    std::copy(state->prior_parameter_blocks[i],
              state->prior_parameter_blocks[i] + sizes[i], data.get());
    prior->parameter_block_size[key] = sizes[i];
    prior->parameter_block_idx[key] = indices[i];
    prior->keep_block_data.push_back(data.get());
    prior->parameter_block_data[key] = std::move(data);
  }
  state->marginalization_prior = std::move(prior);
}

vins::BackendState populatedState() {
  vins::BackendState state;
  state.generation = 9;
  state.options = std::make_shared<VINSOptions>();
  state.options->RIC = {Eigen::Matrix3d::Identity(),
                        Eigen::Matrix3d::Identity()};
  state.options->TIC = {Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero()};
  state.options->num_of_camera = 1;
  state.options->time_delay = 0.004;
  state.feature_manager.setOptions(state.options);
  addFeature(&state.feature_manager);

  state.states[2].position = Eigen::Vector3d(1.0, 2.0, 3.0);
  state.states[2].velocity = Eigen::Vector3d(0.4, 0.5, 0.6);
  state.pose_parameters[2] = {{1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0}};
  state.speed_bias_parameters[2] =
      {{0.4, 0.5, 0.6, 0.01, 0.02, 0.03, 0.001, 0.002, 0.003}};
  state.extrinsic_parameters[0] =
      {{0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0}};
  state.time_delay_parameter[0] = 0.004;
  state.preintegrations[2] = populatedPreintegration();
  state.temporary_preintegration = populatedPreintegration();
  state.delta_time_imu_buffer[2] = {imu(0.01, 0.2), imu(0.02, 0.4)};
  state.rotor_dynamics_measurements[2].timestamp = 12.5;
  state.rotor_dynamics_measurements[2].total_thrust_newton = 15.2;
  state.rotor_dynamics_measurements[2].summed_rotor_speed_rad_s = 3200.0;
  state.rotor_dynamics_measurements[2].valid = true;
  state.gps_dropout_gyro_bias_anchor_initialized = true;
  state.gps_dropout_gyro_bias_anchor_z = 4.0e-5;

  ImageFrame frame;
  frame.t = 12.5;
  frame.R = Eigen::Matrix3d::Identity();
  frame.T = Eigen::Vector3d(1.0, 0.0, 0.0);
  frame.pre_integration = populatedPreintegration();
  frame.is_key_frame = true;
  state.image_frames.emplace(frame.t, frame);
  addPrior(&state);
  return state;
}

bool completeCloneDoesNotShareMutableStorage() {
  auto source = populatedState();
  std::string error;
  auto candidate = source.clone(&error);
  if (!expect(candidate != nullptr, error.c_str()) ||
      !expect(candidate->priorAddressesAreOwned(),
              "candidate prior addresses are candidate-owned") ||
      !expect(source.priorAddressesAreOwned(),
              "source prior addresses are source-owned")) {
    return false;
  }

  for (std::size_t i = 0; i < source.prior_parameter_blocks.size(); ++i) {
    if (!expect(source.prior_parameter_blocks[i] !=
                    candidate->prior_parameter_blocks[i],
                "prior parameter arrays are not shared") ||
        !expect(source.marginalization_prior->keep_block_data[i] !=
                    candidate->marginalization_prior->keep_block_data[i],
                "prior linearization data are not shared")) {
      return false;
    }
  }
  if (!expect(source.preintegrations[2].get() !=
                  candidate->preintegrations[2].get(),
              "window preintegration is not shared") ||
      !expect(source.temporary_preintegration.get() !=
                  candidate->temporary_preintegration.get(),
              "temporary preintegration is not shared") ||
      !expect(source.image_frames.at(12.5).pre_integration.get() !=
                  candidate->image_frames.at(12.5).pre_integration.get(),
              "image-history preintegration is not shared") ||
      !expect(source.options.get() != candidate->options.get(),
              "mutable options are not shared") ||
      !expect(candidate->rotor_dynamics_measurements[2].valid &&
                  candidate->rotor_dynamics_measurements[2]
                          .total_thrust_newton == 15.2,
              "rotor dynamics window measurement is cloned") ||
      !expect(candidate->gps_dropout_gyro_bias_anchor_initialized &&
                  candidate->gps_dropout_gyro_bias_anchor_z == 4.0e-5,
              "GPS-dropout gyro-bias anchor is cloned")) {
    return false;
  }

  candidate->states[2].position.x() = 99.0;
  candidate->feature_manager.feature.front().estimated_depth = 99.0;
  candidate->preintegrations[2]->imu_buffer.front().linear_acceleration.x() =
      99.0;
  candidate->image_frames.at(12.5)
      .pre_integration->imu_buffer.front().linear_acceleration.x() = 88.0;
  candidate->marginalization_prior->keep_block_data[0][0] = 77.0;
  candidate->pose_parameters[2][0] = 66.0;
  candidate->options->time_delay = 0.5;
  candidate->rotor_dynamics_measurements[2].total_thrust_newton = 99.0;

  return expect(source.states[2].position.x() == 1.0,
                "source window state remains unchanged") &&
         expect(source.feature_manager.feature.front().estimated_depth ==
                    3.25,
                "source feature depth remains unchanged") &&
         expect(source.preintegrations[2]
                        ->imu_buffer.front().linear_acceleration.x() == 0.2,
                "source preintegration raw buffer remains unchanged") &&
         expect(source.image_frames.at(12.5)
                        .pre_integration->imu_buffer.front()
                        .linear_acceleration.x() == 0.2,
                "source image-history preintegration remains unchanged") &&
         expect(source.marginalization_prior->keep_block_data[0][0] == 1.0,
                "source prior linearization point remains unchanged") &&
         expect(source.pose_parameters[2][0] == 1.0,
                "source parameter array remains unchanged") &&
         expect(source.options->time_delay == 0.004,
                "source options remain unchanged") &&
         expect(source.rotor_dynamics_measurements[2]
                        .total_thrust_newton == 15.2,
                "source rotor dynamics measurement remains unchanged");
}

bool clonedPriorEvaluatesEquivalently() {
  auto source = populatedState();
  std::string error;
  auto candidate = source.clone(&error);
  if (!expect(candidate != nullptr, error.c_str())) return false;

  MarginalizationFactor source_factor(source.marginalization_prior);
  MarginalizationFactor candidate_factor(candidate->marginalization_prior);
  Eigen::VectorXd source_residual(source.marginalization_prior->n);
  Eigen::VectorXd candidate_residual(candidate->marginalization_prior->n);
  if (!source_factor.Evaluate(source.prior_parameter_blocks.data(),
                              source_residual.data(), nullptr) ||
      !candidate_factor.Evaluate(candidate->prior_parameter_blocks.data(),
                                 candidate_residual.data(), nullptr)) {
    return expect(false, "both priors evaluate");
  }
  return expect(
      (source_residual - candidate_residual).norm() < 1e-12,
      "cloned and source prior residuals are identical");
}

bool unknownPriorAddressFailsClosed() {
  auto source = populatedState();
  double external_block[1] = {0.0};
  source.prior_parameter_blocks[0] = external_block;
  std::string error;
  auto candidate = source.clone(&error);
  return expect(candidate == nullptr,
                "unknown prior address rejects clone") &&
         expect(error.find("not owned") != std::string::npos,
                "unknown prior address reports ownership failure");
}

}  // namespace

int main() {
  if (!completeCloneDoesNotShareMutableStorage() ||
      !clonedPriorEvaluatesEquivalently() ||
      !unknownPriorAddressFailsClosed()) {
    return EXIT_FAILURE;
  }
  std::cout << "backend_state_test: PASS\n";
  return EXIT_SUCCESS;
}
