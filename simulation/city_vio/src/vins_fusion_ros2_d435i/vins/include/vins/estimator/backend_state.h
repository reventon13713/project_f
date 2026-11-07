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
#include <vins/estimator/feature_manager.h>
#include <vins/factor/imu_factor.h>
#include <vins/factor/marginalization_factor.h>
#include <vins/initial/initial_alignment.h>

#include <array>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace vins {

// Complete, owning storage unit for one optimization backend.  It deliberately
// excludes ROS queues, publishers, mutexes, threads and the live feature
// tracker.  Production routing is added only after this storage type and its
// prior-address invariants are independently proven.
class BackendState {
 public:
  using Ptr = std::unique_ptr<BackendState>;

  BackendState() = default;
  BackendState(const BackendState &) = delete;
  BackendState &operator=(const BackendState &) = delete;
  BackendState(BackendState &&) = default;
  BackendState &operator=(BackendState &&) = default;

  Ptr clone(std::string *error = nullptr) const {
    auto result = Ptr(new BackendState);
    result->generation = generation;
    result->previous_timestamp = previous_timestamp;
    result->current_timestamp = current_timestamp;
    result->initial_timestamp = initial_timestamp;
    result->open_ex_estimation = open_ex_estimation;
    result->solver_flag = solver_flag;
    result->marginalization_flag = marginalization_flag;
    result->gravity = gravity;
    result->camera_rotation = camera_rotation;
    result->camera_translation = camera_translation;
    result->states = states;
    result->back_state = back_state;
    result->last_state = last_state;
    result->last_state0 = last_state0;
    result->latest_state = latest_state;
    result->range_measurements = range_measurements;
    result->metric_velocity_measurements =
        metric_velocity_measurements;
    result->rotor_dynamics_measurements =
        rotor_dynamics_measurements;
    result->metric_displacement_anchor_initialized =
        metric_displacement_anchor_initialized;
    result->metric_displacement_anchor_segment =
        metric_displacement_anchor_segment;
    result->metric_displacement_anchor_measurement =
        metric_displacement_anchor_measurement;
    result->metric_displacement_anchor_position =
        metric_displacement_anchor_position;
    result->gps_dropout_gyro_bias_anchor_initialized =
        gps_dropout_gyro_bias_anchor_initialized;
    result->gps_dropout_gyro_bias_anchor_z =
        gps_dropout_gyro_bias_anchor_z;
    result->previous_imu_data = previous_imu_data;
    result->latest_imu_data = latest_imu_data;
    result->delta_time_imu_buffer = delta_time_imu_buffer;
    result->frame_count = frame_count;
    result->back_count = back_count;
    result->front_count = front_count;
    result->nonlinear_update_count = nonlinear_update_count;
    result->stationary_visual_update_count = stationary_visual_update_count;
    result->consecutive_position_innovation_rejects =
        consecutive_position_innovation_rejects;
    result->is_first_imu_received = is_first_imu_received;
    result->failure_occurred = failure_occurred;
    result->range_ground_initialized = range_ground_initialized;
    result->range_ground_z = range_ground_z;
    result->last_published_alignment_scale =
        last_published_alignment_scale;
    result->current_alignment_scale = current_alignment_scale;
    result->current_feature_flow_median_px_s =
        current_feature_flow_median_px_s;
    result->current_feature_flow_count = current_feature_flow_count;
    result->last_optimization_initial_cost =
        last_optimization_initial_cost;
    result->last_optimization_final_cost = last_optimization_final_cost;
    result->last_optimization_time_seconds =
        last_optimization_time_seconds;
    result->last_optimization_successful_steps =
        last_optimization_successful_steps;
    result->last_optimization_unsuccessful_steps =
        last_optimization_unsuccessful_steps;
    result->last_optimization_residual_blocks =
        last_optimization_residual_blocks;
    result->last_optimization_residuals = last_optimization_residuals;
    result->last_optimization_termination =
        last_optimization_termination;
    result->last_marginalization_source_timestamp =
        last_marginalization_source_timestamp;
    result->last_marginalization_source_type =
        last_marginalization_source_type;
    result->latest_orientation = latest_orientation;
    result->is_first_pose_initialized = is_first_pose_initialized;
    result->pose_parameters = pose_parameters;
    result->speed_bias_parameters = speed_bias_parameters;
    result->feature_parameters = feature_parameters;
    result->extrinsic_parameters = extrinsic_parameters;
    result->time_delay_parameter = time_delay_parameter;
    result->range_parameter = range_parameter;
    result->last_solved_pose_parameters = last_solved_pose_parameters;
    result->last_solved_speed_bias_parameters =
        last_solved_speed_bias_parameters;
    result->last_solved_feature_parameters =
        last_solved_feature_parameters;
    result->last_solved_extrinsic_parameters =
        last_solved_extrinsic_parameters;
    result->last_solved_time_delay_parameter =
        last_solved_time_delay_parameter;
    result->last_solved_feature_count = last_solved_feature_count;
    result->last_solved_parameters_valid =
        last_solved_parameters_valid;

    if (options) {
      result->options = std::make_shared<VINSOptions>(*options);
    }
    result->feature_manager =
        feature_manager.cloneWithOptions(result->options);

    for (std::size_t i = 0; i < preintegrations.size(); ++i) {
      if (preintegrations[i]) {
        result->preintegrations[i] = preintegrations[i]->clone();
      }
    }
    if (temporary_preintegration) {
      result->temporary_preintegration =
          temporary_preintegration->clone();
    }
    for (const auto &entry : image_frames) {
      ImageFrame frame = entry.second;
      if (entry.second.pre_integration) {
        frame.pre_integration =
            entry.second.pre_integration->clone();
      }
      result->image_frames.emplace(entry.first, std::move(frame));
    }

    result->prior_parameter_blocks.reserve(
        prior_parameter_blocks.size());
    for (double *source_block : prior_parameter_blocks) {
      double *cloned_block = mapOwnedBlock(source_block, result.get());
      if (!cloned_block) {
        if (error) {
          *error =
              "prior references a parameter block not owned by BackendState";
        }
        return nullptr;
      }
      result->prior_parameter_blocks.push_back(cloned_block);
    }
    if (marginalization_prior) {
      result->marginalization_prior =
          marginalization_prior->cloneRebound(
              prior_parameter_blocks, result->prior_parameter_blocks, error);
      if (!result->marginalization_prior) return nullptr;
    } else if (!prior_parameter_blocks.empty()) {
      if (error) {
        *error = "prior parameter blocks exist without a prior";
      }
      return nullptr;
    }
    if (!result->priorAddressesAreOwned()) {
      if (error) *error = "cloned prior address ownership check failed";
      return nullptr;
    }
    if (error) error->clear();
    return result;
  }

  bool ownsParameterBlock(const double *block) const {
    if (!block) return false;
    for (const auto &value : pose_parameters)
      if (block == value.data()) return true;
    for (const auto &value : speed_bias_parameters)
      if (block == value.data()) return true;
    for (const auto &value : feature_parameters)
      if (block == value.data()) return true;
    for (const auto &value : extrinsic_parameters)
      if (block == value.data()) return true;
    return block == time_delay_parameter.data() ||
           block == range_parameter.data();
  }

  bool priorAddressesAreOwned() const {
    for (double *block : prior_parameter_blocks) {
      if (!ownsParameterBlock(block)) return false;
    }
    return !marginalization_prior ||
           marginalization_prior->ownsLinearizationDataFor(
               prior_parameter_blocks);
  }

  double *poseBlock(std::size_t index) {
    return pose_parameters.at(index).data();
  }
  double *speedBiasBlock(std::size_t index) {
    return speed_bias_parameters.at(index).data();
  }
  double *featureBlock(std::size_t index) {
    return feature_parameters.at(index).data();
  }
  double *extrinsicBlock(std::size_t index) {
    return extrinsic_parameters.at(index).data();
  }
  double *timeDelayBlock() { return time_delay_parameter.data(); }
  double *rangeBlock() { return range_parameter.data(); }

  std::uint64_t generation = 0;
  Timestamp previous_timestamp = -1.0;
  Timestamp current_timestamp = 0.0;
  Timestamp initial_timestamp = 0.0;
  bool open_ex_estimation = false;
  SolverState solver_flag = SolverState::INITIAL;
  MarginalizationType marginalization_flag =
      MarginalizationType::MARGIN_OLD;
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  std::array<Eigen::Matrix3d, 2> camera_rotation{
      {Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity()}};
  std::array<Eigen::Vector3d, 2> camera_translation{
      {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()}};
  std::array<StateData, WINDOW_SIZE + 1> states;
  StateData back_state;
  StateData last_state;
  StateData last_state0;
  StateData latest_state;
  std::array<RangeData, WINDOW_SIZE + 1> range_measurements;
  std::array<MetricVelocityData, WINDOW_SIZE + 1>
      metric_velocity_measurements;
  std::array<RotorDynamicsData, WINDOW_SIZE + 1>
      rotor_dynamics_measurements;
  bool metric_displacement_anchor_initialized = false;
  uint64_t metric_displacement_anchor_segment = 0;
  Eigen::Vector2d metric_displacement_anchor_measurement =
      Eigen::Vector2d::Zero();
  Eigen::Vector2d metric_displacement_anchor_position =
      Eigen::Vector2d::Zero();
  bool gps_dropout_gyro_bias_anchor_initialized = false;
  double gps_dropout_gyro_bias_anchor_z = 0.0;
  std::array<IntegrationBase::Ptr, WINDOW_SIZE + 1> preintegrations;
  IMUData previous_imu_data;
  IMUData latest_imu_data;
  std::array<std::vector<IMUData>, WINDOW_SIZE + 1>
      delta_time_imu_buffer;
  IntegrationBase::Ptr temporary_preintegration;
  std::map<double, ImageFrame> image_frames;
  FeatureManager feature_manager;
  std::shared_ptr<VINSOptions> options;

  int frame_count = 0;
  int back_count = 0;
  int front_count = 0;
  int nonlinear_update_count = 0;
  int stationary_visual_update_count = 0;
  int consecutive_position_innovation_rejects = 0;
  bool is_first_imu_received = false;
  bool failure_occurred = false;
  bool range_ground_initialized = false;
  double range_ground_z = 0.0;
  double last_published_alignment_scale = -1.0;
  double current_alignment_scale = -1.0;
  double current_feature_flow_median_px_s =
      std::numeric_limits<double>::infinity();
  int current_feature_flow_count = 0;
  double last_optimization_initial_cost = 0.0;
  double last_optimization_final_cost = 0.0;
  double last_optimization_time_seconds = 0.0;
  int last_optimization_successful_steps = 0;
  int last_optimization_unsuccessful_steps = 0;
  int last_optimization_residual_blocks = 0;
  int last_optimization_residuals = 0;
  ceres::TerminationType last_optimization_termination = ceres::FAILURE;
  double last_marginalization_source_timestamp =
      std::numeric_limits<double>::quiet_NaN();
  std::string last_marginalization_source_type = "none";
  Eigen::Quaterniond latest_orientation =
      Eigen::Quaterniond::Identity();
  bool is_first_pose_initialized = false;

  std::array<std::array<double, SIZE_POSE>, WINDOW_SIZE + 1>
      pose_parameters{};
  std::array<std::array<double, SIZE_SPEEDBIAS>, WINDOW_SIZE + 1>
      speed_bias_parameters{};
  std::array<std::array<double, SIZE_FEATURE>, NUM_OF_F>
      feature_parameters{};
  std::array<std::array<double, SIZE_POSE>, 2> extrinsic_parameters{};
  std::array<double, 1> time_delay_parameter{};
  std::array<double, 1> range_parameter{};
  std::array<std::array<double, SIZE_POSE>, WINDOW_SIZE + 1>
      last_solved_pose_parameters{};
  std::array<std::array<double, SIZE_SPEEDBIAS>, WINDOW_SIZE + 1>
      last_solved_speed_bias_parameters{};
  std::array<std::array<double, SIZE_FEATURE>, NUM_OF_F>
      last_solved_feature_parameters{};
  std::array<std::array<double, SIZE_POSE>, 2>
      last_solved_extrinsic_parameters{};
  std::array<double, 1> last_solved_time_delay_parameter{};
  int last_solved_feature_count = 0;
  bool last_solved_parameters_valid = false;
  MarginalizationInfo::Ptr marginalization_prior;
  std::vector<double *> prior_parameter_blocks;

 private:
  double *mapOwnedBlock(const double *source, BackendState *destination) const {
    if (!source || !destination) return nullptr;
    for (std::size_t i = 0; i < pose_parameters.size(); ++i) {
      if (source == pose_parameters[i].data())
        return destination->pose_parameters[i].data();
    }
    for (std::size_t i = 0; i < speed_bias_parameters.size(); ++i) {
      if (source == speed_bias_parameters[i].data())
        return destination->speed_bias_parameters[i].data();
    }
    for (std::size_t i = 0; i < feature_parameters.size(); ++i) {
      if (source == feature_parameters[i].data())
        return destination->feature_parameters[i].data();
    }
    for (std::size_t i = 0; i < extrinsic_parameters.size(); ++i) {
      if (source == extrinsic_parameters[i].data())
        return destination->extrinsic_parameters[i].data();
    }
    if (source == time_delay_parameter.data())
      return destination->time_delay_parameter.data();
    if (source == range_parameter.data())
      return destination->range_parameter.data();
    return nullptr;
  }
};

}  // namespace vins
