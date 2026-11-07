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
#include <vins/estimator/parameters.h>
#include <vins/factor/integration_base.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace vins {

struct BackendHealthMetrics {
  bool finite = true;
  double position_innovation_m =
      std::numeric_limits<double>::quiet_NaN();
  double velocity_innovation_mps =
      std::numeric_limits<double>::quiet_NaN();
  double window_kinematic_rms_mps =
      std::numeric_limits<double>::quiet_NaN();
  double pose_scale_ratio =
      std::numeric_limits<double>::quiet_NaN();
  double velocity_scale_ratio =
      std::numeric_limits<double>::quiet_NaN();
  double landmark_depth_scale_ratio =
      std::numeric_limits<double>::quiet_NaN();
  double maximum_scale_log_change =
      std::numeric_limits<double>::quiet_NaN();
  double scale_log_spread =
      std::numeric_limits<double>::quiet_NaN();
  int kinematic_intervals = 0;
  int landmark_depths = 0;
  int usable_scale_components = 0;
};

class BackendHealthMetricCalculator {
 public:
  static BackendHealthMetrics compute(
      const StateData *entry_states, const StateData *optimized_states,
      int frame_count,
      IntegrationBase::Ptr const *preintegrations,
      const std::vector<double> &entry_inverse_depths,
      const double (*optimized_inverse_depths)[SIZE_FEATURE],
      int optimized_depth_count) {
    BackendHealthMetrics result;
    if (!entry_states || !optimized_states || frame_count < 0 ||
        frame_count > WINDOW_SIZE) {
      result.finite = false;
      return result;
    }

    for (int i = 0; i <= frame_count; ++i) {
      result.finite =
          result.finite && stateFinite(entry_states[i]) &&
          stateFinite(optimized_states[i]);
    }
    result.position_innovation_m =
        (optimized_states[frame_count].position -
         entry_states[frame_count].position)
            .norm();
    result.velocity_innovation_mps =
        (optimized_states[frame_count].velocity -
         entry_states[frame_count].velocity)
            .norm();

    double kinematic_squared_sum = 0.0;
    if (preintegrations) {
      for (int i = 1; i <= frame_count; ++i) {
        if (!preintegrations[i]) continue;
        const double dt = preintegrations[i]->sum_dt;
        if (!std::isfinite(dt) || dt <= 1e-6 || dt >= 10.0) continue;
        const Eigen::Vector3d finite_difference_velocity =
            (optimized_states[i].position -
             optimized_states[i - 1].position) /
            dt;
        const Eigen::Vector3d mean_velocity =
            0.5 * (optimized_states[i].velocity +
                   optimized_states[i - 1].velocity);
        kinematic_squared_sum +=
            (finite_difference_velocity - mean_velocity).squaredNorm();
        ++result.kinematic_intervals;
      }
    }
    if (result.kinematic_intervals > 0) {
      result.window_kinematic_rms_mps =
          std::sqrt(kinematic_squared_sum /
                    result.kinematic_intervals);
    }

    const Eigen::Vector3d entry_anchor = entry_states[0].position;
    const Eigen::Vector3d optimized_anchor =
        optimized_states[0].position;
    double pose_denominator = 0.0;
    double pose_numerator = 0.0;
    double velocity_denominator = 0.0;
    double velocity_numerator = 0.0;
    for (int i = 1; i <= frame_count; ++i) {
      const Eigen::Vector3d entry_offset =
          entry_states[i].position - entry_anchor;
      const Eigen::Vector3d optimized_offset =
          optimized_states[i].position - optimized_anchor;
      pose_denominator += entry_offset.squaredNorm();
      pose_numerator += entry_offset.dot(optimized_offset);
    }
    for (int i = 0; i <= frame_count; ++i) {
      velocity_denominator +=
          entry_states[i].velocity.squaredNorm();
      velocity_numerator += entry_states[i].velocity.dot(
          optimized_states[i].velocity);
    }
    if (pose_denominator > 1e-8) {
      result.pose_scale_ratio = pose_numerator / pose_denominator;
    }
    if (velocity_denominator > 1e-8) {
      result.velocity_scale_ratio =
          velocity_numerator / velocity_denominator;
    }

    std::vector<double> depth_ratios;
    const int depth_count = std::min(
        static_cast<int>(entry_inverse_depths.size()),
        std::max(0, optimized_depth_count));
    depth_ratios.reserve(depth_count);
    for (int i = 0; i < depth_count; ++i) {
      const double entry_inverse_depth = entry_inverse_depths[i];
      const double optimized_inverse_depth =
          optimized_inverse_depths ? optimized_inverse_depths[i][0]
                                   : std::numeric_limits<double>::quiet_NaN();
      if (std::isfinite(entry_inverse_depth) &&
          std::isfinite(optimized_inverse_depth) &&
          entry_inverse_depth > 1e-9 &&
          optimized_inverse_depth > 1e-9) {
        depth_ratios.push_back(entry_inverse_depth /
                               optimized_inverse_depth);
      }
    }
    result.landmark_depths = static_cast<int>(depth_ratios.size());
    if (!depth_ratios.empty()) {
      const std::size_t middle = depth_ratios.size() / 2;
      std::nth_element(depth_ratios.begin(),
                       depth_ratios.begin() + middle,
                       depth_ratios.end());
      result.landmark_depth_scale_ratio = depth_ratios[middle];
    }

    std::vector<double> scale_logs;
    addScaleLog(result.pose_scale_ratio, &scale_logs);
    addScaleLog(result.velocity_scale_ratio, &scale_logs);
    addScaleLog(result.landmark_depth_scale_ratio, &scale_logs);
    result.usable_scale_components =
        static_cast<int>(scale_logs.size());
    if (!scale_logs.empty()) {
      const auto limits =
          std::minmax_element(scale_logs.begin(), scale_logs.end());
      result.scale_log_spread = *limits.second - *limits.first;
      result.maximum_scale_log_change = 0.0;
      for (double value : scale_logs) {
        result.maximum_scale_log_change =
            std::max(result.maximum_scale_log_change, std::abs(value));
      }
    }
    result.finite =
        result.finite && std::isfinite(result.position_innovation_m) &&
        std::isfinite(result.velocity_innovation_mps);
    return result;
  }

 private:
  static bool stateFinite(const StateData &state) {
    return std::isfinite(state.timestamp) &&
           state.position.allFinite() && state.velocity.allFinite() &&
           state.rotation.allFinite() && state.accel_bias.allFinite() &&
           state.gyro_bias.allFinite();
  }

  static void addScaleLog(double ratio, std::vector<double> *logs) {
    if (logs && std::isfinite(ratio) && ratio > 1e-9) {
      logs->push_back(std::log(ratio));
    }
  }
};

}  // namespace vins
