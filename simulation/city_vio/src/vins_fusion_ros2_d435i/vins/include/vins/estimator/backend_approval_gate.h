/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 * and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0.
 *******************************************************/

#pragma once

#include <vins/estimator/backend_health_metrics.h>

#include <cstddef>

namespace vins {

struct BackendApprovalThresholds {
  // Fixed from 2,822 accepted, pre-anomaly updates across the saved
  // endurance and patrol 1x single-source full-sensor fixtures.
  double maximum_position_innovation_m = 0.10;
  double maximum_velocity_innovation_mps = 0.15;
  double maximum_window_kinematic_rms_mps = 0.05;
  double maximum_scale_log_change = 0.07;
  double maximum_scale_log_spread = 0.04;
  int minimum_kinematic_intervals = 8;
  int minimum_landmark_depths = 15;
  int required_scale_components = 3;
};

struct BackendApprovalResult {
  bool healthy = false;
  bool finite = false;
  bool solver_usable = false;
  bool position_bounded = false;
  bool velocity_bounded = false;
  bool kinematics_bounded = false;
  bool scale_change_bounded = false;
  bool scale_spread_bounded = false;
  bool support_sufficient = false;
  bool reject_chain_not_worsening = false;
};

class BackendApprovalGate {
 public:
  explicit BackendApprovalGate(
      BackendApprovalThresholds thresholds = BackendApprovalThresholds())
      : thresholds_(thresholds) {}

  BackendApprovalResult evaluate(
      const BackendHealthMetrics &metrics, bool solver_usable,
      int previous_persistent_rejects,
      int current_persistent_rejects) const {
    BackendApprovalResult result;
    result.finite = metrics.finite;
    result.solver_usable = solver_usable;
    result.position_bounded =
        metrics.position_innovation_m <=
        thresholds_.maximum_position_innovation_m;
    result.velocity_bounded =
        metrics.velocity_innovation_mps <=
        thresholds_.maximum_velocity_innovation_mps;
    result.kinematics_bounded =
        metrics.kinematic_intervals >=
            thresholds_.minimum_kinematic_intervals &&
        metrics.window_kinematic_rms_mps <=
            thresholds_.maximum_window_kinematic_rms_mps;
    result.scale_change_bounded =
        metrics.maximum_scale_log_change <=
        thresholds_.maximum_scale_log_change;
    result.scale_spread_bounded =
        metrics.scale_log_spread <=
        thresholds_.maximum_scale_log_spread;
    result.support_sufficient =
        metrics.landmark_depths >= thresholds_.minimum_landmark_depths &&
        metrics.usable_scale_components ==
            thresholds_.required_scale_components;
    result.reject_chain_not_worsening =
        current_persistent_rejects <= previous_persistent_rejects;
    result.healthy =
        result.finite && result.solver_usable &&
        result.position_bounded && result.velocity_bounded &&
        result.kinematics_bounded && result.scale_change_bounded &&
        result.scale_spread_bounded && result.support_sufficient &&
        result.reject_chain_not_worsening;
    return result;
  }

  const BackendApprovalThresholds &thresholds() const {
    return thresholds_;
  }

 private:
  BackendApprovalThresholds thresholds_;
};

}  // namespace vins
