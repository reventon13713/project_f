#pragma once

#include <ceres/ceres.h>

#include <algorithm>
#include <cmath>

// Horizontal velocity-transition factor for a quadrotor point-mass model.
// Unlike a desired mission velocity, measured rotor speed and aerodynamic
// drag provide an independent physical constraint on metric body velocity.
struct RotorDragDynamicsFactor {
  RotorDragDynamicsFactor(double dt, double thrust_i, double rotor_speed_i,
                          double thrust_j, double rotor_speed_j,
                          double mass_kg, double rotor_drag_coefficient,
                          double accel_sigma)
      : dt_(dt),
        thrust_i_(thrust_i),
        rotor_speed_i_(rotor_speed_i),
        thrust_j_(thrust_j),
        rotor_speed_j_(rotor_speed_j),
        inverse_mass_(1.0 / mass_kg),
        rotor_drag_coefficient_(rotor_drag_coefficient),
        inverse_accel_sigma_(1.0 / accel_sigma) {}

  template <typename T>
  bool operator()(const T *const pose_i, const T *const speed_bias_i,
                  const T *const pose_j, const T *const speed_bias_j,
                  T *residuals) const {
    const Eigen::Quaternion<T> q_i(pose_i[6], pose_i[3], pose_i[4],
                                   pose_i[5]);
    const Eigen::Quaternion<T> q_j(pose_j[6], pose_j[3], pose_j[4],
                                   pose_j[5]);
    const Eigen::Matrix<T, 3, 1> velocity_i(
        speed_bias_i[0], speed_bias_i[1], speed_bias_i[2]);
    const Eigen::Matrix<T, 3, 1> velocity_j(
        speed_bias_j[0], speed_bias_j[1], speed_bias_j[2]);

    const auto acceleration =
        [this](const Eigen::Quaternion<T> &q,
               const Eigen::Matrix<T, 3, 1> &velocity_world,
               double thrust, double summed_rotor_speed) {
          const Eigen::Matrix<T, 3, 1> velocity_body =
              q.conjugate() * velocity_world;
          Eigen::Matrix<T, 3, 1> force_body;
          const T drag_scale =
              T(rotor_drag_coefficient_ * summed_rotor_speed);
          force_body << -drag_scale * velocity_body.x(),
              -drag_scale * velocity_body.y(), T(thrust);
          return q * (force_body * T(inverse_mass_));
        };

    const Eigen::Matrix<T, 3, 1> accel_i =
        acceleration(q_i, velocity_i, thrust_i_, rotor_speed_i_);
    const Eigen::Matrix<T, 3, 1> accel_j =
        acceleration(q_j, velocity_j, thrust_j_, rotor_speed_j_);
    const Eigen::Matrix<T, 3, 1> measured_accel =
        (velocity_j - velocity_i) / T(dt_);
    const Eigen::Matrix<T, 3, 1> predicted_accel =
        T(0.5) * (accel_i + accel_j);

    residuals[0] =
        (measured_accel.x() - predicted_accel.x()) *
        T(inverse_accel_sigma_);
    residuals[1] =
        (measured_accel.y() - predicted_accel.y()) *
        T(inverse_accel_sigma_);
    return true;
  }

  static ceres::CostFunction *Create(
      double dt, double thrust_i, double rotor_speed_i, double thrust_j,
      double rotor_speed_j, double mass_kg,
      double rotor_drag_coefficient, double accel_sigma) {
    return new ceres::AutoDiffCostFunction<
        RotorDragDynamicsFactor, 2, 7, 9, 7, 9>(
        new RotorDragDynamicsFactor(
            dt, thrust_i, rotor_speed_i, thrust_j, rotor_speed_j, mass_kg,
            rotor_drag_coefficient, accel_sigma));
  }

 private:
  double dt_;
  double thrust_i_;
  double rotor_speed_i_;
  double thrust_j_;
  double rotor_speed_j_;
  double inverse_mass_;
  double rotor_drag_coefficient_;
  double inverse_accel_sigma_;
};
