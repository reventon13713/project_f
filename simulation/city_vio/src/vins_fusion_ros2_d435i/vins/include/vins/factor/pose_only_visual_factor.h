#pragma once

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace pose_only_visual {

template <typename T>
inline void cameraPose(
    const T *pose, const Eigen::Quaterniond &q_imu_camera,
    const Eigen::Vector3d &t_imu_camera,
    Eigen::Quaternion<T> *q_world_camera,
    Eigen::Matrix<T, 3, 1> *p_world_camera) {
  const Eigen::Quaternion<T> q_world_imu(
      pose[6], pose[3], pose[4], pose[5]);
  const Eigen::Matrix<T, 3, 1> p_world_imu(
      pose[0], pose[1], pose[2]);
  const Eigen::Quaternion<T> q_ic =
      q_imu_camera.template cast<T>();
  *q_world_camera = q_world_imu * q_ic;
  *p_world_camera =
      p_world_imu +
      q_world_imu * t_imu_camera.template cast<T>();
}

struct EpipolarFunctor {
  EpipolarFunctor(
      const Eigen::Vector3d &bearing_j,
      const Eigen::Vector3d &bearing_k,
      const Eigen::Quaterniond &q_imu_camera,
      const Eigen::Vector3d &t_imu_camera,
      double sqrt_information)
      : bearing_j_(bearing_j),
        bearing_k_(bearing_k),
        q_imu_camera_(q_imu_camera),
        t_imu_camera_(t_imu_camera),
        sqrt_information_(sqrt_information) {}

  template <typename T>
  bool operator()(const T *pose_j, const T *pose_k,
                  T *residual) const {
    Eigen::Quaternion<T> q_wcj, q_wck;
    Eigen::Matrix<T, 3, 1> p_wcj, p_wck;
    cameraPose(
        pose_j, q_imu_camera_, t_imu_camera_, &q_wcj, &p_wcj);
    cameraPose(
        pose_k, q_imu_camera_, t_imu_camera_, &q_wck, &p_wck);
    const Eigen::Quaternion<T> q_k_j = q_wck.conjugate() * q_wcj;
    const Eigen::Matrix<T, 3, 1> t_j_in_k =
        q_wck.conjugate() * (p_wcj - p_wck);
    const Eigen::Matrix<T, 3, 1> bearing_j =
        bearing_j_.template cast<T>();
    const Eigen::Matrix<T, 3, 1> bearing_k =
        bearing_k_.template cast<T>();
    const T translation_norm =
        ceres::sqrt(t_j_in_k.squaredNorm() + T(1e-12));
    residual[0] = T(sqrt_information_) *
        bearing_k.dot(t_j_in_k.cross(q_k_j * bearing_j)) /
        translation_norm;
    return true;
  }

  static ceres::CostFunction *Create(
      const Eigen::Vector3d &bearing_j,
      const Eigen::Vector3d &bearing_k,
      const Eigen::Quaterniond &q_imu_camera,
      const Eigen::Vector3d &t_imu_camera,
      double sqrt_information) {
    return new ceres::AutoDiffCostFunction<
        EpipolarFunctor, 1, 7, 7>(
        new EpipolarFunctor(
            bearing_j, bearing_k, q_imu_camera, t_imu_camera,
            sqrt_information));
  }

  Eigen::Vector3d bearing_j_;
  Eigen::Vector3d bearing_k_;
  Eigen::Quaterniond q_imu_camera_;
  Eigen::Vector3d t_imu_camera_;
  double sqrt_information_;
};

struct ThreeViewFunctor {
  ThreeViewFunctor(
      const Eigen::Vector3d &bearing_j,
      const Eigen::Vector3d &bearing_k,
      const Eigen::Vector3d &bearing_i,
      const Eigen::Quaterniond &q_imu_camera,
      const Eigen::Vector3d &t_imu_camera,
      double sqrt_information)
      : bearing_j_(bearing_j),
        bearing_k_(bearing_k),
        bearing_i_(bearing_i),
        q_imu_camera_(q_imu_camera),
        t_imu_camera_(t_imu_camera),
        sqrt_information_(sqrt_information) {}

  template <typename T>
  bool operator()(const T *pose_j, const T *pose_k, const T *pose_i,
                  T *residual) const {
    Eigen::Quaternion<T> q_wcj, q_wck, q_wci;
    Eigen::Matrix<T, 3, 1> p_wcj, p_wck, p_wci;
    cameraPose(
        pose_j, q_imu_camera_, t_imu_camera_, &q_wcj, &p_wcj);
    cameraPose(
        pose_k, q_imu_camera_, t_imu_camera_, &q_wck, &p_wck);
    cameraPose(
        pose_i, q_imu_camera_, t_imu_camera_, &q_wci, &p_wci);
    const Eigen::Quaternion<T> q_k_j = q_wck.conjugate() * q_wcj;
    const Eigen::Quaternion<T> q_i_j = q_wci.conjugate() * q_wcj;
    const Eigen::Matrix<T, 3, 1> t_j_in_k =
        q_wck.conjugate() * (p_wcj - p_wck);
    const Eigen::Matrix<T, 3, 1> t_j_in_i =
        q_wci.conjugate() * (p_wcj - p_wci);
    const Eigen::Matrix<T, 3, 1> bearing_j =
        bearing_j_.template cast<T>();
    const Eigen::Matrix<T, 3, 1> bearing_k =
        bearing_k_.template cast<T>();
    const Eigen::Matrix<T, 3, 1> bearing_i =
        bearing_i_.template cast<T>();
    const T base_translation_weight =
        ceres::sqrt(t_j_in_k.cross(bearing_k).squaredNorm() + T(1e-12));
    const T base_ray_weight =
        ceres::sqrt(
            bearing_k.cross(q_k_j * bearing_j).squaredNorm() +
            T(1e-12));
    const Eigen::Matrix<T, 3, 1> point_i =
        base_translation_weight * (q_i_j * bearing_j) +
        base_ray_weight * t_j_in_i;
    const T inverse_depth = T(1.0) / (point_i.z() + T(1e-12));
    residual[0] = T(sqrt_information_) *
        (point_i.x() * inverse_depth - bearing_i.x());
    residual[1] = T(sqrt_information_) *
        (point_i.y() * inverse_depth - bearing_i.y());
    return true;
  }

  static ceres::CostFunction *Create(
      const Eigen::Vector3d &bearing_j,
      const Eigen::Vector3d &bearing_k,
      const Eigen::Vector3d &bearing_i,
      const Eigen::Quaterniond &q_imu_camera,
      const Eigen::Vector3d &t_imu_camera,
      double sqrt_information) {
    return new ceres::AutoDiffCostFunction<
        ThreeViewFunctor, 2, 7, 7, 7>(
        new ThreeViewFunctor(
            bearing_j, bearing_k, bearing_i, q_imu_camera,
            t_imu_camera, sqrt_information));
  }

  Eigen::Vector3d bearing_j_;
  Eigen::Vector3d bearing_k_;
  Eigen::Vector3d bearing_i_;
  Eigen::Quaterniond q_imu_camera_;
  Eigen::Vector3d t_imu_camera_;
  double sqrt_information_;
};

}  // namespace pose_only_visual
