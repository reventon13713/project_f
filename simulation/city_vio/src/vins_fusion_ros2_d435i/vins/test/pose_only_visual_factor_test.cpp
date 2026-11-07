#include <vins/factor/pose_only_visual_factor.h>

#include <ceres/ceres.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>

namespace {

void pose(double x, double y, double z, double *value) {
  value[0] = x;
  value[1] = y;
  value[2] = z;
  value[3] = 0.0;
  value[4] = 0.0;
  value[5] = 0.0;
  value[6] = 1.0;
}

Eigen::Vector3d bearing(
    const Eigen::Vector3d &point,
    const Eigen::Vector3d &camera_position) {
  const Eigen::Vector3d local = point - camera_position;
  return local / local.z();
}

}  // namespace

int main() {
  const Eigen::Vector3d point(3.0, 1.0, 5.0);
  const Eigen::Vector3d position_j(0.0, 0.0, 0.0);
  const Eigen::Vector3d position_k(1.0, 0.0, 0.0);
  const Eigen::Vector3d position_i(0.5, 0.2, 0.0);
  const Eigen::Vector3d bearing_j = bearing(point, position_j);
  const Eigen::Vector3d bearing_k = bearing(point, position_k);
  const Eigen::Vector3d bearing_i = bearing(point, position_i);
  double pose_j[7], pose_k[7], pose_i[7];
  pose(position_j.x(), position_j.y(), position_j.z(), pose_j);
  pose(position_k.x(), position_k.y(), position_k.z(), pose_k);
  pose(position_i.x(), position_i.y(), position_i.z(), pose_i);

  std::unique_ptr<ceres::CostFunction> epipolar(
      pose_only_visual::EpipolarFunctor::Create(
          bearing_j, bearing_k, Eigen::Quaterniond::Identity(),
          Eigen::Vector3d::Zero(), 100.0));
  double const *epipolar_parameters[] = {pose_j, pose_k};
  double epipolar_residual[1] = {};
  if (!epipolar->Evaluate(
          epipolar_parameters, epipolar_residual, nullptr) ||
      std::abs(epipolar_residual[0]) > 1e-10) {
    std::cerr << "FAILED: exact epipolar geometry residual\n";
    return EXIT_FAILURE;
  }

  std::unique_ptr<ceres::CostFunction> three_view(
      pose_only_visual::ThreeViewFunctor::Create(
          bearing_j, bearing_k, bearing_i,
          Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(),
          100.0));
  double const *three_view_parameters[] = {pose_j, pose_k, pose_i};
  double three_view_residual[2] = {};
  if (!three_view->Evaluate(
          three_view_parameters, three_view_residual, nullptr) ||
      std::hypot(
          three_view_residual[0], three_view_residual[1]) > 1e-8) {
    std::cerr << "FAILED: exact three-view pose-only residual: "
              << three_view_residual[0] << ", "
              << three_view_residual[1] << '\n';
    return EXIT_FAILURE;
  }

  pose_i[1] += 0.1;
  if (!three_view->Evaluate(
          three_view_parameters, three_view_residual, nullptr) ||
      std::hypot(
          three_view_residual[0], three_view_residual[1]) < 0.1) {
    std::cerr << "FAILED: factor did not reject inconsistent pose\n";
    return EXIT_FAILURE;
  }

  std::cout << "pose_only_visual_factor_test: PASS\n";
  return EXIT_SUCCESS;
}
