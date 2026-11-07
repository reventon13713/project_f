#include <vins/factor/marginalization_factor.h>

#include <cassert>
#include <cmath>
#include <iostream>

int main() {
  MarginalizationInfo info;
  info.n = 4;
  info.m = 0;
  info.valid = true;
  info.linearized_jacobians.resize(4, 4);
  info.linearized_jacobians <<
      2.0, 1.0, 0.0, 0.0,
      0.0, 3.0, 1.0, 0.0,
      0.0, 0.0, 4.0, 1.0,
      1.0, 0.0, 0.0, 5.0;
  info.linearized_residuals =
      (Eigen::Vector4d() << 1.0, -2.0, 0.5, 3.0).finished();

  const Eigen::Vector4d direction =
      (Eigen::Vector4d() << 1.0, 2.0, -1.0, 0.5).finished();
  const Eigen::MatrixXd original_jacobian = info.linearized_jacobians;
  const Eigen::VectorXd original_residual = info.linearized_residuals;
  const Eigen::VectorXd affected = original_jacobian * direction;
  const Eigen::VectorXd unit = affected.normalized();
  const Eigen::Vector4d orthogonal =
      (Eigen::Vector4d() << -unit.y(), unit.x(), -unit.w(), unit.z())
          .finished();
  const double original_orthogonal_residual =
      orthogonal.dot(original_residual);
  const Eigen::RowVectorXd original_orthogonal_jacobian =
      orthogonal.transpose() * original_jacobian;

  double removed_norm = 0.0;
  double removed_residual = 0.0;
  assert(info.projectOutLocalDirection(
      direction, &removed_norm, &removed_residual));
  assert(std::abs(removed_norm - affected.norm()) < 1e-12);
  assert(std::abs(removed_residual - unit.dot(original_residual)) < 1e-12);
  assert((info.linearized_jacobians * direction).norm() < 1e-10);
  assert(std::abs(orthogonal.dot(info.linearized_residuals) -
                  original_orthogonal_residual) < 1e-10);
  assert((orthogonal.transpose() * info.linearized_jacobians -
          original_orthogonal_jacobian).norm() < 1e-10);
  assert(info.projectOutLocalDirection(direction));
  assert(!info.projectOutLocalDirection(Eigen::Vector3d::Ones()));

  std::cout << "marginalization_scale_projection_test: PASS\n";
  return 0;
}
