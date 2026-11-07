/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <vins/initial/initial_alignment.h>
#include <vins/logger/logger.h>

#include <Eigen/Eigenvalues>

#include <limits>

void solveGyroscopeBias(map<double, ImageFrame> &all_image_frame,
                        StateData *states) {
  Matrix3d A;
  Vector3d b;
  Vector3d delta_bg;
  double residual_sum_deg = 0.0;
  double residual_max_deg = 0.0;
  int residual_count = 0;
  A.setZero();
  b.setZero();
  map<double, ImageFrame>::iterator frame_i;
  map<double, ImageFrame>::iterator frame_j;
  for (frame_i = all_image_frame.begin();
       next(frame_i) != all_image_frame.end(); frame_i++) {
    frame_j = next(frame_i);
    MatrixXd tmp_A(3, 3);
    tmp_A.setZero();
    VectorXd tmp_b(3);
    tmp_b.setZero();
    Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);
    const Eigen::Quaterniond residual_q =
        frame_j->second.pre_integration->delta_q.inverse() * q_ij;
    const double residual_deg =
        Eigen::AngleAxisd(residual_q.normalized()).angle() * 180.0 / M_PI;
    residual_sum_deg += std::abs(residual_deg);
    residual_max_deg = std::max(residual_max_deg, std::abs(residual_deg));
    ++residual_count;
    tmp_A = frame_j->second.pre_integration->jacobian.template block<3, 3>(
        O_R, O_BG);
    tmp_b =
        2 * (frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec();
    A += tmp_A.transpose() * tmp_A;
    b += tmp_A.transpose() * tmp_b;
  }
  delta_bg = A.ldlt().solve(b);
  VINS_INFO << "Visual/IMU rotation diagnostics: pairs=" << residual_count
            << ", residual_mean_deg="
            << (residual_count > 0 ? residual_sum_deg / residual_count : 0.0)
            << ", residual_max_deg=" << residual_max_deg
            << ", solved_gyro_bias_delta=" << delta_bg.transpose();

  for (int i = 0; i <= WINDOW_SIZE; i++) states[i].gyro_bias += delta_bg;

  for (frame_i = all_image_frame.begin();
       next(frame_i) != all_image_frame.end(); frame_i++) {
    frame_j = next(frame_i);
    frame_j->second.pre_integration->repropagate(Vector3d::Zero(),
                                                 states[0].gyro_bias);
  }
}

MatrixXd TangentBasis(Vector3d &g0) {
  Vector3d b, c;
  Vector3d a = g0.normalized();
  Vector3d tmp(0, 0, 1);
  if (a == tmp) tmp << 1, 0, 0;
  b = (tmp - a * (a.transpose() * tmp)).normalized();
  c = a.cross(b);
  MatrixXd bc(3, 2);
  bc.block<3, 1>(0, 0) = b;
  bc.block<3, 1>(0, 1) = c;
  return bc;
}

void RefineGravity(map<double, ImageFrame> &all_image_frame, Vector3d &g,
                   VectorXd &x, const VINSOptions &options) {
  Vector3d g0 = g.normalized() * options.imu.gravity().norm();
  Vector3d lx, ly;
  // VectorXd x;
  int all_frame_count = all_image_frame.size();
  int n_state = all_frame_count * 3 + 2 + 1;

  MatrixXd A{n_state, n_state};
  A.setZero();
  VectorXd b{n_state};
  b.setZero();

  map<double, ImageFrame>::iterator frame_i;
  map<double, ImageFrame>::iterator frame_j;
  for (int k = 0; k < 4; k++) {
    MatrixXd lxly(3, 2);
    lxly = TangentBasis(g0);
    int i = 0;
    for (frame_i = all_image_frame.begin();
         next(frame_i) != all_image_frame.end(); frame_i++, i++) {
      frame_j = next(frame_i);

      MatrixXd tmp_A(6, 9);
      tmp_A.setZero();
      VectorXd tmp_b(6);
      tmp_b.setZero();

      double dt = frame_j->second.pre_integration->sum_dt;

      tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
      tmp_A.block<3, 2>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 *
                                Matrix3d::Identity() * lxly;
      tmp_A.block<3, 1>(0, 8) = frame_i->second.R.transpose() *
                                (frame_j->second.T - frame_i->second.T) / 100.0;
      tmp_b.block<3, 1>(0, 0) =
          frame_j->second.pre_integration->delta_p +
          frame_i->second.R.transpose() * frame_j->second.R * options.TIC[0] -
          options.TIC[0] - frame_i->second.R.transpose() * dt * dt / 2 * g0;

      tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
      tmp_A.block<3, 3>(3, 3) =
          frame_i->second.R.transpose() * frame_j->second.R;
      tmp_A.block<3, 2>(3, 6) =
          frame_i->second.R.transpose() * dt * Matrix3d::Identity() * lxly;
      tmp_b.block<3, 1>(3, 0) =
          frame_j->second.pre_integration->delta_v -
          frame_i->second.R.transpose() * dt * Matrix3d::Identity() * g0;

      Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
      // cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
      // MatrixXd cov_inv = cov.inverse();
      cov_inv.setIdentity();

      MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
      VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

      A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
      b.segment<6>(i * 3) += r_b.head<6>();

      A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
      b.tail<3>() += r_b.tail<3>();

      A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
      A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
    }
    A = A * 1000.0;
    b = b * 1000.0;
    x = A.ldlt().solve(b);
    VectorXd dg = x.segment<2>(n_state - 3);
    const Vector3d gravity_before_update = g0;
    g0 = (g0 + lxly * dg).normalized() * options.imu.gravity().norm();
    VINS_INFO << "Gravity refinement iteration " << k
              << ": delta_tangent=" << dg.transpose()
              << ", gravity_before=" << gravity_before_update.transpose()
              << ", gravity_after=" << g0.transpose()
              << ", scale=" << x(n_state - 1) / 100.0;
  }
  g = g0;
}

bool LinearAlignment(map<double, ImageFrame> &all_image_frame, Vector3d &g,
                     VectorXd &x, const VINSOptions &options) {
  int all_frame_count = all_image_frame.size();
  int n_state = all_frame_count * 3 + 3 + 1;

  MatrixXd A{n_state, n_state};
  A.setZero();
  VectorXd b{n_state};
  b.setZero();

  map<double, ImageFrame>::iterator frame_i;
  map<double, ImageFrame>::iterator frame_j;
  vector<MatrixXd> local_system_matrices;
  vector<VectorXd> local_system_rhs;
  int i = 0;
  for (frame_i = all_image_frame.begin();
       next(frame_i) != all_image_frame.end(); frame_i++, i++) {
    frame_j = next(frame_i);

    MatrixXd tmp_A(6, 10);
    tmp_A.setZero();
    VectorXd tmp_b(6);
    tmp_b.setZero();

    double dt = frame_j->second.pre_integration->sum_dt;

    tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
    tmp_A.block<3, 3>(0, 6) =
        frame_i->second.R.transpose() * dt * dt / 2 * Matrix3d::Identity();
    tmp_A.block<3, 1>(0, 9) = frame_i->second.R.transpose() *
                              (frame_j->second.T - frame_i->second.T) / 100.0;
    tmp_b.block<3, 1>(0, 0) =
        frame_j->second.pre_integration->delta_p +
        frame_i->second.R.transpose() * frame_j->second.R * options.TIC[0] -
        options.TIC[0];
    // cout << "delta_p   " <<
    // frame_j->second.pre_integration->delta_p.transpose() << endl;
    tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
    tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
    tmp_A.block<3, 3>(3, 6) =
        frame_i->second.R.transpose() * dt * Matrix3d::Identity();
    tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v;
    // cout << "delta_v   " <<
    // frame_j->second.pre_integration->delta_v.transpose() << endl;

    Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
    // cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
    // MatrixXd cov_inv = cov.inverse();
    cov_inv.setIdentity();

    MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
    VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

    local_system_matrices.push_back(tmp_A);
    local_system_rhs.push_back(tmp_b);

    A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
    b.segment<6>(i * 3) += r_b.head<6>();

    A.bottomRightCorner<4, 4>() += r_A.bottomRightCorner<4, 4>();
    b.tail<4>() += r_b.tail<4>();

    A.block<6, 4>(i * 3, n_state - 4) += r_A.topRightCorner<6, 4>();
    A.block<4, 6>(n_state - 4, i * 3) += r_A.bottomLeftCorner<4, 6>();
  }
  A = A * 1000.0;
  b = b * 1000.0;
  x = A.ldlt().solve(b);
  SelfAdjointEigenSolver<MatrixXd> eigen_solver(A);
  const VectorXd eigenvalues = eigen_solver.eigenvalues();
  const double max_eigenvalue = eigenvalues.size() > 0
                                    ? eigenvalues.maxCoeff()
                                    : 0.0;
  const double observable_floor =
      std::max(1e-12, max_eigenvalue * 1e-12);
  double min_observable_eigenvalue = std::numeric_limits<double>::infinity();
  int near_zero_eigenvalues = 0;
  for (int eigen_index = 0; eigen_index < eigenvalues.size(); ++eigen_index) {
    if (eigenvalues(eigen_index) > observable_floor) {
      min_observable_eigenvalue =
          std::min(min_observable_eigenvalue, eigenvalues(eigen_index));
    } else {
      ++near_zero_eigenvalues;
    }
  }
  const double condition_number =
      std::isfinite(min_observable_eigenvalue) &&
              min_observable_eigenvalue > 0.0
          ? max_eigenvalue / min_observable_eigenvalue
          : std::numeric_limits<double>::infinity();
  double equation_residual_sq = 0.0;
  double equation_rhs_sq = 0.0;
  for (size_t pair_index = 0; pair_index < local_system_matrices.size();
       ++pair_index) {
    VectorXd local_x(10);
    local_x.segment<3>(0) = x.segment<3>(pair_index * 3);
    local_x.segment<3>(3) = x.segment<3>((pair_index + 1) * 3);
    local_x.segment<4>(6) = x.tail<4>();
    const VectorXd residual =
        local_system_matrices[pair_index] * local_x -
        local_system_rhs[pair_index];
    equation_residual_sq += residual.squaredNorm();
    equation_rhs_sq += local_system_rhs[pair_index].squaredNorm();
  }
  const int equation_count = local_system_matrices.size() * 6;
  MatrixXd physical_design = MatrixXd::Zero(equation_count, n_state);
  for (size_t pair_index = 0; pair_index < local_system_matrices.size();
       ++pair_index) {
    const int row = static_cast<int>(pair_index) * 6;
    physical_design.block(row, static_cast<int>(pair_index) * 3, 6, 6) =
        local_system_matrices[pair_index].leftCols<6>();
    physical_design.block(row, n_state - 4, 6, 3) =
        local_system_matrices[pair_index].block(0, 6, 6, 3);
    // LinearAlignment stores x_scale as 100 * physical_scale and divides the
    // design column by 100.  Undo that numerical scaling before reporting
    // conditioning or scale observability.
    physical_design.col(n_state - 1).segment(row, 6) =
        local_system_matrices[pair_index].col(9) * 100.0;
  }
  JacobiSVD<MatrixXd> physical_svd(physical_design,
                                   ComputeThinU | ComputeThinV);
  const VectorXd physical_singular_values = physical_svd.singularValues();
  const double physical_sigma_max = physical_singular_values.size() > 0
                                        ? physical_singular_values.maxCoeff()
                                        : 0.0;
  const double physical_sigma_min = physical_singular_values.size() > 0
                                        ? physical_singular_values.minCoeff()
                                        : 0.0;
  const double physical_condition =
      physical_sigma_min > 0.0
          ? physical_sigma_max / physical_sigma_min
          : std::numeric_limits<double>::infinity();
  const MatrixXd nuisance_design = physical_design.leftCols(n_state - 1);
  const VectorXd scale_column = physical_design.col(n_state - 1);
  const VectorXd projected_scale =
      nuisance_design *
      nuisance_design.completeOrthogonalDecomposition().solve(scale_column);
  const double scale_observability_ratio =
      (scale_column - projected_scale).norm() /
      std::max(1e-12, scale_column.norm());
  VINS_INFO << "Linear alignment diagnostics: frames=" << all_frame_count
            << ", equations=" << equation_count
            << ", eigen_min="
            << (eigenvalues.size() > 0 ? eigenvalues.minCoeff() : 0.0)
            << ", eigen_min_observable=" << min_observable_eigenvalue
            << ", eigen_max=" << max_eigenvalue
            << ", condition=" << condition_number
            << ", near_zero_eigenvalues=" << near_zero_eigenvalues
            << ", physical_condition=" << physical_condition
            << ", physical_sigma_min=" << physical_sigma_min
            << ", scale_observability_ratio=" << scale_observability_ratio
            << ", equation_residual_norm=" << std::sqrt(equation_residual_sq)
            << ", equation_relative_residual="
            << std::sqrt(equation_residual_sq) /
                   std::max(1e-12, std::sqrt(equation_rhs_sq))
            << ", normal_relative_residual="
            << (A * x - b).norm() / std::max(1e-12, b.norm());
  double s = x(n_state - 1) / 100.0;
  g = x.segment<3>(n_state - 4);
  VINS_INFO << "Linear alignment raw solution: scale=" << s
            << ", gravity=" << g.transpose()
            << ", gravity_norm=" << g.norm();
  VINS_DEBUG << "estimated scale: " << s;
  VINS_DEBUG << " result g     " << g.norm() << " " << g.transpose();
  if (fabs(g.norm() - options.imu.gravity().norm()) > 0.5 || s < 0) {
    VINS_WARN << "Rejected raw visual/IMU alignment solution: scale=" << s
              << ", gravity_norm=" << g.norm();
    return false;
  }

  VINS_DEBUG << "gravity:\r\n " << options.imu.gravity() << "\r\n g: " << g
             << "\r\n s: " << s;
  RefineGravity(all_image_frame, g, x, options);
  VINS_DEBUG << " refine     " << g.norm() << " " << g.transpose();
  s = (x.tail<1>())(0) / 100.0;
  (x.tail<1>())(0) = s;
  return s >= 0.0;
}

bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame,
                        StateData *states, Vector3d &g, VectorXd &x,
                        const VINSOptions &options) {
  solveGyroscopeBias(all_image_frame, states);

  return LinearAlignment(all_image_frame, g, x, options);
}
