#include <vins/common/sensor_data_type.h>
#include <vins/estimator/parameters.h>
#include <vins/factor/imu_factor.h>
#include <vins/factor/pose_local_parameterization.h>
#include <vins/factor/projectionOneFrameTwoCamFactor.h>
#include <vins/factor/projectionTwoFrameOneCamFactor.h>
#include <vins/factor/projectionTwoFrameTwoCamFactor.h>

#include <ceres/ceres.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr double kFx = 382.8178926643266;
constexpr double kFy = 382.8178926643266;
constexpr double kCx = 320.5;
constexpr double kCy = 240.5;
constexpr double kPi = 3.14159265358979323846;

struct Frame {
  int index = 0;
  double timestamp = 0.0;
  std::array<double, 7> pose{};
  std::array<double, 9> speed_bias{};
  Eigen::Vector3d truth_position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond truth_orientation = Eigen::Quaterniond::Identity();
};

struct FeatureObservation {
  int frame = 0;
  int id = 0;
  int age = 0;
  double ul = 0.0;
  double vl = 0.0;
  double ur = std::numeric_limits<double>::quiet_NaN();
  double vr = std::numeric_limits<double>::quiet_NaN();
  double depth = std::numeric_limits<double>::quiet_NaN();
  bool stereo() const {
    return std::isfinite(ur) && std::isfinite(vr);
  }
};

struct FeatureTrack {
  int id = 0;
  std::vector<FeatureObservation> observations;
  double inverse_depth = 0.2;
};

struct ImuSample {
  double timestamp = 0.0;
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

struct Score {
  double position_rmse = 0.0;
  double position_max = 0.0;
  double roll_rmse = 0.0;
  double pitch_rmse = 0.0;
  double yaw_rmse = 0.0;
};

struct ObservabilityScore {
  int residual_count = 0;
  int column_count = 0;
  int rank = 0;
  double jacobian_sigma_min = 0.0;
  double jacobian_sigma_max = 0.0;
  double hessian_lambda_min = 0.0;
};

std::vector<std::string> split(const std::string &line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) fields.push_back(field);
  return fields;
}

double number(const std::string &value) {
  if (value == "nan" || value == "NaN" || value.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::stod(value);
}

std::vector<Frame> readFrames(const std::string &path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot open frames.csv");
  std::string line;
  std::getline(stream, line);
  std::vector<Frame> frames;
  while (std::getline(stream, line)) {
    const auto fields = split(line);
    if (fields.size() != 25) {
      throw std::runtime_error("invalid frames.csv row");
    }
    Frame frame;
    frame.index = std::stoi(fields[0]);
    frame.timestamp = number(fields[1]);
    for (int i = 0; i < 3; ++i) frame.pose[i] = number(fields[2 + i]);
    for (int i = 0; i < 4; ++i) frame.pose[3 + i] = number(fields[5 + i]);
    for (int i = 0; i < 3; ++i)
      frame.speed_bias[i] = number(fields[9 + i]);
    for (int i = 0; i < 6; ++i)
      frame.speed_bias[3 + i] = number(fields[12 + i]);
    frame.truth_position =
        Eigen::Vector3d(number(fields[18]), number(fields[19]),
                        number(fields[20]));
    frame.truth_orientation =
        Eigen::Quaterniond(number(fields[24]), number(fields[21]),
                           number(fields[22]), number(fields[23]));
    frame.truth_orientation.normalize();
    frames.push_back(frame);
  }
  return frames;
}

std::vector<FeatureTrack> readTracks(const std::string &path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot open features.csv");
  std::string line;
  std::getline(stream, line);
  std::map<int, std::vector<FeatureObservation>> by_id;
  while (std::getline(stream, line)) {
    const auto fields = split(line);
    if (fields.size() != 8) {
      throw std::runtime_error("invalid features.csv row");
    }
    FeatureObservation observation;
    observation.frame = std::stoi(fields[0]);
    observation.id = std::stoi(fields[1]);
    observation.age = std::stoi(fields[2]);
    observation.ul = number(fields[3]);
    observation.vl = number(fields[4]);
    observation.ur = number(fields[5]);
    observation.vr = number(fields[6]);
    observation.depth = number(fields[7]);
    by_id[observation.id].push_back(observation);
  }

  std::vector<FeatureTrack> tracks;
  for (auto &entry : by_id) {
    auto &observations = entry.second;
    std::sort(
        observations.begin(), observations.end(),
        [](const FeatureObservation &first,
           const FeatureObservation &second) {
          return first.frame < second.frame;
        });
    std::size_t begin = 0;
    while (begin < observations.size()) {
      std::size_t end = begin + 1;
      while (end < observations.size() &&
             observations[end].frame ==
                 observations[end - 1].frame + 1) {
        ++end;
      }
      std::size_t anchor = begin;
      while (anchor < end &&
             !(observations[anchor].stereo() &&
               std::isfinite(observations[anchor].depth) &&
               observations[anchor].depth > 0.05)) {
        ++anchor;
      }
      if (anchor < end && end - anchor >= 3) {
        FeatureTrack track;
        track.id = entry.first;
        track.observations.assign(
            observations.begin() + anchor, observations.begin() + end);
        track.inverse_depth = 1.0 / observations[anchor].depth;
        tracks.push_back(std::move(track));
      }
      begin = end;
    }
  }
  return tracks;
}

std::vector<ImuSample> readImu(const std::string &path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot open imu.csv");
  std::string line;
  std::getline(stream, line);
  std::vector<ImuSample> samples;
  while (std::getline(stream, line)) {
    const auto fields = split(line);
    if (fields.size() != 7) {
      throw std::runtime_error("invalid imu.csv row");
    }
    ImuSample sample;
    sample.timestamp = number(fields[0]);
    sample.acceleration =
        Eigen::Vector3d(number(fields[1]), number(fields[2]),
                        number(fields[3]));
    sample.angular_velocity =
        Eigen::Vector3d(number(fields[4]), number(fields[5]),
                        number(fields[6]));
    samples.push_back(sample);
  }
  return samples;
}

ImuSample interpolateImu(
    const std::vector<ImuSample> &samples, double timestamp) {
  const auto upper = std::lower_bound(
      samples.begin(), samples.end(), timestamp,
      [](const ImuSample &sample, double value) {
        return sample.timestamp < value;
      });
  if (upper == samples.begin()) return *upper;
  if (upper == samples.end()) return samples.back();
  const auto lower = upper - 1;
  const double ratio =
      (timestamp - lower->timestamp) /
      (upper->timestamp - lower->timestamp);
  ImuSample result;
  result.timestamp = timestamp;
  result.acceleration =
      (1.0 - ratio) * lower->acceleration + ratio * upper->acceleration;
  result.angular_velocity =
      (1.0 - ratio) * lower->angular_velocity +
      ratio * upper->angular_velocity;
  return result;
}

IntegrationBase::Ptr preintegrate(
    const std::vector<ImuSample> &samples, double start, double end,
    const Eigen::Vector3d &accel_bias,
    const Eigen::Vector3d &gyro_bias, const ImuOptions &options) {
  const ImuSample first = interpolateImu(samples, start);
  IMUData initial;
  initial.linear_acceleration = first.acceleration;
  initial.angular_velocity = first.angular_velocity;
  auto integration = std::make_shared<IntegrationBase>(
      initial, accel_bias, gyro_bias, options);
  double previous = start;
  for (const auto &sample : samples) {
    if (sample.timestamp <= start || sample.timestamp >= end) continue;
    IMUData value;
    value.timestamp = sample.timestamp - previous;
    value.linear_acceleration = sample.acceleration;
    value.angular_velocity = sample.angular_velocity;
    integration->push_back(value);
    previous = sample.timestamp;
  }
  const ImuSample last = interpolateImu(samples, end);
  IMUData value;
  value.timestamp = end - previous;
  value.linear_acceleration = last.acceleration;
  value.angular_velocity = last.angular_velocity;
  integration->push_back(value);
  return integration;
}

Eigen::Vector3d bearing(double u, double v) {
  return Eigen::Vector3d((u - kCx) / kFx, (v - kCy) / kFy, 1.0);
}

Eigen::Quaterniond poseQuaternion(const std::array<double, 7> &pose) {
  return Eigen::Quaterniond(pose[6], pose[3], pose[4], pose[5]).normalized();
}

Score score(const std::vector<Frame> &frames) {
  Eigen::Vector2d estimate_mean = Eigen::Vector2d::Zero();
  Eigen::Vector2d truth_mean = Eigen::Vector2d::Zero();
  for (const auto &frame : frames) {
    estimate_mean += Eigen::Vector2d(frame.pose[0], frame.pose[1]);
    truth_mean += frame.truth_position.head<2>();
  }
  estimate_mean /= frames.size();
  truth_mean /= frames.size();
  double cross = 0.0;
  double dot = 0.0;
  for (const auto &frame : frames) {
    const Eigen::Vector2d estimate =
        Eigen::Vector2d(frame.pose[0], frame.pose[1]) - estimate_mean;
    const Eigen::Vector2d truth =
        frame.truth_position.head<2>() - truth_mean;
    cross += estimate.x() * truth.y() - estimate.y() * truth.x();
    dot += estimate.dot(truth);
  }
  const double yaw = std::atan2(cross, dot);
  const Eigen::AngleAxisd yaw_rotation(yaw, Eigen::Vector3d::UnitZ());
  const Eigen::Vector3d estimate_mean_3d(
      estimate_mean.x(), estimate_mean.y(), 0.0);
  Eigen::Vector3d truth_mean_3d = Eigen::Vector3d::Zero();
  Eigen::Vector3d full_estimate_mean = Eigen::Vector3d::Zero();
  for (const auto &frame : frames) {
    truth_mean_3d += frame.truth_position;
    full_estimate_mean +=
        Eigen::Vector3d(frame.pose[0], frame.pose[1], frame.pose[2]);
  }
  truth_mean_3d /= frames.size();
  full_estimate_mean /= frames.size();
  const Eigen::Vector3d translation =
      truth_mean_3d - yaw_rotation * full_estimate_mean;

  Score result;
  double position_squared = 0.0;
  double rpy_squared[3] = {0.0, 0.0, 0.0};
  for (const auto &frame : frames) {
    const Eigen::Vector3d position =
        yaw_rotation *
            Eigen::Vector3d(frame.pose[0], frame.pose[1], frame.pose[2]) +
        translation;
    const double error =
        (position.head<2>() - frame.truth_position.head<2>()).norm();
    position_squared += error * error;
    result.position_max = std::max(result.position_max, error);
    const Eigen::Quaterniond aligned =
        Eigen::Quaterniond(yaw_rotation) * poseQuaternion(frame.pose);
    const Eigen::Matrix3d delta =
        (frame.truth_orientation.inverse() * aligned).toRotationMatrix();
    const double roll = std::atan2(delta(2, 1), delta(2, 2));
    const double pitch =
        std::asin(std::max(-1.0, std::min(1.0, -delta(2, 0))));
    const double heading = std::atan2(delta(1, 0), delta(0, 0));
    rpy_squared[0] += roll * roll;
    rpy_squared[1] += pitch * pitch;
    rpy_squared[2] += heading * heading;
  }
  result.position_rmse =
      std::sqrt(position_squared / frames.size());
  result.roll_rmse =
      std::sqrt(rpy_squared[0] / frames.size()) * 180.0 / kPi;
  result.pitch_rmse =
      std::sqrt(rpy_squared[1] / frames.size()) * 180.0 / kPi;
  result.yaw_rmse =
      std::sqrt(rpy_squared[2] / frames.size()) * 180.0 / kPi;
  return result;
}

void writeTrajectory(
    const std::string &path, const std::vector<Frame> &frames) {
  std::ofstream stream(path);
  stream << "timestamp,px,py,pz,qx,qy,qz,qw,"
            "vx,vy,vz,bax,bay,baz,bgx,bgy,bgz,"
            "gt_px,gt_py,gt_pz,gt_qx,gt_qy,gt_qz,gt_qw\n";
  stream << std::setprecision(15);
  for (const auto &frame : frames) {
    stream << frame.timestamp;
    for (double value : frame.pose) stream << "," << value;
    for (double value : frame.speed_bias) stream << "," << value;
    stream << "," << frame.truth_position.x()
           << "," << frame.truth_position.y()
           << "," << frame.truth_position.z()
           << "," << frame.truth_orientation.x()
           << "," << frame.truth_orientation.y()
           << "," << frame.truth_orientation.z()
           << "," << frame.truth_orientation.w() << "\n";
  }
}

}  // namespace

int main(int argc, char **argv) {
  if (argc != 3 && argc != 4) {
    std::cerr << "usage: production_vins_batch DATA_DIRECTORY OUTPUT_PREFIX "
                 "[--include-long-imu]\n";
    return 2;
  }
  const std::string directory = argv[1];
  const std::string output_prefix = argv[2];
  const bool include_long_imu =
      argc == 4 && std::string(argv[3]) == "--include-long-imu";
  if (argc == 4 && !include_long_imu) {
    throw std::runtime_error("unknown production_vins_batch option");
  }
  auto frames = readFrames(directory + "/frames.csv");
  auto tracks = readTracks(directory + "/features.csv");
  const auto imu = readImu(directory + "/imu.csv");
  if (frames.size() < 11 || tracks.empty()) {
    throw std::runtime_error("insufficient batch data");
  }
  const Score initial_score = score(frames);

  ImuOptions imu_options;
  imu_options.accelNoiseDensity = 0.005;
  imu_options.gyroNoiseDensity = 0.0005;
  imu_options.accelRandomWalk = 0.001;
  imu_options.gyroRandomWalk = 0.0001;
  imu_options.gravityVector = Eigen::Vector3d(0.0, 0.0, 9.81007);
  imu_options.useImu = 1;

  ProjectionTwoFrameOneCamFactor::sqrt_info =
      FOCAL_LENGTH / 1.5 * Eigen::Matrix2d::Identity();
  ProjectionTwoFrameTwoCamFactor::sqrt_info =
      FOCAL_LENGTH / 1.5 * Eigen::Matrix2d::Identity();
  ProjectionOneFrameTwoCamFactor::sqrt_info =
      FOCAL_LENGTH / 1.5 * Eigen::Matrix2d::Identity();

  std::array<double, 7> extrinsic_left{
      0.0, 0.025, 0.0, -0.5, 0.5, -0.5, 0.5};
  std::array<double, 7> extrinsic_right{
      0.0, -0.025, 0.0, -0.5, 0.5, -0.5, 0.5};
  double time_delay = 0.0;

  ceres::Problem problem;
  for (auto &frame : frames) {
    problem.AddParameterBlock(
        frame.pose.data(), 7, new PoseLocalParameterization());
    problem.AddParameterBlock(frame.speed_bias.data(), 9);
  }
  problem.SetParameterBlockConstant(frames.front().pose.data());
  problem.SetParameterBlockConstant(frames.front().speed_bias.data());
  problem.AddParameterBlock(
      extrinsic_left.data(), 7, new PoseLocalParameterization());
  problem.AddParameterBlock(
      extrinsic_right.data(), 7, new PoseLocalParameterization());
  problem.SetParameterBlockConstant(extrinsic_left.data());
  problem.SetParameterBlockConstant(extrinsic_right.data());
  problem.AddParameterBlock(&time_delay, 1);
  problem.SetParameterBlockConstant(&time_delay);

  int imu_factor_count = 0;
  int skipped_long_imu_interval_count = 0;
  for (std::size_t index = 0; index + 1 < frames.size(); ++index) {
    const Eigen::Vector3d accel_bias(
        frames[index].speed_bias[3], frames[index].speed_bias[4],
        frames[index].speed_bias[5]);
    const Eigen::Vector3d gyro_bias(
        frames[index].speed_bias[6], frames[index].speed_bias[7],
        frames[index].speed_bias[8]);
    auto integration = preintegrate(
        imu, frames[index].timestamp, frames[index + 1].timestamp,
        accel_bias, gyro_bias, imu_options);
    // Match Estimator::AddIMUFactors exactly: accumulated integrations longer
    // than 10 s are retained in the window but omitted from the Ceres graph.
    if (integration->sum_dt > 10.0 && !include_long_imu) {
      ++skipped_long_imu_interval_count;
      continue;
    }
    problem.AddResidualBlock(
        new IMUFactor(integration), nullptr,
        frames[index].pose.data(), frames[index].speed_bias.data(),
        frames[index + 1].pose.data(),
        frames[index + 1].speed_bias.data());
    ++imu_factor_count;
  }

  int mono_factor_count = 0;
  int stereo_factor_count = 0;
  for (auto &track : tracks) {
    problem.AddParameterBlock(&track.inverse_depth, 1);
    problem.SetParameterLowerBound(&track.inverse_depth, 0, 1e-4);
    problem.SetParameterUpperBound(&track.inverse_depth, 0, 20.0);
    const auto &anchor = track.observations.front();
    const Eigen::Vector3d anchor_bearing =
        bearing(anchor.ul, anchor.vl);
    const Eigen::Vector2d zero_velocity = Eigen::Vector2d::Zero();
    for (const auto &observation : track.observations) {
      const Eigen::Vector3d left_bearing =
          bearing(observation.ul, observation.vl);
      if (observation.frame != anchor.frame) {
        problem.AddResidualBlock(
            new ProjectionTwoFrameOneCamFactor(
                anchor_bearing, left_bearing, zero_velocity,
                zero_velocity, 0.0, 0.0),
            new ceres::HuberLoss(1.0),
            frames[anchor.frame].pose.data(),
            frames[observation.frame].pose.data(),
            extrinsic_left.data(), &track.inverse_depth, &time_delay);
        ++mono_factor_count;
      }
      if (!observation.stereo()) continue;
      const Eigen::Vector3d right_bearing =
          bearing(observation.ur, observation.vr);
      if (observation.frame == anchor.frame) {
        problem.AddResidualBlock(
            new ProjectionOneFrameTwoCamFactor(
                anchor_bearing, right_bearing, zero_velocity,
                zero_velocity, 0.0, 0.0),
            new ceres::HuberLoss(1.0),
            extrinsic_left.data(), extrinsic_right.data(),
            &track.inverse_depth, &time_delay);
      } else {
        problem.AddResidualBlock(
            new ProjectionTwoFrameTwoCamFactor(
                anchor_bearing, right_bearing, zero_velocity,
                zero_velocity, 0.0, 0.0),
            new ceres::HuberLoss(1.0),
            frames[anchor.frame].pose.data(),
            frames[observation.frame].pose.data(),
            extrinsic_left.data(), extrinsic_right.data(),
            &track.inverse_depth, &time_delay);
      }
      ++stereo_factor_count;
    }
  }

  // Measure the local information actually present in the requested
  // [roll, pitch, velocity, accel-bias] state block.  Landmark depths and
  // yaw/translation/gyro-bias are deliberately excluded from the columns;
  // residual rows still include every production visual and IMU factor.
  ObservabilityScore observability;
  {
    ceres::Problem::EvaluateOptions evaluate_options;
    for (std::size_t i = 1; i < frames.size(); ++i) {
      evaluate_options.parameter_blocks.push_back(frames[i].pose.data());
      evaluate_options.parameter_blocks.push_back(
          frames[i].speed_bias.data());
    }
    evaluate_options.apply_loss_function = true;
    double evaluated_cost = 0.0;
    std::vector<double> residuals;
    ceres::CRSMatrix crs;
    if (!problem.Evaluate(evaluate_options, &evaluated_cost, &residuals,
                          nullptr, &crs)) {
      throw std::runtime_error("failed to evaluate observability Jacobian");
    }
    Eigen::MatrixXd full = Eigen::MatrixXd::Zero(crs.num_rows, crs.num_cols);
    for (int row = 0; row < crs.num_rows; ++row) {
      for (int entry = crs.rows[row]; entry < crs.rows[row + 1]; ++entry) {
        full(row, crs.cols[entry]) = crs.values[entry];
      }
    }
    // Each non-constant frame contributes pose local [p(3), theta(3)] and
    // speed-bias [v(3), ba(3), bg(3)] columns in the requested order.
    constexpr int kLocalColumnsPerFrame = 15;
    std::vector<int> selected;
    for (int offset = 0; offset + kLocalColumnsPerFrame <= full.cols();
         offset += kLocalColumnsPerFrame) {
      selected.push_back(offset + 3);  // local roll axis
      selected.push_back(offset + 4);  // local pitch axis
      for (int component = 0; component < 6; ++component) {
        selected.push_back(offset + 6 + component);  // velocity + accel bias
      }
    }
    Eigen::MatrixXd block(full.rows(), selected.size());
    for (std::size_t column = 0; column < selected.size(); ++column) {
      block.col(column) = full.col(selected[column]);
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(block);
    const Eigen::VectorXd singular = svd.singularValues();
    observability.residual_count = block.rows();
    observability.column_count = block.cols();
    if (singular.size() > 0) {
      observability.jacobian_sigma_max = singular.maxCoeff();
      const double threshold =
          std::max(1e-12, observability.jacobian_sigma_max * 1e-10);
      observability.jacobian_sigma_min =
          std::numeric_limits<double>::infinity();
      for (int i = 0; i < singular.size(); ++i) {
        if (singular(i) <= threshold) continue;
        ++observability.rank;
        observability.jacobian_sigma_min =
            std::min(observability.jacobian_sigma_min, singular(i));
      }
      if (observability.rank == 0) observability.jacobian_sigma_min = 0.0;
      observability.hessian_lambda_min =
          observability.jacobian_sigma_min *
          observability.jacobian_sigma_min;
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.max_num_iterations = 100;
  options.max_solver_time_in_seconds = 300.0;
  options.num_threads = 8;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  for (auto &frame : frames) {
    Eigen::Quaterniond quaternion = poseQuaternion(frame.pose);
    frame.pose[3] = quaternion.x();
    frame.pose[4] = quaternion.y();
    frame.pose[5] = quaternion.z();
    frame.pose[6] = quaternion.w();
  }
  const Score final_score = score(frames);
  writeTrajectory(output_prefix + "_trajectory.csv", frames);

  std::ofstream result(output_prefix + "_summary.json");
  result << std::setprecision(12)
         << "{\n"
         << "  \"schema_version\": 1,\n"
         << "  \"method\": \"production ProjectionTwoFrameOneCam/"
            "ProjectionOneFrameTwoCam/ProjectionTwoFrameTwoCam factors, "
            "production IMUFactor and IntegrationBase, accel/gyro bias and "
            "gravity states, no marginalization prior\",\n"
         << "  \"frame_count\": " << frames.size() << ",\n"
         << "  \"feature_track_count\": " << tracks.size() << ",\n"
         << "  \"imu_factor_count\": " << imu_factor_count << ",\n"
         << "  \"skipped_long_imu_interval_count\": "
         << skipped_long_imu_interval_count << ",\n"
         << "  \"include_long_imu\": "
         << (include_long_imu ? "true" : "false") << ",\n"
         << "  \"mono_factor_count\": " << mono_factor_count << ",\n"
         << "  \"stereo_factor_count\": " << stereo_factor_count << ",\n"
         << "  \"observability_roll_pitch_velocity_accel_bias\": {\n"
         << "    \"residual_count\": " << observability.residual_count
         << ",\n"
         << "    \"column_count\": " << observability.column_count
         << ",\n"
         << "    \"rank\": " << observability.rank << ",\n"
         << "    \"jacobian_sigma_min\": "
         << observability.jacobian_sigma_min << ",\n"
         << "    \"jacobian_sigma_max\": "
         << observability.jacobian_sigma_max << ",\n"
         << "    \"hessian_lambda_min\": "
         << observability.hessian_lambda_min << "\n"
         << "  },\n"
         << "  \"initial_cost\": " << summary.initial_cost << ",\n"
         << "  \"final_cost\": " << summary.final_cost << ",\n"
         << "  \"iterations\": " << summary.iterations.size() << ",\n"
         << "  \"termination\": \"" << summary.BriefReport() << "\",\n"
         << "  \"initial\": {\n"
         << "    \"horizontal_position_rmse_m\": "
         << initial_score.position_rmse << ",\n"
         << "    \"horizontal_position_max_m\": "
         << initial_score.position_max << ",\n"
         << "    \"roll_rmse_deg\": " << initial_score.roll_rmse << ",\n"
         << "    \"pitch_rmse_deg\": " << initial_score.pitch_rmse << ",\n"
         << "    \"yaw_rmse_deg\": " << initial_score.yaw_rmse << "\n"
         << "  },\n"
         << "  \"optimized\": {\n"
         << "    \"horizontal_position_rmse_m\": "
         << final_score.position_rmse << ",\n"
         << "    \"horizontal_position_max_m\": "
         << final_score.position_max << ",\n"
         << "    \"roll_rmse_deg\": " << final_score.roll_rmse << ",\n"
         << "    \"pitch_rmse_deg\": " << final_score.pitch_rmse << ",\n"
         << "    \"yaw_rmse_deg\": " << final_score.yaw_rmse << "\n"
         << "  },\n"
         << "  \"limitations\": \"first pose and first velocity/bias block "
            "fixed to the pre-contamination online state; tracks without a "
            "valid stereo anchor are excluded; td and extrinsics fixed as in "
            "production\"\n"
         << "}\n";
  std::cout << summary.FullReport() << "\n"
            << "frames=" << frames.size()
            << " tracks=" << tracks.size()
            << " imu=" << imu_factor_count
            << " mono=" << mono_factor_count
            << " stereo=" << stereo_factor_count << "\n";
  return summary.IsSolutionUsable() ? 0 : 1;
}
