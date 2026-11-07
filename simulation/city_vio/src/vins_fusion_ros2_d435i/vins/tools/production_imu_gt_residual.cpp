#include <vins/common/sensor_data_type.h>
#include <vins/estimator/parameters.h>
#include <vins/factor/integration_base.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct State {
  double timestamp = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
};

struct ImuSample {
  double timestamp = 0.0;
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

std::vector<std::string> split(const std::string &line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) fields.push_back(field);
  return fields;
}

double number(const std::string &value) { return std::stod(value); }

std::vector<State> readStates(const std::string &path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot open states.csv");
  std::string line;
  std::getline(stream, line);
  std::vector<State> states;
  while (std::getline(stream, line)) {
    const auto fields = split(line);
    if (fields.size() != 18) {
      throw std::runtime_error("invalid states.csv row");
    }
    State state;
    state.timestamp = number(fields[1]);
    state.position =
        Eigen::Vector3d(number(fields[2]), number(fields[3]), number(fields[4]));
    state.orientation = Eigen::Quaterniond(
        number(fields[8]), number(fields[5]), number(fields[6]),
        number(fields[7])).normalized();
    state.velocity =
        Eigen::Vector3d(number(fields[9]), number(fields[10]), number(fields[11]));
    state.accel_bias = Eigen::Vector3d(
        number(fields[12]), number(fields[13]), number(fields[14]));
    state.gyro_bias = Eigen::Vector3d(
        number(fields[15]), number(fields[16]), number(fields[17]));
    states.push_back(state);
  }
  return states;
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
        Eigen::Vector3d(number(fields[1]), number(fields[2]), number(fields[3]));
    sample.angular_velocity =
        Eigen::Vector3d(number(fields[4]), number(fields[5]), number(fields[6]));
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
    const Eigen::Vector3d &accel_bias, const Eigen::Vector3d &gyro_bias,
    const ImuOptions &options) {
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

Eigen::Matrix<double, 15, 1> evaluate(
    const IntegrationBase::Ptr &integration, const State &first,
    const State &second, const Eigen::Vector3d &first_accel_bias,
    const Eigen::Vector3d &first_gyro_bias,
    const Eigen::Vector3d &second_accel_bias,
    const Eigen::Vector3d &second_gyro_bias) {
  return integration->evaluate(
      first.position, first.orientation, first.velocity,
      first_accel_bias, first_gyro_bias,
      second.position, second.orientation, second.velocity,
      second_accel_bias, second_gyro_bias);
}

Eigen::Matrix<double, 15, 1> whiten(
    const IntegrationBase::Ptr &integration,
    const Eigen::Matrix<double, 15, 1> &residual) {
  const Eigen::Matrix<double, 15, 15> information =
      integration->covariance.inverse();
  const Eigen::Matrix<double, 15, 15> sqrt_info =
      Eigen::LLT<Eigen::Matrix<double, 15, 15>>(information)
          .matrixL()
          .transpose();
  return sqrt_info * residual;
}

Eigen::Vector3d smallRotation(
    const Eigen::Quaterniond &reference,
    const Eigen::Quaterniond &value) {
  Eigen::Quaterniond delta = reference.inverse() * value;
  if (delta.w() < 0.0) delta.coeffs() *= -1.0;
  return 2.0 * delta.vec();
}

struct JacobianErrors {
  double dp_dba = 0.0;
  double dp_dbg = 0.0;
  double dq_dbg = 0.0;
  double dv_dba = 0.0;
  double dv_dbg = 0.0;
};

JacobianErrors finiteDifferenceErrors(
    const std::vector<ImuSample> &samples, const State &first,
    const State &second, const ImuOptions &options,
    const IntegrationBase::Ptr &base) {
  constexpr double kAccelStep = 1e-4;
  constexpr double kGyroStep = 1e-5;
  Eigen::Matrix3d fd_dp_dba;
  Eigen::Matrix3d fd_dp_dbg;
  Eigen::Matrix3d fd_dq_dbg;
  Eigen::Matrix3d fd_dv_dba;
  Eigen::Matrix3d fd_dv_dbg;
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();
    accel(axis) = kAccelStep;
    const auto accel_plus = preintegrate(
        samples, first.timestamp, second.timestamp, accel,
        Eigen::Vector3d::Zero(), options);
    const auto accel_minus = preintegrate(
        samples, first.timestamp, second.timestamp, -accel,
        Eigen::Vector3d::Zero(), options);
    fd_dp_dba.col(axis) =
        (accel_plus->delta_p - accel_minus->delta_p) /
        (2.0 * kAccelStep);
    fd_dv_dba.col(axis) =
        (accel_plus->delta_v - accel_minus->delta_v) /
        (2.0 * kAccelStep);

    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
    gyro(axis) = kGyroStep;
    const auto gyro_plus = preintegrate(
        samples, first.timestamp, second.timestamp,
        Eigen::Vector3d::Zero(), gyro, options);
    const auto gyro_minus = preintegrate(
        samples, first.timestamp, second.timestamp,
        Eigen::Vector3d::Zero(), -gyro, options);
    fd_dp_dbg.col(axis) =
        (gyro_plus->delta_p - gyro_minus->delta_p) /
        (2.0 * kGyroStep);
    fd_dv_dbg.col(axis) =
        (gyro_plus->delta_v - gyro_minus->delta_v) /
        (2.0 * kGyroStep);
    fd_dq_dbg.col(axis) =
        (smallRotation(base->delta_q, gyro_plus->delta_q) -
         smallRotation(base->delta_q, gyro_minus->delta_q)) /
        (2.0 * kGyroStep);
  }
  JacobianErrors errors;
  errors.dp_dba =
      (fd_dp_dba - base->jacobian.block<3, 3>(O_P, O_BA)).cwiseAbs().maxCoeff();
  errors.dp_dbg =
      (fd_dp_dbg - base->jacobian.block<3, 3>(O_P, O_BG)).cwiseAbs().maxCoeff();
  errors.dq_dbg =
      (fd_dq_dbg - base->jacobian.block<3, 3>(O_R, O_BG)).cwiseAbs().maxCoeff();
  errors.dv_dba =
      (fd_dv_dba - base->jacobian.block<3, 3>(O_V, O_BA)).cwiseAbs().maxCoeff();
  errors.dv_dbg =
      (fd_dv_dbg - base->jacobian.block<3, 3>(O_V, O_BG)).cwiseAbs().maxCoeff();
  return errors;
}

void writeVector(
    std::ostream &stream, const Eigen::Matrix<double, 15, 1> &value) {
  for (int index = 0; index < 15; ++index) stream << "," << value(index);
}

void writeResidualHeader(std::ostream &stream, const std::string &prefix) {
  static const std::array<const char *, 15> names{
      "p_x", "p_y", "p_z", "r_x", "r_y", "r_z", "v_x", "v_y", "v_z",
      "ba_x", "ba_y", "ba_z", "bg_x", "bg_y", "bg_z"};
  for (const auto *name : names) stream << "," << prefix << "_" << name;
}

}  // namespace

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "usage: production_imu_gt_residual DATA_DIRECTORY OUTPUT.csv\n";
    return 2;
  }
  const std::string directory = argv[1];
  const auto states = readStates(directory + "/states.csv");
  const auto imu = readImu(directory + "/imu.csv");
  if (states.size() < 30 || imu.size() < 100) {
    throw std::runtime_error("insufficient state or IMU rows");
  }

  ImuOptions options;
  options.accelNoiseDensity = 0.005;
  options.gyroNoiseDensity = 0.0005;
  options.accelRandomWalk = 0.001;
  options.gyroRandomWalk = 0.0001;
  options.gravityVector = Eigen::Vector3d(0.0, 0.0, 9.81007);
  options.useImu = 1;

  std::ofstream output(argv[2]);
  if (!output) throw std::runtime_error("cannot open output CSV");
  output << "start_timestamp,end_timestamp,dt,"
            "online_bax,online_bay,online_baz,"
            "online_bgx,online_bgy,online_bgz";
  writeResidualHeader(output, "zero");
  writeResidualHeader(output, "corrected_online");
  writeResidualHeader(output, "reintegrated_online");
  writeResidualHeader(output, "white_zero");
  writeResidualHeader(output, "white_reintegrated_online");
  output << ",correction_p_error,correction_r_error,correction_v_error,"
            "jacobian_fd_dp_dba_max,jacobian_fd_dp_dbg_max,"
            "jacobian_fd_dq_dbg_max,jacobian_fd_dv_dba_max,"
            "jacobian_fd_dv_dbg_max\n";
  output << std::setprecision(15);

  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  for (std::size_t index = 0; index + 1 < states.size(); ++index) {
    const auto &first = states[index];
    const auto &second = states[index + 1];
    const auto zero_integration = preintegrate(
        imu, first.timestamp, second.timestamp, zero, zero, options);
    const auto online_integration = preintegrate(
        imu, first.timestamp, second.timestamp,
        first.accel_bias, first.gyro_bias, options);
    const auto zero_residual =
        evaluate(zero_integration, first, second, zero, zero, zero, zero);
    const auto corrected_online_residual = evaluate(
        zero_integration, first, second,
        first.accel_bias, first.gyro_bias,
        second.accel_bias, second.gyro_bias);
    const auto reintegrated_online_residual = evaluate(
        online_integration, first, second,
        first.accel_bias, first.gyro_bias,
        second.accel_bias, second.gyro_bias);
    const auto white_zero = whiten(zero_integration, zero_residual);
    const auto white_online =
        whiten(online_integration, reintegrated_online_residual);
    const auto correction_error =
        corrected_online_residual - reintegrated_online_residual;
    const auto jacobian_errors = finiteDifferenceErrors(
        imu, first, second, options, zero_integration);

    output << first.timestamp << "," << second.timestamp << ","
           << zero_integration->sum_dt;
    for (int axis = 0; axis < 3; ++axis)
      output << "," << first.accel_bias(axis);
    for (int axis = 0; axis < 3; ++axis)
      output << "," << first.gyro_bias(axis);
    writeVector(output, zero_residual);
    writeVector(output, corrected_online_residual);
    writeVector(output, reintegrated_online_residual);
    writeVector(output, white_zero);
    writeVector(output, white_online);
    output << "," << correction_error.segment<3>(O_P).norm()
           << "," << correction_error.segment<3>(O_R).norm()
           << "," << correction_error.segment<3>(O_V).norm()
           << "," << jacobian_errors.dp_dba
           << "," << jacobian_errors.dp_dbg
           << "," << jacobian_errors.dq_dbg
           << "," << jacobian_errors.dv_dba
           << "," << jacobian_errors.dv_dbg << "\n";
  }
  std::cout << "wrote " << states.size() - 1
            << " production IntegrationBase residual intervals to "
            << argv[2] << "\n";
  return 0;
}
