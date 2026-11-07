#ifndef DATA_TYPE_H
#define DATA_TYPE_H
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <map>
#include <vector>

enum class SolverState { INITIAL, NON_LINEAR };
enum class MarginalizationType { MARGIN_OLD, MARGIN_SECOND_NEW };

using Timestamp = double;
using FeatureFrame =
    std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>;
struct TimestampedFeatureFrame {
  std::uint64_t sequence = 0;
  std::uint64_t raw_image_sequence = 0;
  Timestamp timestamp = 0.0;
  FeatureFrame features;
};
using TimestampedVector3d = std::pair<Timestamp, Eigen::Vector3d>;

#endif  //
