#include <vins_fusion_ros2/vins_estimator.h>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <cstdlib>
#include <string>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  const char* deterministic_env =
      std::getenv("VINS_DETERMINISTIC_REPLAY");
  const bool deterministic =
      deterministic_env && std::string(deterministic_env) == "1";
  if (deterministic) {
    cv::setNumThreads(1);
    cv::setRNGSeed(0);
    Eigen::setNbThreads(1);
  }
  auto node = std::make_shared<VinsEstimator>();
  if (deterministic) {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
  }
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
