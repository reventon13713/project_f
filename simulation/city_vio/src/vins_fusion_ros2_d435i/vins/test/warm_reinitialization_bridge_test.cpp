#include <vins/estimator/warm_reinitialization_bridge.h>

#include <cassert>
#include <cmath>
#include <iostream>

int main() {
  vins::WarmReinitializationBridge bridge;
  OdomData anchor;
  anchor.timestamp = 10.0;
  anchor.position = Eigen::Vector3d(4.0, -2.0, 1.0);
  anchor.velocity = Eigen::Vector3d(1.0, 0.0, 0.0);

  IMUData imu;
  imu.timestamp = 10.0;
  imu.linear_acceleration = Eigen::Vector3d(0.0, 0.0, 9.81);
  assert(bridge.begin(anchor, imu, Eigen::Vector3d::Zero(),
                      Eigen::Vector3d::Zero(),
                      Eigen::Vector3d(0.0, 0.0, 9.81), 1.0));

  for (int i = 1; i <= 10; ++i) {
    imu.timestamp = 10.0 + 0.01 * i;
    assert(bridge.propagate(imu));
  }
  OdomData propagated;
  assert(bridge.latest(&propagated));
  assert(std::abs(propagated.position.x() - 4.1) < 1e-9);
  assert(std::abs(propagated.position.y() + 2.0) < 1e-9);
  assert(std::abs(propagated.position.z() - 1.0) < 1e-9);
  assert((propagated.velocity - anchor.velocity).norm() < 1e-9);

  OdomData sample;
  assert(bridge.sample(10.05, &sample));
  assert(std::abs(sample.position.x() - 4.05) < 1e-9);
  assert(!bridge.sample(9.0, &sample));

  IMUData invalid = imu;
  invalid.timestamp = imu.timestamp;
  assert(!bridge.propagate(invalid));
  assert(!bridge.active());

  std::cout << "warm_reinitialization_bridge_test: PASS\n";
  return 0;
}
