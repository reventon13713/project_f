# TODO

## P1

- [x] 공개 Open Robotics Baylands outdoor world를 선정해 `vio_baylands`로 통합했다. 원본 3개 visual mesh와 자연 texture, 경량 평면 collision, PX4 v1.14.4 `iris_fpv_cam`, 카메라/IMU/range/GPS, 독립 `/vio/ground_truth/odometry`, `spherical_coordinates`를 2026-07-21 smoke로 확인했다.
- [x] Baylands에서 초기화/검증 뒤 별도 12-waypoint XYZ cross를 비행하고 raw VINS와 독립 Gazebo GT를 rosbag으로 판정한다. acceptance는 RMSE `≤0.30 m`, trajectory/GT waypoint/VIO-GT waypoint max `≤0.50 m`; 최종 자동 batch `20260721T080015KST`가 GT `2/2` 통과했다(RMSE `0.0413~0.0423 m`, trajectory max `0.1192~0.1309 m`). GPS/EV/global conversion은 변경하지 않았다.
- [x] `vio_global_position` ROS 2 노드를 구현했다. 유효 GPS/PX4 reference 구간의 current VINS epoch 474 sample로 scale·yaw·translation과 WGS84 ellipsoid origin을 고정했고 `/vio/global_position` 및 `/vio/aligned_odometry`를 발행한다. reset/output-gap/jump/new-ready는 restart 전 latch-close한다. Baylands `20260721T084626KST`에서 PX4 fused global 대비 RMSE/max `0.0387/0.0699 m`로 통과했다.
- [x] `vio_global_position`의 정렬된 local-NED VIO를 `vio_bridge`와 연결해 PX4 external vision에 전달하고, GPS 정상 중 PX4 GPS+EV position/velocity fusion을 확인했다. Baylands `20260721T093911KST/run01`에서 native EKF2 `cs_gps`, `cs_ev_pos`, `cs_ev_vel`가 동시 true였다. global-position 연속성 정량 판정은 다음 단계에서 수행한다.
- [x] GPS 정상 중 PX4 global position과 `/vio/global_position`의 연속성·오차를 rosbag/ULog로 판정했다. Baylands `20260721T202758KST/run01`은 18.56초/177쌍에서 RMSE/max `0.1072/0.1223 m`, VIO/PX4 max step `0.00470/0.00355 m`, ULog GPS+EV 동시 fusion 25 sample·aid rejection 0·global reset 0으로 통과했다.

## P2

- [x] Gazebo world plugin과 stage-6 gate로 실제 simulated GPS sensor를 `off`로 만들고 VIO 지속 비행을 확인했다. `20260721T232119KST/run01`에서 GPS age `10.0 s`, `cs_gps=false`, EV pos/vel true, dead-reckoning false, armed/Offboard 유지, 8초 drift `0.0488 m`로 통과했다. 글로벌 mission waypoint 연동은 다음 항목에서 수행한다.
- [x] 표준 MAVLink/QGC `.plan` 글로벌 mission을 실제 PX4 `AUTO.MISSION`으로 실행했다. `DO_CHANGE_SPEED=0.55 m/s`, 고정 yaw, 7개 글로벌 waypoint 중 sequence 5에서 실제 GPS sensor를 끈 뒤 5·6·7을 완료했다. 최종 evidence `20260722T005543KST_run01`은 GPS age `17.9 s`, EV age `0.10 s`, alignment/handover 유지, mission finished를 통과했다.
- [x] QGC 지도 feed인 PX4 global position의 GPS 상실 뒤 연속 이동과 rosbag/ULog/독립 GT를 판정했다. 최종 run의 post-dropout GT/PX4/VIO path는 `9.28/8.79/8.64 m`, PX4/VIO 대 GT RMSE `0.461/0.418 m`, max `0.546/0.534 m`, GT waypoint max `0.734 m`, PX4 지도 max step `0.214 m`였다. ULog은 EV pos/vel fused `177/178`, GPS aid off, mission finished, reset 1회·max step `0.214 m`로 통과했다.

## P3

- GPS 복구 후 GPS와 VIO가 큰 global-position jump 없이 재융합되는지 Gazebo mission에서 검증한다.
- 실기체 이전에는 SITL에서 확정한 world/transform/fusion/fault-injection 절차를 HIL·계류 go/no-go 절차로 전환한다.
