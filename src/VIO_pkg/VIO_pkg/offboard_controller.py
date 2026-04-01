#!/usr/bin/env python3
"""
offboard_controller.py
Phase 1: PX4 Offboard 기본 비행 제어
  상태 머신: INIT → ARMING → TAKEOFF → HOVER → LAND → DONE

실행 방법 (컨테이너 터미널 3개 필요):
  [T1] MicroXRCE-DDS Agent:  MicroXRCEAgent udp4 -p 8888
  [T2] PX4 SITL + Gazebo:   cd ~/PX4-Autopilot && make px4_sitl gz_x500
  [T3] 이 노드:              ros2 run VIO_pkg offboard_controller
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleOdometry,
)


class OffboardController(Node):
    """
    PX4 Offboard 모드 기본 비행 제어 노드.
    이·착륙 및 지정 고도 호버링을 자동 수행.
    """

    # ---------- 설정 파라미터 ----------
    TARGET_ALTITUDE_M = 5.0    # 목표 고도 [m] (양수)
    HOVER_DURATION_S  = 10.0   # 호버링 유지 시간 [s]
    REACHED_THRESHOLD = 0.3    # 목표 도달 판정 오차 [m]
    CONTROL_FREQ_HZ   = 10     # 제어 루프 주파수 [Hz]

    def __init__(self):
        super().__init__('offboard_controller')

        # PX4 ROS 2 인터페이스는 BEST_EFFORT, TRANSIENT_LOCAL QoS 사용
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Publishers ──────────────────────────────────────────────────────
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self._cb_vehicle_status, qos)
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self._cb_odometry, qos)

        # ── 상태 변수 ────────────────────────────────────────────────────────
        self.vehicle_status  = VehicleStatus()
        self.current_alt_m   = 0.0   # 현재 고도 [m] (양수: 상승)
        self.init_alt_m      = None  # 이륙 기준 고도
        self.offboard_counter = 0    # Offboard 진입 전 setpoint 사전 스트림 카운터
        self.state           = 'INIT'
        self.hover_start     = None

        # ── 타이머 ───────────────────────────────────────────────────────────
        period = 1.0 / self.CONTROL_FREQ_HZ
        self.timer = self.create_timer(period, self._control_loop)

        self.get_logger().info('OffboardController 초기화 완료')
        self.get_logger().info(f'목표 고도: {self.TARGET_ALTITUDE_M}m  |  호버링: {self.HOVER_DURATION_S}s')

    # ====================================================================
    # Subscriber 콜백
    # ====================================================================

    def _cb_vehicle_status(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def _cb_odometry(self, msg: VehicleOdometry):
        """
        VehicleOdometry.position 은 NED 좌표계 [m].
        NED에서 Z 축은 아래 방향이 양수이므로, 고도(m) = -z
        """
        z_ned = msg.position[2]
        if self.init_alt_m is None:
            self.init_alt_m = -z_ned  # 최초 수신 시 기준 고도 저장
        self.current_alt_m = -z_ned - self.init_alt_m  # 이륙 기준 상대 고도

    # ====================================================================
    # Publisher 헬퍼
    # ====================================================================

    def _timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _pub_offboard_mode(self):
        """Position 제어 모드를 지속적으로 발행. (Offboard heartbeat 역할)"""
        msg = OffboardControlMode()
        msg.position     = True
        msg.velocity     = False
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        msg.timestamp    = self._timestamp_us()
        self.offboard_mode_pub.publish(msg)

    def _pub_setpoint(self, x: float = 0.0, y: float = 0.0,
                      z_ned: float = 0.0, yaw: float = 0.0):
        """TrajectorySetpoint 발행. z_ned: NED 좌표 (음수 = 상승)."""
        msg = TrajectorySetpoint()
        msg.position  = [x, y, z_ned]
        msg.yaw       = yaw
        msg.timestamp = self._timestamp_us()
        self.trajectory_pub.publish(msg)

    def _pub_vehicle_command(self, command: int,
                              param1: float = 0.0, param2: float = 0.0):
        msg = VehicleCommand()
        msg.command          = command
        msg.param1           = param1
        msg.param2           = param2
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = self._timestamp_us()
        self.vehicle_cmd_pub.publish(msg)

    # ====================================================================
    # 명령 헬퍼
    # ====================================================================

    def _arm(self):
        self._pub_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('→ ARM 명령 전송')

    def _disarm(self):
        self._pub_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('→ DISARM 명령 전송')

    def _engage_offboard(self):
        """PX4에 Offboard 모드 전환 명령. param2=6은 Offboard sub-mode."""
        self._pub_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('→ Offboard 모드 명령 전송')

    def _land(self):
        self._pub_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('→ LAND 명령 전송')

    # ====================================================================
    # 상태 머신 (메인 제어 루프)
    # ====================================================================

    def _control_loop(self):
        target_z_ned = -self.TARGET_ALTITUDE_M  # 목표 고도를 NED Z로 변환

        # ── INIT: Offboard 진입 전 setpoint 사전 스트림 (~1초) ──────────────
        if self.state == 'INIT':
            self._pub_offboard_mode()
            self._pub_setpoint(z_ned=target_z_ned)
            self.offboard_counter += 1
            if self.offboard_counter >= self.CONTROL_FREQ_HZ:  # 1초 경과
                self._log_state('ARMING')

        # ── ARMING: Offboard 모드 전환 + Arm ──────────────────────────────
        elif self.state == 'ARMING':
            self._pub_offboard_mode()
            self._pub_setpoint(z_ned=target_z_ned)
            self._engage_offboard()
            self._arm()
            if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self._log_state('TAKEOFF')

        # ── TAKEOFF: 목표 고도까지 상승 ────────────────────────────────────
        elif self.state == 'TAKEOFF':
            self._pub_offboard_mode()
            self._pub_setpoint(z_ned=target_z_ned)
            alt = self.current_alt_m
            self.get_logger().info(f'고도: {alt:.2f}m / {self.TARGET_ALTITUDE_M}m', throttle_duration_sec=1.0)
            if alt >= self.TARGET_ALTITUDE_M - self.REACHED_THRESHOLD:
                self.hover_start = self.get_clock().now()
                self._log_state('HOVER')

        # ── HOVER: 지정 시간 유지 ───────────────────────────────────────────
        elif self.state == 'HOVER':
            self._pub_offboard_mode()
            self._pub_setpoint(z_ned=target_z_ned)
            elapsed = (self.get_clock().now() - self.hover_start).nanoseconds / 1e9
            remaining = self.HOVER_DURATION_S - elapsed
            self.get_logger().info(f'호버링 중... {remaining:.1f}s 남음', throttle_duration_sec=1.0)
            if elapsed >= self.HOVER_DURATION_S:
                self._log_state('LAND')

        # ── LAND: 착륙 명령 ─────────────────────────────────────────────────
        elif self.state == 'LAND':
            self._land()
            if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_STANDBY:
                self._log_state('DONE')

        # ── DONE ─────────────────────────────────────────────────────────────
        elif self.state == 'DONE':
            self.get_logger().info('미션 완료. 노드를 종료합니다.', once=True)
            self.timer.cancel()

    def _log_state(self, new_state: str):
        self.get_logger().info(f'[상태 전환] {self.state} → {new_state}')
        self.state = new_state


# ============================================================
# 진입점
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = OffboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자 중단 (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
