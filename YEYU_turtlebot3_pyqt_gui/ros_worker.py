#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ros_worker.py
TurtleBot3 PyQt GUI 미니 프로젝트 - ROS2 연동 담당 모듈

- QThread 위에서 rclpy 노드를 실행하고, 토픽 구독 결과를 PyQt signal로 GUI에 전달합니다.
- Nav2 NavigateToPose 액션 클라이언트를 이용한 경유점/trajectory 이동 기능을 제공합니다.
- 배터리, cmd_vel, odom, scan 토픽을 구독합니다.
- bringup/nav2/teleop 프로세스는 GUI(main_gui.py) 쪽에서 subprocess로 직접 관리합니다.
"""

from __future__ import annotations

import math
import threading

from PyQt5.QtCore import QThread, pyqtSignal

# ROS2(rclpy)가 설치되지 않은 환경에서도 GUI 레이아웃만은 확인할 수 있도록
# ROS2 관련 임포트를 try/except로 감쌉니다. 실제 로봇/시뮬레이션 연동 시에는
# ROS2 Humble 환경에서 실행해야 정상 동작합니다.
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

    from geometry_msgs.msg import Twist, PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import BatteryState, LaserScan
    from nav2_msgs.action import NavigateToPose
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = object  # 타입 힌트/상속용 더미


def quaternion_to_yaw(q):
    """geometry_msgs/Quaternion -> yaw(rad)"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw_rad):
    """yaw(rad) -> (x, y, z, w)"""
    return 0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


class TurtleBotGuiNode(Node):
    """실제 ROS2 토픽/액션을 다루는 노드"""

    def __init__(self, signals):
        super().__init__('turtlebot3_pyqt_gui_node')
        self.signals = signals
        self._current_goal_handle = None

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---- 구독자 (요구사항 4: 주요 토픽 3개 이상 표시) ----
        self.create_subscription(BatteryState, '/battery_state', self._battery_cb, sensor_qos)
        self.create_subscription(Twist, '/cmd_vel', self._cmdvel_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, sensor_qos)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, sensor_qos)

        # ---- 발행자 ----
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---- Nav2 액션 클라이언트 (경유점 / trajectory 이동) ----
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.signals.log_signal.emit('ROS2 노드 초기화 완료')
        self.signals.connection_status.emit(True)

    # ---------------- 콜백 ----------------
    def _battery_cb(self, msg: BatteryState):
        percentage = msg.percentage if msg.percentage <= 1.0 else msg.percentage / 100.0
        self.signals.battery_updated.emit({
            'voltage': msg.voltage,
            'percentage': percentage * 100.0,
            'present': msg.present,
        })

    def _cmdvel_cb(self, msg: Twist):
        self.signals.cmdvel_updated.emit(msg.linear.x, msg.angular.z)

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.signals.odom_updated.emit(x, y, math.degrees(yaw))

    def _scan_cb(self, msg: LaserScan):
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        min_range = min(valid) if valid else float('inf')
        self.signals.scan_updated.emit(min_range)

    # ---------------- 긴급 정지 ----------------
    def publish_zero_velocity(self):
        self.cmd_vel_pub.publish(Twist())

    # ---------------- Nav2 이동 ----------------
    def send_goal(self, x, y, yaw_deg, done_callback=None):
        """단일 경유점으로 이동. done_callback(success: bool) 은 완료 시 호출."""
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.signals.log_signal.emit('오류: Nav2 액션 서버에 연결할 수 없습니다.')
            if done_callback:
                done_callback(False)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quaternion(math.radians(yaw_deg))
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.signals.goal_pose_updated.emit(x, y, yaw_deg)
        self.signals.nav_state_updated.emit('이동 중')

        send_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self._nav_feedback_cb)

        def goal_response_cb(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.signals.log_signal.emit('Nav2 목표가 거부되었습니다.')
                self.signals.nav_state_updated.emit('거부됨')
                if done_callback:
                    done_callback(False)
                return
            self._current_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()

            def result_cb(res_future):
                status = res_future.result().status
                success = (status == 4)  # GoalStatus.STATUS_SUCCEEDED
                self.signals.nav_state_updated.emit('도착 완료' if success else '이동 실패/취소')
                self.signals.log_signal.emit(
                    f'목표 지점 도착 (x={x:.2f}, y={y:.2f})' if success else '목표 이동 실패 또는 취소됨')
                if success:
                    self.signals.goal_reached.emit(x, y, yaw_deg)
                if done_callback:
                    done_callback(success)

            result_future.add_done_callback(result_cb)

        send_future.add_done_callback(goal_response_cb)

    def _nav_feedback_cb(self, feedback_msg):
        distance = feedback_msg.feedback.distance_remaining
        self.signals.nav_feedback.emit(distance)

    def cancel_goal(self):
        if self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()
            self.signals.log_signal.emit('현재 이동 목표를 취소했습니다.')
            self.signals.nav_state_updated.emit('취소됨')


class RosSignals:
    """QThread(RosWorker) 인스턴스가 소유하는 signal 모음.
    QObject을 상속하지 않고 RosWorker(QThread) 안에서 signal을 직접 정의합니다.
    (아래 RosWorker 클래스 참고)"""
    pass


class RosWorker(QThread):
    """rclpy.spin()을 별도 스레드에서 실행하며 GUI와 signal로 통신"""

    battery_updated = pyqtSignal(dict)
    cmdvel_updated = pyqtSignal(float, float)
    odom_updated = pyqtSignal(float, float, float)          # x, y, yaw(deg)
    scan_updated = pyqtSignal(float)                         # min range
    nav_state_updated = pyqtSignal(str)
    nav_feedback = pyqtSignal(float)                         # 남은 거리
    goal_pose_updated = pyqtSignal(float, float, float)
    goal_reached = pyqtSignal(float, float, float)
    connection_status = pyqtSignal(bool)
    log_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.node = None
        self._running = True
        self._lock = threading.Lock()

    def run(self):
        if not ROS2_AVAILABLE:
            self.log_signal.emit(
                '경고: rclpy(ROS2)를 찾을 수 없습니다. ROS2 Humble 환경(source install/setup.bash)에서 실행하세요.')
            self.connection_status.emit(False)
            return
        rclpy.init(args=None)
        self.node = TurtleBotGuiNode(self)
        try:
            while self._running and rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)
        finally:
            self.node.destroy_node()
            rclpy.shutdown()

    def stop(self):
        self._running = False
        self.wait(2000)

    # ---- GUI 스레드에서 호출하는 편의 메서드 (내부적으로 노드 메서드 위임) ----
    def move_to(self, x, y, yaw_deg, done_callback=None):
        if self.node is not None:
            self.node.send_goal(x, y, yaw_deg, done_callback)

    def cancel_move(self):
        if self.node is not None:
            self.node.cancel_goal()

    def emergency_stop(self):
        if self.node is not None:
            self.node.publish_zero_velocity()
