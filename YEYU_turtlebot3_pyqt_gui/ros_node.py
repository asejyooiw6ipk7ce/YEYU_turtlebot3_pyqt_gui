#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math  
import yaml
from rclpy.node import Node  
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# 메시지 및 액션 임포트
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, BatteryState
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from action_msgs.msg import GoalStatus  # 목표(goal)가 성공/실패 했는지 확인하기 위해 추가

from qt_signals import RosSignals

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class TurtleBot3RosNode(Node):
    def __init__(self):
        super().__init__('turtlebot3_ros_node')
        
        # 외부에서 시그널을 주입받거나 참조할 수 있도록 설정
        self.signals = RosSignals()
        
        # 데이터를 저장할 변수 초기화
        self.waypoints = {}
        self.trajectories = {}
        self.last_scan_min = 0.0
        
        # QoS 설정 (센서 및 오도메트리용 Best Effort)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)
        self.battery_sub = self.create_subscription(BatteryState, '/battery_status', self.battery_callback, 10)
        
        # Nav2 액션 클라이언트
        self.navigate_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoint_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
        self.signals.log_triggered.emit('ROS 2 노드가 성공적으로 시작되었습니다.')

    def load_yaml(self, file_path):
        """YAML 파일에서 경유점과 경로 데이터를 읽어옵니다."""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                
            waypoint_list = data.get('waypoints', [])
            trajectory_list = data.get('trajectories', [])
            
            self.waypoints = {}
            for wp in waypoint_list:
                name = wp['name']
                self.waypoints[name] = wp  # 전체 딕셔너리 보관 (pose, angle 정보 포함)

            self.trajectories = {}
            for traj in trajectory_list:
                name = traj['name']
                self.trajectories[name] = traj.get('waypoints', [])

            self.signals.yaml_loaded.emit(list(self.waypoints.keys()), list(self.trajectories.keys()))
            self.signals.log_triggered.emit(f'YAML 로드 완료: 경유점 {len(self.waypoints)}개, 경로 {len(self.trajectories)}개')
        except Exception as e:
            self.signals.log_triggered.emit(f'YAML 로드 실패: {e}')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.signals.odom_received.emit(x, y, yaw)

    def scan_callback(self, msg):
        # 무한대(inf)나 0 값 제외 처리 후 최소 거리 계산
        values = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if values:
            self.last_scan_min = min(values)  # 주석 해제하여 실시간 계산 반영
            self.signals.scan_received.emit(self.last_scan_min)

    def battery_callback(self, msg):
        # 배터리 정보를 아직 알 수 없을 때는 percentage 값이 NaN(숫자 아님)으로 옵니다.
        # 이 경우는 그냥 무시합니다. (전에는 이걸 그대로 화면에 표시해서 이상한 값이 떴습니다)
        if math.isnan(msg.percentage):
            return

        # 전압과 백분율(0.0 ~ 1.0 값을 퍼센트로 변환) 전달
        percent = msg.percentage * 100.0 if msg.percentage <= 1.0 else msg.percentage
        self.signals.battery_received.emit(percent, msg.voltage)

    def publish_cmd(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)

    def go_to_waypoint(self, wp_name):
        """단일 경유점으로 이동 (waypoint_combo + waypoint_go_PB 에서 호출)"""
        if wp_name not in self.waypoints:
            self.signals.log_triggered.emit(f'존재하지 않는 경유점입니다: {wp_name}')
            return

        # 예전에는 여기서 wait_for_server(timeout_sec=1.0)를 사용했는데,
        # 이 함수는 "최대 1초간 기다리는" 함수라서 그동안 화면(GUI) 전체가 멈췄습니다.
        # server_is_ready()는 기다리지 않고 바로 "준비됐는지 아닌지"만 확인하므로 화면이 안 멈춥니다.
        if not self.navigate_client.server_is_ready():
            self.signals.log_triggered.emit('NavigateToPose 액션 서버가 아직 준비되지 않았습니다. 잠시 후 다시 시도해주세요.')
            return

        try:
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.make_pose(wp_name)
        except (KeyError, TypeError, ValueError) as e:
            # YAML 파일에 필요한 값(pose, angle 등)이 빠져있을 때 여기로 옵니다.
            self.signals.log_triggered.emit(f'{wp_name}의 좌표 정보를 읽을 수 없습니다: {e}')
            return

        self.signals.log_triggered.emit(f'{wp_name}(으)로 이동 명령 송신 중...')
        send_goal_future = self.navigate_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.waypoint_goal_response)

    def waypoint_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.signals.log_triggered.emit('경유점 이동 요청이 거부되었습니다.')
            return
        self.signals.log_triggered.emit('경유점 이동 요청이 서버에서 수락되었습니다.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.waypoint_result)

    def waypoint_result(self, future):
        # 전에는 결과가 성공이든 실패든 무조건 "완료!"라고만 표시했습니다.
        # 실제로 성공했는지(STATUS_SUCCEEDED) 확인해서 다르게 표시하도록 고쳤습니다.
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.signals.log_triggered.emit('경유점 이동 완료!')
        else:
            self.signals.log_triggered.emit(f'경유점 이동 실패 (상태 코드: {status})')

    def make_pose(self, waypoint_name):
        wp = self.waypoints[waypoint_name]
        frame_id = wp.get('frame_id', 'map')
        position = wp['pose']['position']
        angle = wp['pose']['angle']

        x = float(position['x'])
        y = float(position['y'])
        z = 0.0

        yaw_deg = float(angle['yaw'])
        yaw_rad = math.radians(yaw_deg)

        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def go_to_trajectory(self, traj_name):
        """선택된 궤적(경로 목록)을 순차 주행하도록 액션 서버에 요청합니다."""
        if traj_name not in self.trajectories:
            self.signals.log_triggered.emit(f'존재하지 않는 경로명입니다: {traj_name}')
            return
            
        # go_to_waypoint와 마찬가지로, 화면이 멈추지 않도록 server_is_ready()로 바꿨습니다.
        if not self.waypoint_client.server_is_ready():
            self.signals.log_triggered.emit('/follow_waypoints 액션 서버가 아직 준비되지 않았습니다. 잠시 후 다시 시도해주세요.')
            return

        waypoint_names = self.trajectories[traj_name]
        poses = []

        try:
            for name in waypoint_names:
                if name not in self.waypoints:
                    self.signals.log_triggered.emit(f'YAML에 없는 경유점이 경로에 포함되어 있습니다: {name}')
                    return
                poses.append(self.make_pose(name))
        except (KeyError, TypeError, ValueError) as e:
            # YAML 파일에 필요한 값(pose, angle 등)이 빠져있을 때 여기로 옵니다.
            self.signals.log_triggered.emit(f'경로의 좌표 정보를 읽을 수 없습니다: {e}')
            return

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.signals.log_triggered.emit(f'Trajectory 순차 주행 요청: {traj_name} (경유점 {len(poses)}개)')
        send_goal_future = self.waypoint_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.trajectory_goal_response)

    def trajectory_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.signals.log_triggered.emit('Trajectory 주행 요청이 거부되었습니다.')
            return
        self.signals.log_triggered.emit('Trajectory 주행 요청이 서버에서 수락되었습니다.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.trajectory_result)

    def trajectory_result(self, future):
        # waypoint_result와 마찬가지로, 실제 성공 여부를 확인해서 표시합니다.
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.signals.log_triggered.emit('Trajectory 순차 주행 완료!')
        else:
            self.signals.log_triggered.emit(f'Trajectory 순차 주행 실패 (상태 코드: {status})')