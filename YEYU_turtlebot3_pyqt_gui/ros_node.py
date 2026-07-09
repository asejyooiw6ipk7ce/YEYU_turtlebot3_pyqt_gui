import math
import yaml
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# === geometry_msgs 관련 메시지들 ===
from geometry_msgs.msg import Twist                      # /cmd_vel용
from geometry_msgs.msg import PoseStamped                 # make_pose()용
from geometry_msgs.msg import PoseWithCovarianceStamped   # /initialpose용

# === nav_msgs 관련 메시지들 ===
from nav_msgs.msg import Odometry                         # /odom용

# === sensor_msgs 관련 메시지들 ===
from sensor_msgs.msg import LaserScan                     # /scan용
from sensor_msgs.msg import BatteryState                  # /battery_status용

# === nav2_msgs 관련 액션들 ===
from nav2_msgs.action import NavigateToPose               # 단일 waypoint 이동용
from nav2_msgs.action import FollowWaypoints              # trajectory(순차 waypoint) 이동용

# === qt_signals.py ===
from qt_signals import RosSignals


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class TurtleBot3RosNode(Node):
    def __init__(self):
        super().__init__('turtlebot3_ros_node')
        self.signals = RosSignals()

        # gui.py에서 yaml 경로를 채워줌
        self.yaml_file = ''

        # yaml에서 읽어올 정보
        self.waypoints = {}         # 단일 목적지 저장할 딕셔너리
        self.trajectories = {}      # 경로(목적지들의 묶음) 저장할 딕셔너리

        # 상태 캐시
        self.last_battery_p = None
        self.last_battery_v = None
        self.last_odom = None
        self.last_scan_min = None
        self.goal_handle = None

        # /battery_status 수신자
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_status',
            self.battery_callback,
            10
        )

        # /cmd_vel 발행자
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # /odom 수신자
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # /initialpose 발행자
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # 단일 waypoint 이동용 액션클라이언트
        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # trajectory(경유점 순차 주행) 액션클라이언트
        self.follow_client = ActionClient(
            self,
            FollowWaypoints,
            'follow_waypoints'
        )

        # /scan 수신 (BEST_EFFORT QoS)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )

    # YAML 파일 읽어서 waypoints/trajectories 채우는 함수
    def load_yaml(self):
        with open(self.yaml_file, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

        waypoint_list = data['waypoints']
        trajectory_list = data['trajectories']

        for wp in waypoint_list:
            name = wp['name']
            self.waypoints[name] = wp

        for traj in trajectory_list:
            name = traj['name']
            wp_names = traj['waypoints']
            self.trajectories[name] = wp_names

        # GUI는 trajectory_combo만 사용하므로 trajectory 이름 목록만 전달
        # (waypoint 콤보박스가 필요하면 emit(list(self.waypoints.keys()), list(self.trajectories.keys()))로
        #  바꾸고 qt_signals.py의 yaml_loaded 시그널 인자도 맞춰줘야 함)
        self.signals.yaml_loaded.emit(list(self.trajectories.keys()))

        self.signals.log_triggered.emit('YAML 로드 완료')
        self.signals.log_triggered.emit(f'Waypoint 개수: {len(self.waypoints)}')
        self.signals.log_triggered.emit(f'Trajectory 개수: {len(self.trajectories)}')

    # /battery_status 콜백
    def battery_callback(self, msg):
        self.last_battery_p = msg.percentage
        self.last_battery_v = msg.voltage
        self.signals.battery_received.emit(msg.percentage * 100.0, msg.voltage)

    # /cmd_vel 발행
    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    # /odom 콜백
    def odom_callback(self, msg):
        self.last_odom = msg
        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.signals.odom_received.emit(p.x, p.y, yaw)

    # /scan 콜백
    # 원본에는 values를 계산해놓고 실제로는 쓰지 않은 채 self.last_scan_min(항상 None)을
    # 그대로 emit하는 버그가 있었음 -> 계산한 값을 저장 후 emit하도록 수정
    def scan_callback(self, msg):
        values = [
            v for v in msg.ranges
            if math.isfinite(v) and v > 0.0
        ]
        self.last_scan_min = min(values) if values else None
        self.signals.scan_received.emit(self.last_scan_min)

    # /initialpose 발행
    def publish_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)

        yaw_rad = float(yaw)
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # 공분산: 위치 추정 초기 불확실도
        msg.pose.covariance[0] = 0.25      # x 오차
        msg.pose.covariance[7] = 0.25      # y 오차
        msg.pose.covariance[35] = 0.0685   # 각도 오차

        self.initial_pose_pub.publish(msg)

    # waypoint 이름 -> PoseStamped 변환
    def make_pose(self, waypoint_name):
        wp = self.waypoints[waypoint_name]

        frame_id = wp.get('frame_id', 'map')
        position = wp['pose']['position']
        angle = wp['pose']['angle']

        x = float(position['x'])
        y = float(position['y'])
        z = 0.0  # TurtleBot3 Burger는 2D 주행 로봇이므로 z는 0으로 고정

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
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose

    # 단일 waypoint 이동 요청
    def go_to_waypoint(self, waypoint_name):
        if waypoint_name == '':
            return False, '선택된 waypoint가 없습니다.'

        if not self.navigate_client.wait_for_server(timeout_sec=1.0):
            return False, '/navigate_to_pose 액션 서버가 준비되지 않았습니다.'

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose(waypoint_name)
        goal_msg.behavior_tree = ''

        self.signals.log_triggered.emit(f'Waypoint 이동 요청: {waypoint_name}')

        future = self.navigate_client.send_goal_async(goal_msg)
        future.add_done_callback(self.waypoint_goal_response)

        return True, f'Waypoint 이동 요청 송신 완료: {waypoint_name}'

    def waypoint_goal_response(self, future):
        goal_handle = future.result()
        self.goal_handle = goal_handle

        if not goal_handle.accepted:
            self.signals.log_triggered.emit('Waypoint goal이 거부되었습니다.')
            return

        self.signals.log_triggered.emit('Waypoint goal이 수락되었습니다.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.waypoint_result)

    def waypoint_result(self, future):
        self.signals.log_triggered.emit('Waypoint 이동 완료')

    def cancel_goal(self):
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
            return True
        return False

    # trajectory(경유점 순차 주행) 이동 요청
    # 원래 gui.py의 go_to_trajectory 안에 있던 로직인데, FollowWaypoints import도 안 되어 있고
    # self.make_pose / self.waypoints / self.follow_client를 GUI 자신의 속성처럼 참조하고 있어서
    # 그대로 실행하면 무조건 에러가 나는 코드였음 -> go_to_waypoint와 동일한 패턴으로 노드 쪽에 정리
    def go_to_trajectory(self, traj_name):
        if traj_name == '':
            return False, '선택된 trajectory가 없습니다.'

        if traj_name not in self.trajectories:
            return False, 'trajectory 정보가 없습니다.'

        if not self.follow_client.wait_for_server(timeout_sec=1.0):
            return False, '/follow_waypoints 액션 서버가 준비되지 않았습니다.'

        waypoint_names = self.trajectories[traj_name]
        poses = []
        for name in waypoint_names:
            if name not in self.waypoints:
                return False, f'YAML에 없는 waypoint입니다: {name}'
            poses.append(self.make_pose(name))

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.signals.log_triggered.emit(f'Trajectory 주행 요청: {traj_name}')
        self.signals.log_triggered.emit(f'포함된 waypoint 개수: {len(poses)}')

        future = self.follow_client.send_goal_async(goal_msg)
        future.add_done_callback(self.trajectory_goal_response)

        return True, f'Trajectory 이동 요청 송신 완료: {traj_name}'

    def trajectory_goal_response(self, future):
        goal_handle = future.result()
        self.goal_handle = goal_handle

        if not goal_handle.accepted:
            self.signals.log_triggered.emit('Trajectory goal이 거부되었습니다.')
            return

        self.signals.log_triggered.emit('Trajectory goal이 수락되었습니다.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.trajectory_result)

    def trajectory_result(self, future):
        self.signals.log_triggered.emit('Trajectory 주행 완료')