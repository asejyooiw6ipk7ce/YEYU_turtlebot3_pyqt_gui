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
from nav2_msgs.action import FollowWaypoints              # trajectory 이동용

# === audio_msgs 관련 액션들 ===
from robot_audio_interfaces.msg import AudioCommand       

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

        # /audio/command 발행자 생성 ; 변형) 명령어 -topic_name:= 뒤에 원하는 토픽명 정할 수 잇음
        self.publisher = self.create_publisher(
            AudioCommand,
            '/audio/command',
            10
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

        self.signals.yaml_loaded.emit(list(self.waypoints.keys()), list(self.trajectories.keys()))

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

    # waypoint 이름 -> PoseStamped 변환
    def make_pose(self, waypoint_name):
        wp = self.waypoints[waypoint_name]

        frame_id = wp.get('frame_id', 'map')

        x = float(wp['x'])
        y = float(wp['y'])
        z = 0.0  # TurtleBot3 Burger는 2D 주행 로봇이므로 z는 0으로 고정

        yaw_rad = float(wp['yaw'])
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

   # 2. waypoint_button 클릭의 슬롯함수 ; 단일 목적지 액션 통신 기능
    def go_to_waypoint(self, waypoint_name):

        if not self.navigate_client.wait_for_server(timeout_sec=1.0):
            return False, '/navigate_to_pose 액션 서버가 준비되지 않았습니다.'

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose(waypoint_name)
        goal_msg.behavior_tree = ''

        self.signals.log_triggered.emit(f'Waypoint 이동 요청: {waypoint_name}')

        future = self.navigate_client.send_goal_async(goal_msg)
        future.add_done_callback(self.waypoint_goal_response)

        return True, f'Waypoint 이동 요청 송신 완료: {waypoint_name}'

    # 2-(4) navigate_client가 요청-> navigate_server 응답시 콜백함수
    def waypoint_goal_response(self, future):
        goal_handle = future.result()
        self.goal_handle = goal_handle

        if not goal_handle.accepted:
            self.signals.log_triggered.emit('Waypoint goal이 거부되었습니다.')
            return

        self.signals.log_triggered.emit('Waypoint goal이 수락되었습니다.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.waypoint_result)

    # 2-(5) in(4) navigate_server 응답시 콜백함수
    def waypoint_result(self, future):
        self.signals.log_triggered.emit('Waypoint 이동 완료')

    '''
    def cancel_goal(self):
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
            return True
        return False
    '''
    
    def go_to_trajectory(self, traj_name):

        # 1초동안 액션서버 켜져있는지 확인
        if not self.follow_client.wait_for_server(timeout_sec=1.0):
            return False, '/follow_waypoints 액션 서버가 준비되지 않았습니다.'

        # self.trajectories에서 선택항목만 -> waypoint_names
        waypoint_names = self.trajectories[traj_name]

        poses = []

        for name in waypoint_names:
            if name not in self.waypoints:
                return False, f'YAML에 없는 waypoint입니다: {name}'
            
            pose = self.make_pose(name)
            poses.append(pose)

        # Goal 메세지
        goal_msg = FollowWaypoints.Goal()   # 순차주행용 액션 goal 메세지 생성 -> goal_msg
        goal_msg.poses = poses              # goal_msg에 목표값 poses 주입

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

    # /audio/command 콜백함수
    def publish_command(self, command_type, text='', sound_id='', volume=1.0, repeat=1 ):
        msg = AudioCommand()        # 주문서 양식: AudioCommand()
        msg.type = command_type     # 매개변수로 받은 값들 msg에 채워넣기
        msg.text = text
        msg.sound_id = sound_id
        msg.volume = float(volume)
        msg.repeat = int(repeat)

        self.publisher.publish(msg)