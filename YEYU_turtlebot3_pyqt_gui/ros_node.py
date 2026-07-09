import math  
import yaml
from rclpy.node import Node  
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# === geometry_msgs 관련 메시지들 ===
from geometry_msgs.msg import Twist                       # /cmd_vel용
from geometry_msgs.msg import PoseStamped                 # make_pose()용
from geometry_msgs.msg import PoseWithCovarianceStamped   # /initialpose용

# === nav_msgs 관련 메시지들 ===
from nav_msgs.msg import Odometry                         # /odom용

# === sensor_msgs 관련 메시지들 ===
from sensor_msgs.msg import LaserScan                     # /scan용
from sensor_msgs.msg import BatteryState                  # /battery_status용

# === nav2_msgs 관련 액션들 ===
from nav2_msgs.action import NavigateToPose               # 액션클라이언트용
from nav2_msgs.action import FollowWaypoints              # 액션클라이언트용

# === qt_signals.py ===
from qt_signals import RosSignals

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class TurtleBot3RosNode(Node):
    def __init__(self):                                     # 클래스 생성자. 

        # yaml파일 불러와야함

        super().__init__('turtlebot3_ros_node')                         # 부모 생성자 호출 + 노드 이름 지정
        self.signals = RosSignals()

        # pyqt노드에서 
        self.yaml_file = ''

        # yaml파일에서 가져올 정보
        self.waypoints = {}         # 단일 목적지 저장할 딕셔너리
        self.trajectories = {}      # 경로(목적지들의 묶음) 저장할 딕셔너리

        # /battery_status 수신자 생성
        self.battery_sub = self.create_subscription(
            BatteryState, 
            '/battery_status',
            self.battery_callback,
            10
        )
        self.last_battery_p = None
        self.last_battery_v = None
                            

        # /cmd_vel 발행자 생성
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # /odom 수신자 생성
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.last_odom = None

        # /initialpose 발행자 생성
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        '''
        # navigate_to_pose 액션클라이언트 생성
        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        self.goal_handle = None
        '''

        #/scan 수신
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
        self.last_scan_min = None


        ''''[경유점 순자 주행]'''
        # follow_waypoints 액션클라이언트 생성
        self.follow_client = ActionClient(
            self,
            FollowWaypoints,
            'follow_waypoints'
        )

	# YAML 파일 읽어서 데이터 채우는 함수
    def load_yaml(self):
		# YAML 내용 -> 파이썬 딕셔너리 구조로 변환 - data에 저장
        with open(self.yaml_file, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

		# data에서 읽은 데이터 waypoint_list,trajectory_list에 저장
        ''' 
        # waypoint_list의 실제 모습
        waypoint_list = [
            {"name": "point1", "frame_id": "map", "pose": {...}},  # [0]번 상자
            {"name": "point2", "frame_id": "map", "pose": {...}}   # [1]번 상자
        ]
        '''
        waypoint_list = data['waypoints']
        trajectory_list = data['trajectories']
				
		# waypoint_list 리스트 -> self.waypoints 딕셔너리로 정제
        for wp in waypoint_list:
            name = wp['name']                 # waypoint_list에 있는 첫번째 딕셔너리에서 name 내용 ex. point1
            self.waypoints[name] = wp         # 딕셔너리에 데이터 추가(키,값)
            '''self.waypoint_combo.addItem(name)   #nav2_waypoint_gui.py에 있는 내용(GUI에 해당)
               ㄴ> self.signals.log_triggered.emit(list(self.waypoints.keys()))
            '''

		# trajectory_list 리스트 -> self.trajectories 딕셔너리로 정제
        for traj in trajectory_list:
            name = traj['name']
            wp_names = traj['waypoints']
            self.trajectories[name] = wp_names
            '''self.trajectory_combo.addItem(name) #nav2_waypoint_gui.py에 있는 내용(GUI에 해당)
                ㄴ> self.signals.log_triggered.emit(list(self.trajectories.keys()))
            '''

        self.signals.yaml_loaded.emit(list(self.waypoints.keys()), list(self.trajectories.keys()))

		# GUI에 로그 출력
        ''' #nav2_waypoint_gui.py에 있는 내용(GUI에 해당)
        self.log('YAML 로드 완료')
        self.log(f'Waypoint 개수: {len(self.waypoints)}')
        self.log(f'Trajectory 개수: {len(self.trajectories)}')
        '''
        self.signals.log_triggered.emit('YAML 로드 완료')
        self.signals.log_triggered.emit(f'Waypoint 개수: {len(self.waypoints)}')
        self.signals.log_triggered.emit(f'Trajectory 개수: {len(self.trajectories)}')
        
    # [추가] battery_status 콜백함수 정의
    def battery_callback(self, msg):
        # QTimer->멀티스레드 방식으로 바꾸면서 ui_imter의 플롯함수 다 흩어짐
        # self.last_battery_p = msg.percentage 
        # self.last_battery_v = msg.voltage

        self.signals.battery_received.emit(msg.percentage * 100.0, msg.voltage)


    # cmd_topic 발행함수 정의
    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    # odom_topic 수신받았을 때 콜백함수 정의
    def odom_callback(self, msg):
        # self.last_odom = msg        # 받은 msg를 lase_odom에 저장
        p = msg.pose.pose.position                           
        yaw = quaternion_to_yaw(msg.pose.pose.orientation) 

        self.signals.odom_received.emit(p.x, p.y, yaw)

    # # initpos_topic 발행함수 정의
    # def publish_initial_pose(self, x, y, yaw):
    #     msg = PoseWithCovarianceStamped()

    #     msg.header.frame_id = 'map'                # 이 좌표의 기준 'map'으로 지정
    #     msg.header.stamp = self.get_clock().now().to_msg() # msg가 발행되는 현재 컴퓨터 시간의 타임스탬프 찍어줌 

    #     msg.pose.pose.position.x = float(x)        # 입력한 위치를 메세지 주머니에 대입
    #     msg.pose.pose.position.y = float(y)

    #     qx, qy, qz, qw = yaw_to_quaternion(yaw)    # 각도->쿼터니언 값 qx,qy,qz,qw에 담아
    #     msg.pose.pose.orientation.x = qx           # 쿼터니언 값 주머니에 채움
    #     msg.pose.pose.orientation.y = qy
    #     msg.pose.pose.orientation.z = qz
    #     msg.pose.pose.orientation.w = qw

    #     # 공분산 ; "내가 지금 찍어준 이 위치가 얼마나 불확실한가"에 대한 에러 확률 지표
    #     ''' -> 자율주행 알고리즘(AMCL)이 이를 기반으로 로봇 주변 파티클 흩뿌려 위치 추정 시작할 수 있게 됨'''
    #     msg.pose.covariance[0] = 0.25      # X오차
    #     msg.pose.covariance[7] = 0.25      # y오차
    #     msg.pose.covariance[35] = 0.0685   # 각도 오차

    #     # 발행
    #     self.initial_pose_pub.publish(msg)   
    
    # # navpose_topic 액션클라이언트 ; goal 전송 함수
    # def send_goal(self, x, y, yaw):
    #     goal_msg = NavigateToPose.Goal()

    #     goal_msg.pose = PoseStamped()
    #     goal_msg.pose.header.frame_id = 'map'
    #     goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

    #     goal_msg.pose.pose.position.x = float(x)
    #     goal_msg.pose.pose.position.y = float(y)

    #     qx, qy, qz, qw = yaw_to_quaternion(yaw)

    #     goal_msg.pose.pose.orientation.x = qx
    #     goal_msg.pose.pose.orientation.y = qy
    #     goal_msg.pose.pose.orientation.z = qz
    #     goal_msg.pose.pose.orientation.w = qw

    #     if not self.navigate_client.wait_for_server(timeout_sec=1.0):
    #         return False, 'Nav2 action server is not available'

    #     future = self.navigate_client.send_goal_async(goal_msg)
    #     future.add_done_callback(self._goal_response_callback)

    #     return True, f'Goal sent: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
    
    # # navpose_topic 액션클라이언트 ; goal 응답 피드백 함수
    # def _goal_response_callback(self, future):
    #     self.goal_handle = future.result()

    #     if self.goal_handle and self.goal_handle.accepted:
    #         self.get_logger().info('Goal accepted')
    #     else:
    #         self.get_logger().warn('Goal rejected')

    # 2. waypoint_button 클릭의 슬롯함수 ; 단일 목적지 액션 통신 기능
    def go_to_waypoint(self,waypoint_name):

        if waypoint_name == '':
            self.signals.log_triggered.emit('선택된 waypoint가 없습니다.')
            return

		# 1초동안 Nav2 액션서버 켜져있는지 확인 -> 안켜져있음 빠져나감
        if not self.navigate_client.wait_for_server(timeout_sec=1.0):
            self.signals.log_triggered.emit('/navigate_to_pose 액션 서버가 준비되지 않았습니다.')
            return

        goal_msg = NavigateToPose.Goal()                 # 액션 목적지 NavigateToPose메세지 생성
        goal_msg.pose = self.make_pose(waypoint_name)    # waypoint_name -> msg.pose(goal)
        goal_msg.behavior_tree = ''                      # msg.behavior_tree 공란으로 설정

        self.signals.log_triggered.emit(f'Waypoint 이동 요청: {waypoint_name}')

        # navigat_client에게 비동기(async) 명령 - goal_msg 좌표로 이동해줘
        future = self.navigate_client.send_goal_async(goal_msg)
        future.add_done_callback(self.waypoint_goal_response) # 서버가 수락거절 응답 오면 -> (4) waypoint_goal_response 실행

        # 요청이 정상적으로 액션 서버에 송신되었음을 GUI에 알림 (튜플 반환)
        return True, f'Waypoint 이동 요청 송신 완료: {waypoint_name}'

     # go_to_waypoints() 내부에 호출
    def make_pose(self, waypoint_name):
        wp = self.waypoints[waypoint_name]     # self.waypoints 중 선택한 목적지(waypoint_name) -> wp

        frame_id = wp.get('frame_id', 'map')   # wp 안에 기준좌표계(없으면 map) -> frame_id

        position = wp['pose']['position']      # wp 안에 x,y 데이터 -> position
        angle = wp['pose']['angle']            # wp 안에 yaw 데이터 -> angle

		# position 안에 x,y 저장
        x = float(position['x'])               
        y = float(position['y'])              

        z = 0.0                                 # TurtleBot3 Burger는 2D 주행 로봇이므로 z는 0으로 고정한다.

		# angle -> 쿼터니언 yaw_rad로 
        yaw_deg = float(angle['yaw'])
        yaw_rad = math.radians(yaw_deg)

        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)

        pose = PoseStamped()                   # ROS2 표준 위치 msg 객체생성
        pose.header.frame_id = frame_id        # 좌표계 주입
        pose.header.stamp = self.get_clock().now().to_msg()  # ROS2 타임스탬프 주입

		# pose에 x,y,z,yaw 저장
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose


    # 2-1 navigate_client가 요청-> navigate_server 응답시 콜백함수
    def waypoint_goal_response(self, future):
        goal_handle = future.result()              # future.result(요청결과) -> goal_handle

        # 서버 명령 거절
        if not goal_handle.accepted:                
            self.signals.log_triggered.emit('Waypoint goal이 거부되었습니다.')
            return

        # 서버 명령 수락
        self.signals.log_triggered.emit('Waypoint goal이 수락되었습니다.')

        # navigat_client에게 비동기(async) 명령 - 방금 주문 실시간 공유해줘
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.waypoint_result) # 도착 완료 응답 -> (5) waypoint_result 콜백

    # 2-2 in(4) navigate_server 응답시 콜백함수
    def waypoint_result(self, future):
        self.signals.log_triggered.emit('Waypoint 이동 완료')  # 도착 성공 로그

        '''
        # navpose_topic 액션클라이언트 ; goal 취소
        def cancel_goal(self):
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
                return True

            return False
        '''    
    # scan_topic 콜백함수
    def scan_callback(self, msg):
        values = [
            v for v in msg.ranges
            if math.isfinite(v) and v > 0.0
        ]

        # QTimer 대신 멀티스레드로 바뀌면서 ui_timer의 슬롯 함수 흩어져서 여기도 수정
        #self.last_scan_min = min(values) if values else None

        self.signals.scan_received.emit(self.last_scan_min)

















    # QTimer 대신 멀티스레드로 변경
    # # 6. ros_timer 타이머 울릴 때마다의 슬롯
    # def spin_ros_once(self):
    #     if self.node:                                                      
    #         rclpy.spin_once(self.node, timeout_sec=0.0)                    

    # 7. load_preset_PB 시그널의 슬롯
    def load_preset_goal(self):
        idx = self.preset_goal_CB.currentIndex()                           

        presets = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 1.57)
        ]

        x, y, yaw = presets[idx]             

        self.goal_x_spinBox.setValue(x)
        self.goal_y_spinBox.setValue(y)
        self.goal_yaw_spinBox.setValue(yaw)

        print(f'Preset loaded: x={x}, y={y}, yaw={yaw}')


    # refresh_robot_status(self) 부분을 update_odom_ui(),update_scan_ui(),update_battery_ui()로 넘어감
    # # 8. ui_timer 타이머 울릴때마다의 슬롯
    # def refresh_robot_status(self):
    #     if not self.node:                       
    #         return                              

    #     odom = self.node.last_odom              
        
    #     if odom:                                                  
    #         p = odom.pose.pose.position                           
    #         yaw = quaternion_to_yaw(odom.pose.pose.orientation)   

    #         self.odom_x_lcd.display(f'{p.x:.2f}')                 
    #         self.odom_y_lcd.display(f'{p.y:.2f}')                 
    #         self.odom_yaw_lcd.display(f'{yaw:.2f}')               

    #     if self.node.last_scan_min is not None:
    #         self.scan_lineEdit.setText(f'{self.node.last_scan_min:.2f}')    

    #         '''[추가] 배터리 정보 UI 및 상태창 반영'''
    #     if self.node.last_battery_p is not None and self.node.last_battery_v is not None:
    #         # 1. 만약 UI 파일에 배터리를 표시할 LineEdit 위젯 등이 있다면 아래 주석을 풀어서 사용하세요.
    #         self.battery_lineEdit.setText(f'{self.node.last_battery_p * 100.0:.1f}% ({self.node.last_battery_v:.2f}V)')
            
    #         # 2. 하단 리스트 위젯에 주기적으로 배터리 로그 남기기 (원치 않을 시 주석처리 가능)
    #         self.log(f'[Battery] {self.node.last_battery_p * 100.0:.1f}%, Voltage: {self.node.last_battery_v:.2f}V')
    
    # 9. reset_odom_view_PB 시그널의 슬롯 함수 ; Odometry View 화면 리셋
    def reset_odom_display(self):
        self.odom_x_lcd.display('0.00')
        self.odom_y_lcd.display('0.00')
        self.odom_yaw_lcd.display('0.00')
        self.log('Odometry display reset. Robot odom frame is not reset.')

    # 10. initial_pose_PB 시그널의 슬롯함수
    def set_initial_pose(self):
        if not self.node:
            self.log('Connect ROS 2 first')
            return

        self.node.publish_initial_pose(
            self.goal_x_spinBox.value(),
            self.goal_y_spinBox.value(),
            self.goal_yaw_spinBox.value()
        )

        self.log('Initial pose published to /initialpose')
    
    # 11. 
    def send_nav_goal(self):
        if not self.node:
            self.log('Connect ROS 2 first')
            return
        
        waypoint_name = self.waypoint_combo.currentText()  # GUI창 waypoint_combo에서 선택한 값 -> way

        if waypoint_name == '':
            self.log('선택된 waypoint가 없습니다.')
            return

        ok, text = self.node.go_to_waypoint(waypoint_name)

        self.log(text)

    '''
        # 12. 
        def cancel_nav_goal(self):
            if not self.node:
                self.log('Connect ROS 2 first')
                return

            if self.node.cancel_goal():
                self.log('Goal cancel requested')
            else:
                self.log('No active goal handle')
    '''

    # 13. yaml_loaded 신호 시그널의 플롯 함수
    def update_comboboxes(self,wp_names,traj_names):
        # 콤보박스 초기화 후 데이터 추가
        self.waypoint_combo.clear()
        self.trajectory_combo.clear()

        # UI 작업 수행
        self.waypoint_combo.addItems(wp_names)
        self.trajectory_combo.addItems(traj_names)

        # (3) 첫번째 경로 정보 화면에 띄우는 함수 호출
        self.show_trajectory_info()

    # 13-(1) 내부에 호출 ; 첫번째 경로 정보 화면에 띄우는 함수
    def show_trajectory_info(self):
        traj_name = self.trajectory_combo.currentText()   # 현재 선택된 경로 이름 가져옴 

        if traj_name in self.trajectories:
            wp_names = self.trajectories[traj_name]
            text = ' -> '.join(wp_names)            # 예: ['point1', 'point2'] 상태를 "point1 -> point2" 형태의 문자열로
            self.trajectory_label.setText(text)     # 화면에 경로순서 표시

    # 14. print 대신 self.log()로
    def log(self, text):
        self.log_listWidget.addItem(text)
        self.log_listWidget.scrollToBottom()

    # 15.odom_received 시그널의 플롯함수
    def update_odom_ui(self, x, y, yaw):
        self.odom_x_lcd.display(f'{x:.2f}')
        self.odom_y_lcd.display(f'{y:.2f}')
        self.odom_yaw_lcd.display(f'{yaw:.2f}')

    # 16. scan_received 시그널의 플롯함수
    def update_scan_ui(self, min_scan):
        self.scan_lineEdit.setText(f'{min_scan:.2f}') 

    # 17. battery_received 시그널의 플롯함수
    def update_battery_ui(self, percentage, voltage):
        try:
            self.battery_lineEdit.setText(f'{percentage:.1f}% ({voltage:.2f}V)')
        except AttributeError:
            pass

    # 100. 
    def closeEvent(self, event):
        if self.node:
            self.send_velocity(0.0, 0.0)

        self.stop_all_processes()
        self.disconnect_ros()

        if rclpy.ok():
            rclpy.shutdown()

        event.accept()

    def __init__(self):                                     # 클래스 생성자. 

        # yaml파일 불러와야함

        super().__init__('turtlebot3_ros_node')                         # 부모 생성자 호출 + 노드 이름 지정
        self.signals = RosSignals()

        # pyqt노드에서 
        self.yaml_file = ''

        # yaml파일에서 가져올 정보
        self.waypoints = {}         # 단일 목적지 저장할 딕셔너리
        self.trajectories = {}      # 경로(목적지들의 묶음) 저장할 딕셔너리

        # /battery_status 수신자 생성
        self.battery_sub = self.create_subscription(
            BatteryState, 
            '/battery_status',
            self.battery_callback,
            10
        )
        self.last_battery_p = None
        self.last_battery_v = None
                            

        # /cmd_vel 발행자 생성
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # /odom 수신자 생성
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.last_odom = None

        # /initialpose 발행자 생성
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # navigate_to_pose 액션클라이언트 생성
        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        self.goal_handle = None

        '''[/scan 수신]'''
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # scan 수신자 정의
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        self.last_scan_min = None

        ''''[경유점 순자 주행]'''
        # follow_waypoints 액션클라이언트 생성
        self.follow_client = ActionClient(
            self,
            FollowWaypoints,
            'follow_waypoints'
        )

	# YAML 파일 읽어서 데이터 채우는 함수
    def load_yaml(self):
		# YAML 내용 -> 파이썬 딕셔너리 구조로 변환 - data에 저장
        with open(self.yaml_file, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

		# data에서 읽은 데이터 waypoint_list,trajectory_list에 저장
        ''' 
        # waypoint_list의 실제 모습
        waypoint_list = [
            {"name": "point1", "frame_id": "map", "pose": {...}},  # [0]번 상자
            {"name": "point2", "frame_id": "map", "pose": {...}}   # [1]번 상자
        ]
        '''
        waypoint_list = data['waypoints']
        trajectory_list = data['trajectories']
				
		# waypoint_list 리스트 -> self.waypoints 딕셔너리로 정제
        for wp in waypoint_list:
            name = wp['name']                 # waypoint_list에 있는 첫번째 딕셔너리에서 name 내용 ex. point1
            self.waypoints[name] = wp         # 딕셔너리에 데이터 추가(키,값)
            '''self.waypoint_combo.addItem(name)  # GUI 콤보박스(waypoint_combo)에 추가
               ㄴ> self.signals.log_triggered.emit(list(self.waypoints.keys()))
            '''

		# trajectory_list 리스트 -> self.trajectories 딕셔너리로 정제
        for traj in trajectory_list:
            name = traj['name']
            wp_names = traj['waypoints']
            self.trajectories[name] = wp_names
            '''self.trajectory_combo.addItem(name)
                ㄴ> self.signals.log_triggered.emit(list(self.trajectories.keys()))
            '''

        self.signals.yaml_loaded.emit(list(self.waypoints.keys()), list(self.trajectories.keys()))

		# GUI에 로그 출력
        '''
        self.log('YAML 로드 완료')
        self.log(f'Waypoint 개수: {len(self.waypoints)}')
        self.log(f'Trajectory 개수: {len(self.trajectories)}')
        '''
        self.signals.log_triggered.emit('YAML 로드 완료')
        self.signals.log_triggered.emit(f'Waypoint 개수: {len(self.waypoints)}')
        self.signals.log_triggered.emit(f'Trajectory 개수: {len(self.trajectories)}')
        
    # [추가] battery_status 콜백함수 정의
    def battery_callback(self, msg):
        # QTimer->멀티스레드 방식으로 바꾸면서 ui_imter의 플롯함수 다 흩어짐
        # self.last_battery_p = msg.percentage 
        # self.last_battery_v = msg.voltage

        self.signals.battery_received.emit(msg.percentage * 100.0, msg.voltage)


    # cmd_topic 발행함수 정의
    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    # odom_topic 수신받았을 때 콜백함수 정의
    def odom_callback(self, msg):
        # self.last_odom = msg        # 받은 msg를 lase_odom에 저장
        p = msg.pose.pose.position                           
        yaw = quaternion_to_yaw(msg.pose.pose.orientation) 

        self.signals.odom_received.emit(p.x, p.y, yaw)

    # # initpos_topic 발행함수 정의
    # def publish_initial_pose(self, x, y, yaw):
    #     msg = PoseWithCovarianceStamped()

    #     msg.header.frame_id = 'map'                # 이 좌표의 기준 'map'으로 지정
    #     msg.header.stamp = self.get_clock().now().to_msg() # msg가 발행되는 현재 컴퓨터 시간의 타임스탬프 찍어줌 

    #     msg.pose.pose.position.x = float(x)        # 입력한 위치를 메세지 주머니에 대입
    #     msg.pose.pose.position.y = float(y)

    #     qx, qy, qz, qw = yaw_to_quaternion(yaw)    # 각도->쿼터니언 값 qx,qy,qz,qw에 담아
    #     msg.pose.pose.orientation.x = qx           # 쿼터니언 값 주머니에 채움
    #     msg.pose.pose.orientation.y = qy
    #     msg.pose.pose.orientation.z = qz
    #     msg.pose.pose.orientation.w = qw

    #     # 공분산 ; "내가 지금 찍어준 이 위치가 얼마나 불확실한가"에 대한 에러 확률 지표
    #     ''' -> 자율주행 알고리즘(AMCL)이 이를 기반으로 로봇 주변 파티클 흩뿌려 위치 추정 시작할 수 있게 됨'''
    #     msg.pose.covariance[0] = 0.25      # X오차
    #     msg.pose.covariance[7] = 0.25      # y오차
    #     msg.pose.covariance[35] = 0.0685   # 각도 오차

    #     # 발행
    #     self.initial_pose_pub.publish(msg)   
    
    # # navpose_topic 액션클라이언트 ; goal 전송 함수
    # def send_goal(self, x, y, yaw):
    #     goal_msg = NavigateToPose.Goal()

    #     goal_msg.pose = PoseStamped()
    #     goal_msg.pose.header.frame_id = 'map'
    #     goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

    #     goal_msg.pose.pose.position.x = float(x)
    #     goal_msg.pose.pose.position.y = float(y)

    #     qx, qy, qz, qw = yaw_to_quaternion(yaw)

    #     goal_msg.pose.pose.orientation.x = qx
    #     goal_msg.pose.pose.orientation.y = qy
    #     goal_msg.pose.pose.orientation.z = qz
    #     goal_msg.pose.pose.orientation.w = qw

    #     if not self.navigate_client.wait_for_server(timeout_sec=1.0):
    #         return False, 'Nav2 action server is not available'

    #     future = self.navigate_client.send_goal_async(goal_msg)
    #     future.add_done_callback(self._goal_response_callback)

    #     return True, f'Goal sent: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
    
    # # navpose_topic 액션클라이언트 ; goal 응답 피드백 함수
    # def _goal_response_callback(self, future):
    #     self.goal_handle = future.result()

    #     if self.goal_handle and self.goal_handle.accepted:
    #         self.get_logger().info('Goal accepted')
    #     else:
    #         self.get_logger().warn('Goal rejected')

    # 2. waypoint_button 클릭의 슬롯함수 ; 단일 목적지 액션 통신 기능
    def go_to_waypoint(self,waypoint_name):

        if waypoint_name == '':
            self.signals.log_triggered.emit('선택된 waypoint가 없습니다.')
            return

		# 1초동안 Nav2 액션서버 켜져있는지 확인 -> 안켜져있음 빠져나감
        if not self.navigate_client.wait_for_server(timeout_sec=1.0):
            self.signals.log_triggered.emit('/navigate_to_pose 액션 서버가 준비되지 않았습니다.')
            return

        goal_msg = NavigateToPose.Goal()                 # 액션 목적지 NavigateToPose메세지 생성
        goal_msg.pose = self.make_pose(waypoint_name)    # waypoint_name -> msg.pose(goal)
        goal_msg.behavior_tree = ''                      # msg.behavior_tree 공란으로 설정

        self.signals.log_triggered.emit(f'Waypoint 이동 요청: {waypoint_name}')

        # navigat_client에게 비동기(async) 명령 - goal_msg 좌표로 이동해줘
        future = self.navigate_client.send_goal_async(goal_msg)
        future.add_done_callback(self.waypoint_goal_response) # 서버가 수락거절 응답 오면 -> (4) waypoint_goal_response 실행

        # 요청이 정상적으로 액션 서버에 송신되었음을 GUI에 알림 (튜플 반환)
        return True, f'Waypoint 이동 요청 송신 완료: {waypoint_name}'

     # go_to_waypoints() 내부에 호출
    def make_pose(self, waypoint_name):
        wp = self.waypoints[waypoint_name]     # self.waypoints 중 선택한 목적지(waypoint_name) -> wp

        frame_id = wp.get('frame_id', 'map')   # wp 안에 기준좌표계(없으면 map) -> frame_id

        position = wp['pose']['position']      # wp 안에 x,y 데이터 -> position
        angle = wp['pose']['angle']            # wp 안에 yaw 데이터 -> angle

		# position 안에 x,y 저장
        x = float(position['x'])               
        y = float(position['y'])              

        z = 0.0                                 # TurtleBot3 Burger는 2D 주행 로봇이므로 z는 0으로 고정한다.

		# angle -> 쿼터니언 yaw_rad로 
        yaw_deg = float(angle['yaw'])
        yaw_rad = math.radians(yaw_deg)

        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)

        pose = PoseStamped()                   # ROS2 표준 위치 msg 객체생성
        pose.header.frame_id = frame_id        # 좌표계 주입
        pose.header.stamp = self.get_clock().now().to_msg()  # ROS2 타임스탬프 주입

		# pose에 x,y,z,yaw 저장
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose


    # 2-1 navigate_client가 요청-> navigate_server 응답시 콜백함수
    def waypoint_goal_response(self, future):
        goal_handle = future.result()              # future.result(요청결과) -> goal_handle

        # 서버 명령 거절
        if not goal_handle.accepted:                
            self.signals.log_triggered.emit('Waypoint goal이 거부되었습니다.')
            return

        # 서버 명령 수락
        self.signals.log_triggered.emit('Waypoint goal이 수락되었습니다.')

        # navigat_client에게 비동기(async) 명령 - 방금 주문 실시간 공유해줘
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.waypoint_result) # 도착 완료 응답 -> (5) waypoint_result 콜백

    # 2-2 in(4) navigate_server 응답시 콜백함수
    def waypoint_result(self, future):
        self.signals.log_triggered.emit('Waypoint 이동 완료')  # 도착 성공 로그

        '''
        # navpose_topic 액션클라이언트 ; goal 취소
        def cancel_goal(self):
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
                return True

            return False
        '''    
    # scan_topic 콜백함수
    def scan_callback(self, msg):
        values = [
            v for v in msg.ranges
            if math.isfinite(v) and v > 0.0
        ]

        # QTimer 대신 멀티스레드로 바뀌면서 ui_timer의 슬롯 함수 흩어져서 여기도 수정
        #self.last_scan_min = min(values) if values else None

        self.signals.battery_received.emit(msg.percentage * 100.0, msg.voltage)
