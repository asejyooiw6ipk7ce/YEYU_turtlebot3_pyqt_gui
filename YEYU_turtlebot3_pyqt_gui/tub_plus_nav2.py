import sys
import os                                                                 # os의 환경 변수를 ROS_DOMAIN_ID 값으로 저장하기 위해
import subprocess
import signal 
import math                                                               # yaw 계산(도->쿼터니언)
import rclpy                                                              # ros2 client library for python (TurtleBot3GuiNode)
from rclpy.node import Node                                               # Node 클래스 (TurtleBot3GuiNode(Node))
from geometry_msgs.msg import Twist                                       # msg
from nav_msgs.msg import Odometry                                         # msg
from pathlib import Path
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget
from PyQt5.QtCore import QTimer                                           # PYQT5 루프를 굴리면서 ROS2 루프 굴리기 위해 QTimer 데려옴
from PyQt5.QtCore import QObject, pyqtSignal                              # RosSignals 클래스 추가시 필요한 것들  
from geometry_msgs.msg import PoseWithCovarianceStamped                   # ROS2 내비게이션 시스템에서 로봇 위치와 방향 전달할 때 사용되는 msg규격(초기위치 지정할 때 반드시 이 형식으로 보내야 로봇이 이해)
from rclpy.action import ActionClient                                     # /navigate_to_pose
from nav2_msgs.action import NavigateToPose                               # /navigate_to_pose
from geometry_msgs.msg import PoseStamped                                 # /navigate_to_pose
from nav2_msgs.action import FollowWaypoints  # /follow_waypoints
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import BatteryState                              # [추가] 배터리 상태 토픽 메시지 규격


ROBOT_USER = "yeyu"
ROBOT_IP = "192.168.230.100"

ROBOT = f"{ROBOT_USER}@{ROBOT_IP}"


# ros2 콜백 - PyQt 테이터 전달
class RosSignals(QObject):
	# 문자열 전달할 수 있는 Qt 시그널 정의
    yaml_loaded = pyqtSignal(list)      
    log_triggered = pyqtSignal(str)
    '''신호통로    이 통로로는 list 데이터만 보낼거야(통로 종류 지정) '''


class TurtleBot3GuiNode(Node):
    def __init__(self):                                     # 클래스 생성자. 

        ''' 다른 방식으로 변경(미완성)
        # 인자인 yaml 설정
        self.yaml_file = yaml_file  # 이 노드 객체 생성할 때 받는 인자의 yaml_file을 self.yaml_file에 저장
                                    # 인자로 받는 건 __init__ 나가면 사라짐, self.를 붙이면 이 객체 내부 인스턴스 변수가 됨 
        
        ''' 

        super().__init__('turtlebot3_burger_gui')                         # 부모 생성자 호출 + 노드 이름 지정
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
        self.nav_client = ActionClient(
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
            self.node,
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

        self.signals.log_triggered.emit(list(self.waypoints.keys()))
        self.signals.log_triggered.emit(list(self.trajectories.keys()))

        self.show_trajectory_info()          # (3) 첫번째 경로 정보 화면에 띄우는 함수

		# GUI에 로그 출력
        '''
        self.log('YAML 로드 완료')
        self.log(f'Waypoint 개수: {len(self.waypoints)}')
        self.log(f'Trajectory 개수: {len(self.trajectories)}')
        '''
        self.signals.log_triggered.emit('YAML 로드 완료')
        self.signals.log_triggered.emit(f'Waypoint 개수: {len(self.waypoints)}')
        self.signals.log_triggered.emit(f'Trajectory 개수: {len(self.trajectories)}')

    # load_yaml() 내부에 호출 ; 첫번째 경로 정보 화면에 띄우는 함수
    def show_trajectory_info(self):
        traj_name = self.trajectory_combo.currentText()   # 현재 선택된 경로 이름 가져옴 
        '''2. trajectory_combo'''

        if traj_name in self.trajectories:
            wp_names = self.trajectories[traj_name]
            text = ' -> '.join(wp_names)            # 예: ['point1', 'point2'] 상태를 "point1 -> point2" 형태의 문자열로
            self.trajectory_label.setText(text)     # 화면에 경로순서 표
        '''3. trajectory_label'''

    # show_trajectory_info() 내부에 호출
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
        pose.header.stamp = self.node.get_clock().now().to_msg()  # ROS2 타임스탬프 주입

		# pose에 x,y,z,yaw 저장
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose

    # [추가] battery_status 콜백함수 정의
    def battery_callback(self, msg):
        self.last_battery_p = msg.percentage 
        self.last_battery_v = msg.voltage

    # cmd_topic 발행함수 정의
    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    # odom_topic 수신받았을 때 콜백함수 정의
    def odom_callback(self, msg):
        self.last_odom = msg        # 받은 msg를 lase_odom에 저장

    # initpos_topic 발행함수 정의
    def publish_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()

        msg.header.frame_id = 'map'                # 이 좌표의 기준 'map'으로 지정
        msg.header.stamp = self.get_clock().now().to_msg() # msg가 발행되는 현재 컴퓨터 시간의 타임스탬프 찍어줌 

        msg.pose.pose.position.x = float(x)        # 입력한 위치를 메세지 주머니에 대입
        msg.pose.pose.position.y = float(y)

        qx, qy, qz, qw = yaw_to_quaternion(yaw)    # 각도->쿼터니언 값 qx,qy,qz,qw에 담아
        msg.pose.pose.orientation.x = qx           # 쿼터니언 값 주머니에 채움
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # 공분산 ; "내가 지금 찍어준 이 위치가 얼마나 불확실한가"에 대한 에러 확률 지표
        ''' -> 자율주행 알고리즘(AMCL)이 이를 기반으로 로봇 주변 파티클 흩뿌려 위치 추정 시작할 수 있게 됨'''
        msg.pose.covariance[0] = 0.25      # X오차
        msg.pose.covariance[7] = 0.25      # y오차
        msg.pose.covariance[35] = 0.0685   # 각도 오차

        # 발행
        self.initial_pose_pub.publish(msg)   
    
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

    #     if not self.nav_client.wait_for_server(timeout_sec=1.0):
    #         return False, 'Nav2 action server is not available'

    #     future = self.nav_client.send_goal_async(goal_msg)
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
            self.log('선택된 waypoint가 없습니다.')
            return

		# 1초동안 Nav2 액션서버 켜져있는지 확인 -> 안켜져있음 빠져나감
        if not self.navigate_client.wait_for_server(timeout_sec=1.0):
            self.log('/navigate_to_pose 액션 서버가 준비되지 않았습니다.')
            return

        goal_msg = NavigateToPose.Goal()                 # 액션 목적지 NavigateToPose메세지 생성
        goal_msg.pose = self.make_pose(waypoint_name)    # waypoint_name -> msg.pose(goal)
        goal_msg.behavior_tree = ''                      # msg.behavior_tree 공란으로 설정

        self.log(f'Waypoint 이동 요청: {waypoint_name}')

        # navigat_client에게 비동기(async) 명령 - goal_msg 좌표로 이동해줘
        future = self.navigate_client.send_goal_async(goal_msg)
        future.add_done_callback(self.waypoint_goal_response) # 서버가 수락거절 응답 오면 -> (4) waypoint_goal_response 실행

    # 2-1 navigate_client가 요청-> navigate_server 응답시 콜백함수
    def waypoint_goal_response(self, future):
        goal_handle = future.result()              # future.result(요청결과) -> goal_handle

        # 서버 명령 거절
        if not goal_handle.accepted:                
            self.log('Waypoint goal이 거부되었습니다.')
            return

        # 서버 명령 수락
        self.log('Waypoint goal이 수락되었습니다.')

        # navigat_client에게 비동기(async) 명령 - 방금 주문 실시간 공유해줘
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.waypoint_result) # 도착 완료 응답 -> (5) waypoint_result 콜백

    # 2-2 in(4) navigate_server 응답시 콜백함수
    def waypoint_result(self, future):
        self.log('Waypoint 이동 완료')  # 도착 성공 로그

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

        self.last_scan_min = min(values) if values else None

class TurtleBot3GUI(QWidget):
    def __init__(self,ros_node):
        super().__init__()

        self.signals = RosSignals()
        

        self.node = ros_node

        # turtlebot3_burger_gui.ui 파일 띄우기
        ui_path = Path(__file__).parent.parent / "resource" / "turtlebot3_pyqt_gui2.ui"
        uic.loadUi(str(ui_path), self)

        self.processes = []        # 실행중인 프로세스 보관할 리스트


        self.connect_signals()     # 사용자정의함수 ; 시그널->슬롯 연결

        self.ros_timer = QTimer(self)                                           # QTimer 객체 생성
        self.ros_timer.timeout.connect(self.spin_ros_once)                      # ros2_timer에서 알림이 울리면 -> 6. spin_ros_once 실행

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.refresh_robot_status)                # ui_timer에서 알림이 울리면 -> 8. refresh_robot_status 실행
        self.ui_timer.start(200)                                                # 200ms초마다 울리기

    # print 대신 self.log()로
    def log(self, text):
        self.log_listWidget.addItem(text)
        self.log_listWidget.scrollToBottom()

    # 시그널->슬롯 연결
    def connect_signals(self):
        self.connect_PB.clicked.connect(self.connect_ros)                        # connect_PB 클릭 -> 1. connect_ros 실행
        self.disconnect_PB.clicked.connect(self.disconnect_ros)                  # disconnect_PB 클릭 -> 2. disconnect_ros 실행
        self.exit_PB.clicked.connect(self.closeEvent)                                 # exit_PB 클릭 -> 13. closeEvent 실행
        
        # Launch Control 박스 속 5가지 버튼 시그널 -> 4. run_command() 슬롯 연결 
        self.connect_PB.clicked.connect (self.connect_ros)
        self.nav2_PB.clicked.connect(lambda: self.run__command('nav2',['ros2','launch', 'turtlebot3_navigation2', 'navigation2.launch.py', 'use_sim_time:=false']))
        self.rviz_PB.clicked.connect(lambda: self.run__command('rviz',['rviz2']))
        self.teleop_PB.clicked.connect(lambda: self.run__command('teleop',['ros2','run', 'turtlebot3_teleop', 'teleop_keyboard']))
        self.bringup_PB.clicked.connect(self.bringup_ros)

        # Launch Control 박스 속 kill_proc_PB 시그널 -> 5. stop_processos() 슬롯 연결
        self.stopall_PB.clicked.connect(self.stop_all_processes)


        # Velocity Control 박스 속 forward,stop,right,left,backward 시그널 -> 5. send_velocity() 슬롯 연결
        self.forward_PB.clicked.connect(lambda: self.send_velocity(self.linear_spinBox.value(), 0.0))   # forward_PB 클릭 -> send_velocity(입력값, 0.0)
        self.backward_PB.clicked.connect(lambda: self.send_velocity(-self.linear_spinBox.value(), 0.0)) # backward_PB 클릭 -> send_velocity(-입력값, 0.0)
        self.left_PB.clicked.connect(lambda: self.send_velocity(0.0, self.angular_spinBox.value()))     # left_PB 클릭 -> send_velocity(0.0,입력값)
        self.right_PB.clicked.connect(lambda: self.send_velocity(0.0, -self.angular_spinBox.value()))   # right_PB 클릭 -> send_velocity(0.0, -입력값)
        self.stop_PB.clicked.connect(lambda: self.send_velocity(0.0, 0.0))

        # Nav2 Goal 박스 속 시그널 -> 슬롯 연결
        self.load_preset_PB.clicked.connect(self.load_preset_goal)         # load_preset_PB 클릭 -> 7. load_preset_goal 실행
        self.reset_odom_view_PB.clicked.connect(self.reset_odom_display)   # reset_odom_view_PB 클릭 -> 9. reset_odom_display 실행
        
        #self.initial_pose_PB.clicked.connect(self.set_initial_pose)        # initial_pose_PB 클릭 -> 10. set_initial_pose 실행
        #self.send_goal_PB.clicked.connect(self.send_nav_goal)              # send_goal_PB 클릭 -> 11. send_nav_goal 실행
        #self.cancel_goal_PB.clicked.connect(self.cancel_nav_goal)          # cancel_goal_PB 클릭 -> 12. cancel_nav_goal 실행


    # [슬롯 함수]

    # 1. connect 시그널의 슬롯 ; 환경변수 등록,rclpy 초기화 , turtlebot3_gui_node 생성 + ros_timer 시작
    def connect_ros(self):

        os.environ['ROS_DOMAIN_ID'] = '40'

        self.robot_state_lineEdit.setText('Connected to ROS')

        if not rclpy.ok():
            rclpy.init(args=None)

        print('ROS 2 connected')


    # 2. disconnect 시그널의 슬롯 ; node 퇴근 , ros_timer 멈춤
    def disconnect_ros(self):
        if self.node:                 
            self.destroy_node()  
            self.node = None          

        self.ros_timer.stop()         
        self.ros_state_lineEdit.setText('Disconnected')                  
        self.log('ROS 2 disconnected')

    # 3. Launch Control 박스 슬롯 ; 외부 명령어를 실행하고, 실행된 프로세스를 관리리스트(processes)에 저장
    def run_command(self, name, cmd):
        env = os.environ.copy()                                            
        env['TURTLEBOT3_MODEL'] = env.get('TURTLEBOT3_MODEL', 'burger')    

        try:
            proc = subprocess.Popen(                                       
                cmd,                                                       
                env=env,                                                   
                preexec_fn=os.setsid                                       
            )                                                              

            self.processes.append((name, proc))                            
            self.log(f'{name} launched: {" ".join(cmd)}')

        except FileNotFoundError:                        
            self.log(f'Command not found: {cmd[0]}')

        except Exception as e:
            self.log(f'Launch failed: {e}')
            
    # 4. stop_all_PB 시그널의 플롯 ; 프로세스 종료
    def stop_all_processes(self):
        self.log_text.append("Stopping bringup...")
        self.run_ssh('~/tb3_scripts/stop_bringup.sh')

        for name, proc in self.processes:
            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                print(f'{name} stopped')
                self.log_text.append(f'{name} stopped')

        self.processes.clear()
        self.log_text.append("All processes stopped.")                                       

    # 5. 전진후진좌우회전정지 버튼의 슬롯
    def send_velocity(self, linear, angular):
        if not self.node:                    
            self.log('Connect ROS 2 first')     
            return

        self.node.publish_cmd(linear, angular)
        self.cmd_lineEdit.setText(f'lin={linear:.2f}, ang={angular:.2f}')  
        self.log(f'cmd_vel: linear={linear:.2f}, angular={angular:.2f}')
    
    # 6. Brindup 버튼의 슬롯 ; ssh로 bringup 스크립트 실행
    def run_ssh(self,command):
        self.process = QProcess(self)

        ssh_command=[ROBOT, command]

        self.process.readyReadStandardOutput.connect(self.read_stdout)
        self.process.readyReadStandardError.connect(self.read_stderr)

        self.process.start('ssh', ssh_command)

    def read_stdout(self):
        data = self.process.readAllStandardOutput().data().decode()
        data = data.strip()

        if data:
            self.log_text.append(data)

        if "STARTED" in data or "ALREADY_RUNNING" in data or "RUNNING" in data:
            self.robot_state_lineEdit.setText("Status: RUNNING")

        elif "STOPPED" in data or "NOT_RUNNING" in data:
            self.robot_state_lineEdit.setText("Status: STOPPED")

        elif "FAILED" in data:
            self.robot_state_lineEdit.setText("Status: FAILED")

    def read_stderr(self):
        data = self.process.readAllStandardError().data().decode()
        data = data.strip()

        if data:
            self.log_text.append(data)

    def bringup_ros(self):
        self.log_text.append("Starting bringup...")
        self.run_ssh('~/tb3_scripts/start_bringup.sh')


    def bringup_stop(self):
        self.log_text.append("Stopping bringup...")
        self.run_ssh('~/tb3_scripts/stop_bringup.sh')




















    # 6. ros_timer 타이머 울릴 때마다의 슬롯
    def spin_ros_once(self):
        if self.node:                                                      
            rclpy.spin_once(self.node, timeout_sec=0.0)                    

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

    # 8. ui_timer 타이머 울릴때마다의 슬롯
    def refresh_robot_status(self):
        if not self.node:                       
            return                              

        odom = self.node.last_odom              

        if odom:                                                  
            p = odom.pose.pose.position                           
            yaw = quaternion_to_yaw(odom.pose.pose.orientation)   

            self.odom_x_lcd.display(f'{p.x:.2f}')                 
            self.odom_y_lcd.display(f'{p.y:.2f}')                 
            self.odom_yaw_lcd.display(f'{yaw:.2f}')               

        if self.node.last_scan_min is not None:
            self.scan_lineEdit.setText(f'{self.node.last_scan_min:.2f}')    

        '''[추가] 배터리 정보 UI 및 상태창 반영'''
        if self.node.last_battery_p is not None and self.node.last_battery_v is not None:
            # 1. 만약 UI 파일에 배터리를 표시할 LineEdit 위젯 등이 있다면 아래 주석을 풀어서 사용하세요.
            # self.battery_lineEdit.setText(f'{self.node.last_battery_p * 100.0:.1f}% ({self.node.last_battery_v:.2f}V)')
            
            # 2. 하단 리스트 위젯에 주기적으로 배터리 로그 남기기 (원치 않을 시 주석처리 가능)
            self.log(f'[Battery] {self.node.last_battery_p * 100.0:.1f}%, Voltage: {self.node.last_battery_v:.2f}V')
    
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

    # 13. 
    def closeEvent(self, event):
        if self.node:
            self.send_velocity(0.0, 0.0)

        self.stop_all_processes()
        self.disconnect_ros()

        if rclpy.ok():
            rclpy.shutdown()

        event.accept()

def main():
    app = QApplication(sys.argv)

    ros_node = TurtleBot3GuiNode()

    window = TurtleBot3GUI(ros_node)
    window.show()

    exit_code = app.exec_()

    ros_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()