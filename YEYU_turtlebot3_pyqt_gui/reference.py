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
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer                                           # PYQT5 루프를 굴리면서 ROS2 루프 굴리기 위해 QTimer 데려옴
from geometry_msgs.msg import PoseWithCovarianceStamped                   # ROS2 내비게이션 시스템에서 로봇 위치와 방향 전달할 때 사용되는 msg규격(초기위치 지정할 때 반드시 이 형식으로 보내야 로봇이 이해)
from rclpy.action import ActionClient                                     # /navigate_to_pose
from nav2_msgs.action import NavigateToPose                               # /navigate_to_pose
from geometry_msgs.msg import PoseStamped                                 # /navigate_to_pose
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


# 쿼터니언 -> 각도 변환 (ros2에서 가져오는 yaw값)
def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

# 각도 -> 쿼터니언 
def yaw_to_quaternion(yaw):
    half = yaw * 0.5
    qx = 0.0
    qy = 0.0
    qz = math.sin(half)
    qw = math.cos(half)
    return qx, qy, qz, qw

class TurtleBot3GuiNode(Node):
    def __init__(self, namespace=''):                                     # 클래스 생성자. namespace값 인자로 받음(로봇 여러대면 구분하기 위해)
        super().__init__('turtlebot3_burger_gui')                         # 부모 생성자 호출 + 노드 이름 지정

        # namespace 입력값에서 뽑아오기
        namespace = namespace.strip()                                     # '입력값 ' -> '입력값'
        if namespace.lower() in ['', 'empty', 'none', '/']:               # 사용자가 "empty", "none", "/" 등을 넣으면 namespace 없음으로 처리
            namespace = ''
        namespace = namespace.strip('/')                                  # '/입력값/' -> '입력값'

        '''[/cmd_vel 발행]'''
        # cmd_topic 정의
        if namespace:
            cmd_topic = f'/{namespace}/cmd_vel'                           # ex. /tb3_01/cmd_vel
        else:
            cmd_topic = '/cmd_vel'                                        

        # cmd_topic 발행자 생성
        self.cmd_pub = self.create_publisher(
            Twist,
            cmd_topic,
            10
        )
    
        '''[/odom 수신]'''
        # odom_topic 정의
        if namespace:
            odom_topic = f'/{namespace}/odom'
        else:
            odom_topic = '/odom'

        # odom_topic 수신자 생성
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        self.last_odom = None

        '''[/initialpose 발행]'''
        # initpos_topic 정의
        if namespace:
            initpos_topic = f'/{namespace}/initialpose'
        else:
            initpos_topic = '/initialpose'

        # initpos_topic 발행자 생성
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            initpos_topic,
            10
        )

        '''[/navigate_to_pose 액션 클라이언트]'''
        # navpose_topic 정의
        if namespace:
            navpos_topic = f'/{namespace}/navigate_to_pose'
        else:
            navpos_topic = '/navigate_to_pose'

        # navpose_topic 액션클라이언트 생성
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            navpos_topic
        )

        self.goal_handle = None

        '''[/scan 수신]'''
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # scan_topic 정의
        if namespace:
            scan_topic = f'/{namespace}/scan'
        else:
            scan_topic = '/scan'

        # scan_topic 수신자 정의
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            qos_profile
        )

        self.last_scan_min = None


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
    
    # navpose_topic 액션클라이언트 ; goal 전송 함수
    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)

        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            return False, 'Nav2 action server is not available'

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

        return True, f'Goal sent: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
    
    # navpose_topic 액션클라이언트 ; goal 응답 피드백 함수
    def _goal_response_callback(self, future):
        self.goal_handle = future.result()

        if self.goal_handle and self.goal_handle.accepted:
            self.get_logger().info('Goal accepted')
        else:
            self.get_logger().warn('Goal rejected')

    # navpose_topic 액션클라이언트 ; goal 취소
    def cancel_goal(self):
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
            return True

        return False
    
    # scan_topic 콜백함수
    def scan_callback(self, msg):
        values = [
            v for v in msg.ranges
            if math.isfinite(v) and v > 0.0
        ]

        self.last_scan_min = min(values) if values else None

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # turtlebot3_burger_gui.ui 파일 띄우기
        ui_path = Path(__file__).with_name('turtlebot3_burger_gui.ui')
        uic.loadUi(str(ui_path), self)

        self.processes = []        # 실행중인 프로세스 보관할 리스트

        self.node = None           # ROS2 노드로 쓸 node 생성

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
        
        ''' 현재 ssh y나 source install/setup.bash를 쓸 수 없어서 못 씀'''
        # Launch Control 박스 속 5가지 버튼 시그널 -> 4. run_command() 슬롯 연결 
        self.bringup_PB.clicked.connect(lambda: self.run_command('bringup',['ros2', 'launch', 'turtlebot3_bringup', 'robot.launch.py']))
                                                                # 프로세스명(별명) 
            # 왜 [' ',' ']로 쓰나 ? python에서 외부프로세스를 실행할 때 쓰는 subprocess모듈의 사용법이 단어 단위로 쪼개진 리스트 형식
        self.slam_PB.clicked.connect(lambda: self.run_command('slam',['ros2', 'launch', 'turtlebot3_cartographer', 'cartographer.launch.py', 'use_sim_time:=false']))
        self.nav2_PB.clicked.connect(lambda: self.run_command('nav2',['ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py', 'use_sim_time:=false']))
        self.rviz_PB.clicked.connect(lambda: self.run_command('rviz2',['rviz2']))
        self.save_map_PB.clicked.connect(lambda: self.run_command('save_map',['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', 'tb3_map']))
        
        # Launch Control 박스 속 kill_proc_PB 시그널 -> 5. stop_processos() 슬롯 연결
        self.kill_proc_PB.clicked.connect(self.stop_processes)

        # Velocity Control 박스 속 forward,stop,right,left,backward 시그널 -> 5. send_velocity() 슬롯 연결
        self.forward_PB.clicked.connect(lambda: self.send_velocity(self.linear_spinBox.value(), 0.0))   # forward_PB 클릭 -> send_velocity(입력값, 0.0)
        self.backward_PB.clicked.connect(lambda: self.send_velocity(-self.linear_spinBox.value(), 0.0)) # backward_PB 클릭 -> send_velocity(-입력값, 0.0)
        self.left_PB.clicked.connect(lambda: self.send_velocity(0.0, self.angular_spinBox.value()))     # left_PB 클릭 -> send_velocity(0.0,입력값)
        self.right_PB.clicked.connect(lambda: self.send_velocity(0.0, -self.angular_spinBox.value()))   # right_PB 클릭 -> send_velocity(0.0, -입력값)
        self.stop_PB.clicked.connect(lambda: self.send_velocity(0.0, 0.0))

        # Nav2 Goal 박스 속 시그널 -> 슬롯 연결
        self.load_preset_PB.clicked.connect(self.load_preset_goal)         # load_preset_PB 클릭 -> 7. load_preset_goal 실행
        self.reset_odom_view_PB.clicked.connect(self.reset_odom_display)   # reset_odom_view_PB 클릭 -> 9. reset_odom_display 실행
        self.initial_pose_PB.clicked.connect(self.set_initial_pose)        # initial_pose_PB 클릭 -> 10. set_initial_pose 실행
        self.send_goal_PB.clicked.connect(self.send_nav_goal)              # send_goal_PB 클릭 -> 11. send_nav_goal 실행
        self.cancel_goal_PB.clicked.connect(self.cancel_nav_goal)          # cancel_goal_PB 클릭 -> 12. cancel_nav_goal 실행


    # [슬롯 함수]

    # 1. connect 시그널의 슬롯 ; 환경변수 등록,rclpy 초기화 , turtlebot3_gui_node 생성 + ros_timer 시작
    def connect_ros(self):
        if self.node:
            self.log('ROS 2 is already connected')
            return

        # gui의 입력값을 변수에 저장
        domain_id = self.domain_lineEdit.text().strip()
        namespace = self.namespace_lineEdit.text().strip()

        # domain_id의 입력값 -> os 환경변수 등록
        os.environ['ROS_DOMAIN_ID'] = domain_id if domain_id else '40'

        # connect 여러번 눌러서 시스템이 중복 초기화되어 에러나는 것을 방지
        if not rclpy.ok():                                              # ros2 통신 시스템 켜져있니? 꺼져있으면
            rclpy.init(args=None)                                       # ros2 엔진 키기
        
        # TurtleBot3GuiNode 객체 생성 (namespace입력값을 인자로 받는)
        self.node = TurtleBot3GuiNode(self.namespace_lineEdit.text())  

        # 0.02초마다 타이머 울리기
        self.ros_timer.start(20)                               

        self.ros_status_lineEdit.setText('Connected')           # 연결 누르면, ros_status_lineEdit에 Connected 뜸

        # print(f'ROS_DOMAIN_ID = {os.environ["ROS_DOMAIN_ID"]}')
        # print(f'Namespace = {namespace}')
        self.log('ROS 2 connected')


    # 2. disconnect 시그널의 슬롯 ; node 퇴근 , ros_timer 멈춤
    def disconnect_ros(self):

        if self.node:                 # 살아있는 노드 있으면
            self.node.destroy_node()  # 노드 파괴
            self.node = None          # 노드 초기화

        self.ros_timer.stop()         # 타이머 중단(ros2 연결 끊었으니)

        self.ros_status_lineEdit.setText('Disconnected')                  # 연결 해제 누르면, ros_status_lineEdit에 Disconnected 뜸
        self.log('ROS 2 disconnected')

    # 3. Launch Control 박스 슬롯 ; 외부 명령어를 실행하고, 실행된 프로세스를 관리리스트(processes)에 저장
    def run_command(self, name, cmd):
        env = os.environ.copy()                                            # 새 프로세스에 세팅할 똑같은 환경변수들
        env['TURTLEBOT3_MODEL'] = env.get('TURTLEBOT3_MODEL', 'burger')    # 새 프로세스에 세팅할 TURTLEBOT3_MODEL=burger

        try:
            proc = subprocess.Popen(                                       # subprocess.Popen() : python에서 외부 프로그램을 실행, 백그라운드에서 실행 후 다음 코드로 넘어감
                cmd,                                                       # 명령어 리스트
                env=env,                                                   # 앞에 세팅한 환경변수 이 터미널에 적용
                preexec_fn=os.setsid                                       # 이 프로세스를 독립된 새 프로세스 그룹으로 묶어서 실행(이 그룹 전체를 한번에 죽이기 위함)
            )                                                              # -> 리모컨(proc) 탄생

            self.processes.append((name, proc))                            # 리모컨을 self.process에 추가(프로세스명name와 튜플 형태로 묶어서)
            self.log(f'{name} launched: {" ".join(cmd)}')

        # 명령어가 설치되어있지 않아 실행파일을 찾을 수 없을 경우
        except FileNotFoundError:                        
            self.log(f'Command not found: {cmd[0]}')

        # 그 외의 이유로 실행 실패할 경우
        except Exception as exc:
            self.log(f'Launch failed: {exc}')
            
    # 4. kill_proc_PB 시그널의 플롯 ; 프로세스 종료
    def stop_processes(self):
        for name, proc in self.processes:                                 # processes 안에 있는 것들 모두 반복
            if proc.poll() is None:                                       # proc.poll() : 리모컨의 프로그램 아직 실행 중? os에 물어보는 거
                            #Non
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)           # os.getpgid(): proc의 프로세스 id 가져옴 , os.killpg():그룹의 프로세스 전체에 신호 보냄 
                                        #프로세스id      #종료신호
                self.log(f'{name} stopped')                                  # 어떤 프로세스가 성공적으로 닫혔는지 알 수 있음

        self.processes.clear()                                            # processes 초기화(안하면 ? 꺼진 리모컨들을 바구니에 계속 두게 됨)

    # 5. 전진후진좌우회전정지 버튼의 슬롯
    def send_velocity(self, linear, angular):

        # connect 안 누르고 전진후진부터 눌러 프로그램 뻗는 거 방지하기 위함
        if not self.node:                    
            self.log('Connect ROS 2 first')     # self.node = none인데 전진후진회전 버튼을 누르면 나오는 거 
            return

        # 버튼에서의 정보를 node에게 넘기며 발행해달라 명령
        self.node.publish_cmd(linear, angular)
        
        self.cmd_lineEdit.setText(f'lin={linear:.2f}, ang={angular:.2f}')  # cmd_lineEdit에 명령한 숫자 띄우기
        self.log(f'cmd_vel: linear={linear:.2f}, angular={angular:.2f}')
    
    # 6. ros_timer 타이머 울릴 때마다의 슬롯
    def spin_ros_once(self):
        if self.node:                                                      # 노드가 생성되지 않았거나 이미 파괴했을 때 ros2 기능을 실행할려고 하면 에러가 남
            rclpy.spin_once(self.node, timeout_sec=0.0)                    # 밀린 거 없나 확인해줘
                                       #확인할게없으면 0초만에 복귀해

    # 7. load_preset_PB 시그널의 슬롯
    def load_preset_goal(self):
        # preset_goal_CB에서 몇번째 항목 선택했는지 인덱스번호 idx 저장
        idx = self.preset_goal_CB.currentIndex()                           

        # 항목에 임의로 지정한 좌표셋
        presets = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 1.57)
        ]

        # idx번째 presets 뽑아서 x,y,yaw에 저장
        x, y, yaw = presets[idx]             

        # x,y,yaw -> GUI 숫자 입력 창에 주입(setValue)
        self.goal_x_spinBox.setValue(x)
        self.goal_y_spinBox.setValue(y)
        self.goal_yaw_spinBox.setValue(yaw)

        print(f'Preset loaded: x={x}, y={y}, yaw={yaw}')

    # 8. ui_timer 타이머 울릴때마다의 슬롯
    def refresh_robot_status(self):
        if not self.node:                       # ros2노드가 연결되지 않은 상태 -> 화면을 갱신할 수 없음
            return                              # 함수 종료

        odom = self.node.last_odom              # self.node에서 받아오는 lase_odom을 odom에 저장

        if odom:                                                  # 데이터가 존재한다면
            p = odom.pose.pose.position                           # 위치데이터 중 좌표 데이터를 p에 저장
            yaw = quaternion_to_yaw(odom.pose.pose.orientation)   # 쿼터니언->도로 바꾼 부분 yaw로 변환

            self.odom_x_lcd.display(f'{p.x:.2f}')                 # X값을 GUI의 LCD창에 반영
            self.odom_y_lcd.display(f'{p.y:.2f}')                 # Y값을 GUI의 LCD창에 반영
            self.odom_yaw_lcd.display(f'{yaw:.2f}')               # Yaw값을 GUI의 LCD창에 반영

        if self.node.last_scan_min is not None:
            self.scan_lineEdit.setText(f'{self.node.last_scan_min:.2f}')    
    
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

        # 입력창에 있는 x,y,yaw 받아 ros2 publish_initial_pose함수에 전달
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

        ok, text = self.node.send_goal(
            self.goal_x_spinBox.value(),
            self.goal_y_spinBox.value(),
            self.goal_yaw_spinBox.value()
        )

        self.log(text)

    # 12. 
    def cancel_nav_goal(self):
        if not self.node:
            self.log('Connect ROS 2 first')
            return

        if self.node.cancel_goal():
            self.log('Goal cancel requested')
        else:
            self.log('No active goal handle')

    # 13. 
    def closeEvent(self, event):
        if self.node:
            self.send_velocity(0.0, 0.0)

        self.stop_processes()
        self.disconnect_ros()

        if rclpy.ok():
            rclpy.shutdown()

        event.accept()

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()