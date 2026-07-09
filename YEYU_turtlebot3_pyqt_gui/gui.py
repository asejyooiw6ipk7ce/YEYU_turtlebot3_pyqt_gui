import os        
import signal 
import subprocess
import rclpy   
from pathlib import Path
from PyQt5 import uic
from PyQt5.QtCore import QProcess
from PyQt5.QtWidgets import QWidget
from qt_signals import RosSignals

ROBOT_USER = "yeyu"
ROBOT_IP = "192.168.230.100"

ROBOT = f"{ROBOT_USER}@{ROBOT_IP}"

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

        # QTimer 대신 멀티스레드로 변경
        # self.ros_timer = QTimer(self)                                           # QTimer 객체 생성
        # self.ros_timer.timeout.connect(self.spin_ros_once)                      # ros2_timer에서 알림이 울리면 -> 6. spin_ros_once 실행
        # self.ui_timer = QTimer(self)
        # self.ui_timer.timeout.connect(self.refresh_robot_status)                # ui_timer에서 알림이 울리면 -> 8. refresh_robot_status 실행
        # self.ui_timer.start(200)                                                # 200ms초마다 울리기

        # self.signals 신호 시그널 -> 플롯 연결
        self.node.signals.yaml_loaded.connect(self.update_comboboxes)           # yaml_loaded 신호 -> 13. update_comboboxes() 실행
        self.node.signals.log_triggered.connect(self.log)                       # log_triggered 신호 -> 14. log() 실행
        self.node.signals.odom_received.connect(self.update_odom_ui)            # odom_received 신호 -> 15. update_odom_ui() 실행 
        self.node.signals.scan_received.connect(self.update_scan_ui)            # scan_received 신호 -> 16. update_scan_ui() 실행
        self.node.signals.battery_received.connect(self.update_battery_ui)      # battery_received 신호 -> 17. update_battery_ui() 실행

    # 시그널->슬롯 연결
    def connect_signals(self):
        self.connect_PB.clicked.connect(self.connect_ros)                        # connect_PB 클릭 -> 1. connect_ros 실행
        #self.disconnect_PB.clicked.connect(self.disconnect_ros)                  # disconnect_PB 클릭 -> 2. disconnect_ros 실행
        #self.exit_PB.clicked.connect(self.closeEvent)                                 # exit_PB 클릭 -> 100. closeEvent 실행
        
        # Launch Control 박스 속 5가지 버튼 시그널 -> 4. run_command() 슬롯 연결 
        self.connect_PB.clicked.connect (self.connect_ros)
        self.nav2_PB.clicked.connect(lambda: self.run_command('nav2',['ros2','launch', 'turtlebot3_navigation2', 'navigation2.launch.py', 'use_sim_time:=false']))
        self.rviz_PB.clicked.connect(lambda: self.run_command('rviz',['rviz2']))
        self.teleop_PB.clicked.connect(lambda: self.run_command('teleop',['ros2','run', 'turtlebot3_teleop', 'teleop_keyboard']))
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
        #self.load_preset_PB.clicked.connect(self.load_preset_goal)         # load_preset_PB 클릭 -> 7. load_preset_goal 실행
        #self.reset_odom_view_PB.clicked.connect(self.reset_odom_display)   # reset_odom_view_PB 클릭 -> 9. reset_odom_display 실행
        
        #self.initial_pose_PB.clicked.connect(self.set_initial_pose)        # initial_pose_PB 클릭 -> 10. set_initial_pose 실행
        #self.send_goal_PB.clicked.connect(self.send_nav_goal)              # send_goal_PB 클릭 -> 11. send_nav_goal 실행
        #self.cancel_goal_PB.clicked.connect(self.cancel_nav_goal)          # cancel_goal_PB 클릭 -> 12. cancel_nav_goal 실행


    # [슬롯 함수]

    # 1. connect 시그널의 슬롯 ; 환경변수 등록,rclpy 초기화 , turtlebot3_gui_node 생성 + ros_timer 시작
    def connect_ros(self):

        os.environ['ROS_DOMAIN_ID'] = '40'

        self.robot_state_lineEdit.setText('Connected to ROS')

        print('ROS 2 connected')

        # QTimer -> 멀티스레드 방식으로 바꾸면서 주석처리
        #self.ros_timer.start(20)  # 100ms마다 울리기


    # 2. disconnect 시그널의 슬롯 ; node 퇴근 , ros_timer 멈춤
    def disconnect_ros(self):
        if self.node:                 
            self.node.destroy_node()  
            self.node = None          

        #self.ros_timer.stop()  -> QTimer 대신 멀티스레드        
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
        self.log("Stopping bringup...")
        self.run_ssh('~/tb3_scripts/stop_bringup.sh')

        for name, proc in self.processes:
            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                print(f'{name} stopped')
                self.log(f'{name} stopped')


        self.processes.clear()
        self.log("All processes stopped.")                                       

    # 5. 전진후진좌우회전정지 버튼의 슬롯
    def send_velocity(self, linear, angular):
        if not self.node:                    
            self.log('Connect ROS 2 first')     
            return

        self.node.publish_cmd(linear, angular)
        self.current_cmd_vel_lineEdit.setText(f'lin={linear:.2f}, ang={angular:.2f}')  
        self.log(f'cmd_vel: linear={linear:.2f}, angular={angular:.2f}')
    
    # 6. Brindup 버튼의 슬롯 ; ssh로 bringup 스크립트 실행
    def run_ssh(self,command):
        self.process = QProcess(self)                                             
        # 파이썬 내부의 숨겨진 가상 터미널 창 생성

        ssh_command=[ROBOT, command]   # ROBOT = f"{ROBOT_USER}@{ROBOT_IP}"

        self.process.readyReadStandardOutput.connect(self.read_stdout)
        self.process.readyReadStandardError.connect(self.read_stderr)

        self.process.start('ssh', ssh_command)       # 백그라운드에서 조용히 ssh 명령 실행

    def read_stdout(self):
        data = self.process.readAllStandardOutput().data().decode()
        data = data.strip()

        if data:
            self.log(data)

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
            self.log(data)

    def bringup_ros(self):
        self.log("Starting bringup...")
        self.run_ssh('~/tb3_scripts/start_bringup.sh')   # 로봇에 있는 ~/tb3_scripts/start_bringup.sh를 통해 bringup을 하게 됨


    def bringup_stop(self):
        self.log("Stopping bringup...")
        self.run_ssh('~/tb3_scripts/stop_bringup.sh')



















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
        self.log_text.append(text)  # UI 파일 이름인 log_text로 맞추고 append 사용

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
