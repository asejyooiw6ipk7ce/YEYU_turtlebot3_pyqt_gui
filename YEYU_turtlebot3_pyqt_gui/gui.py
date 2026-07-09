import os
import signal
import subprocess
import rclpy
from PyQt5.QtCore import QProcess
from pathlib import Path
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QFileDialog
from qt_signals import RosSignals

# .ui 파일을 설치 위치(share 폴더)에서 찾기 위해 사용합니다.
# ros2 run으로 실행하면 이 방법으로 찾고, 실패하면(예: colcon build 전) 아래에서
# 소스 폴더의 resource/ 안에 있는 파일을 대신 사용합니다.
try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

ROBOT_USER = "yeyu"
ROBOT_IP = "192.168.230.100"

ROBOT = f"{ROBOT_USER}@{ROBOT_IP}"


class TurtleBot3GUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()

        self.signals = RosSignals()
        self.node = ros_node

        # turtlebot3_burger_gui.ui 파일 띄우기
        ui_path = Path(__file__).parent.parent / "resource" / "turtlebot3_pyqt_gui2.ui"
        uic.loadUi(str(ui_path), self)

        self.processes = []        # 실행중인 프로세스 보관할 리스트

        self.connect_signals()     # 사용자정의함수 ; 시그널->슬롯 연결

        # self.signals 신호 시그널 -> 플롯 연결
        self.node.signals.yaml_loaded.connect(self.update_comboboxes)          # yaml_loaded 신호 -> update_comboboxes() 실행
        self.node.signals.log_triggered.connect(self.log)                      # log_triggered 신호 -> log() 실행
        self.node.signals.odom_received.connect(self.update_odom_ui)           # odom_received 신호 -> update_odom_ui() 실행
        self.node.signals.scan_received.connect(self.update_scan_ui)           # scan_received 신호 -> update_scan_ui() 실행
        self.node.signals.battery_received.connect(self.update_battery_ui)     # battery_received 신호 -> update_battery_ui() 실행

    # 시그널->슬롯 연결
    def connect_signals(self):

        # ROS2 Launch Buttons
        self.connect_PB.clicked.connect(self.connect_ros)
        # self.disconnect_PB.clicked.connect(self.disconnect_ros)
        # self.exit_PB.clicked.connect(self.closeEvent)
        self.bringup_PB.clicked.connect(self.bringup_ros)
        self.nav2_PB.clicked.connect(lambda: self.run_command('nav2', ['ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py', 'use_sim_time:=false']))
        self.rviz_PB.clicked.connect(lambda: self.run_command('rviz', ['rviz2']))
        self.teleop_PB.clicked.connect(lambda: self.run_command('teleop', ['ros2', 'run', 'turtlebot3_teleop', 'teleop_keyboard']))
        self.stopall_PB.clicked.connect(self.stop_all_processes)

        # cmd_vel 박스 속 forward,stop,right,left,backward 시그널 -> send_velocity() 슬롯 연결
        self.forward_PB.clicked.connect(lambda: self.send_velocity(self.linear_spinBox.value(), 0.0))
        self.backward_PB.clicked.connect(lambda: self.send_velocity(-self.linear_spinBox.value(), 0.0))
        self.left_PB.clicked.connect(lambda: self.send_velocity(0.0, self.angular_spinBox.value()))
        self.right_PB.clicked.connect(lambda: self.send_velocity(0.0, -self.angular_spinBox.value()))
        self.stop_PB.clicked.connect(lambda: self.send_velocity(0.0, 0.0))

        # Waypoint
        self.load_preset_PB.clicked.connect(self.load_preset_goal)
        # self.send_goal_PB.clicked.connect(self.send_nav_goal)
        # self.cancel_goal_PB.clicked.connect(self.cancel_nav_goal)
        # 초기 위치 지정 버튼이 UI에 있다면 아래처럼 연결
        # self.set_initial_pose_PB.clicked.connect(self.set_initial_pose)
        # self.reset_odom_PB.clicked.connect(self.reset_odom_display)

        # Trajectory
        # clicked 시그널은 bool을 넘겨주기 때문에 traj_name 문자열을 넘기려면 lambda로 감싸야 함
        self.trajectory_button.clicked.connect(lambda: self.go_to_trajectory(self.trajectory_combo.currentText()))
        self.trajectory_combo.currentTextChanged.connect(lambda _: self.show_trajectory_info())

        # gTTS
        ''' self.speak_PB.clicked.connect()'''

    # [슬롯 함수]

    # connect 시그널의 슬롯 ; ROS_DOMAIN_ID 등록
    def connect_ros(self):
        os.environ['ROS_DOMAIN_ID'] = '40'
        self.robot_state_lineEdit.setText('Connected to ROS')
        print('ROS 2 connected')

    # disconnect 시그널의 슬롯 ; node 정리
    def disconnect_ros(self):
        if self.node:
            self.node.destroy_node()
            self.node = None

        self.robot_state_lineEdit.setText('Disconnected')
        self.log('ROS 2 disconnected')

    # Launch Control 박스 슬롯 ; 외부 명령어를 실행하고, 실행된 프로세스를 관리리스트(processes)에 저장
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

    # stop_all_PB 시그널의 슬롯 ; 프로세스 종료
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

    # 전진후진좌우회전정지 버튼의 슬롯
    def send_velocity(self, linear, angular):
        if not self.node:
            self.log('Connect ROS 2 first')
            return

        self.node.publish_cmd(linear, angular)
        self.current_cmd_vel_lineEdit.setText(f'lin={linear:.2f}, ang={angular:.2f}')
        self.log(f'cmd_vel: linear={linear:.2f}, angular={angular:.2f}')

    # bringup 버튼의 슬롯 ; ssh로 bringup 스크립트 실행
    def run_ssh(self, command):
        self.process = QProcess(self)
        ssh_command = [ROBOT, command]

        self.process.readyReadStandardOutput.connect(self.read_stdout)
        self.process.readyReadStandardError.connect(self.read_stderr)

        self.process.start('ssh', ssh_command)

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
        self.run_ssh('~/tb3_scripts/start_bringup.sh')

    def bringup_stop(self):
        self.log("Stopping bringup...")
        self.run_ssh('~/tb3_scripts/stop_bringup.sh')

    # load_preset_PB 시그널의 슬롯
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

    # reset_odom_view_PB 시그널의 슬롯 (버튼이 있다면 connect_signals에서 연결)
    def reset_odom_display(self):
        self.odom_x_lcd.display('0.00')
        self.odom_y_lcd.display('0.00')
        self.odom_yaw_lcd.display('0.00')
        self.log('Odometry display reset. Robot odom frame is not reset.')

    # initial_pose_PB 시그널의 슬롯 (버튼이 있다면 connect_signals에서 연결)
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

    # yaml_loaded 신호의 슬롯
    def update_comboboxes(self, traj_names):
        self.trajectory_combo.clear()
        self.trajectory_combo.addItems(traj_names)
        self.show_trajectory_info()

    # 현재 선택된 trajectory의 waypoint 순서를 화면에 표시
    def show_trajectory_info(self):
        traj_name = self.trajectory_combo.currentText()

        if traj_name in self.node.trajectories:
            wp_names = self.node.trajectories[traj_name]
            text = ' -> '.join(wp_names)
            self.trajectory_label.setText(text)

    def log(self, text):
        self.log_text.append(text)

    def update_odom_ui(self, x, y, yaw):
        self.odom_x_lcd.display(f'{x:.2f}')
        self.odom_y_lcd.display(f'{y:.2f}')
        self.odom_yaw_lcd.display(f'{yaw:.2f}')

    def update_scan_ui(self, min_scan):
        self.scan_lineEdit.setText(f'{min_scan:.2f}' if min_scan is not None else '--')

    def update_battery_ui(self, percentage, voltage):
        try:
            self.battery_lineEdit.setText(f'{percentage:.1f}% ({voltage:.2f}V)')
        except AttributeError:
            pass

    # trajectory_button 클릭 시그널의 슬롯
    # 원본은 여기서 직접 FollowWaypoints 액션을 만들었는데(FollowWaypoints import도 안 되어 있고
    # self.make_pose/self.waypoints/self.follow_client가 GUI 자신에는 없어서 무조건 에러가 났음)
    # go_to_waypoint와 동일하게 self.node에 위임하는 방식으로 수정
    def go_to_trajectory(self, traj_name):
        if not self.node:
            self.log('Connect ROS 2 first')
            return

        ok, text = self.node.go_to_trajectory(traj_name)
        self.log(text)

    def closeEvent(self, event):
        if self.node:
            self.send_velocity(0.0, 0.0)

        self.stop_all_processes()
        self.disconnect_ros()

        if rclpy.ok():
            rclpy.shutdown()

        event.accept()