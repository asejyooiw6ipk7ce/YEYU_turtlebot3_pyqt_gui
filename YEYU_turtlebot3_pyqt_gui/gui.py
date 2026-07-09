#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 예전에는 os, signal, subprocess, rclpy, QProcess, RosSignals 를 import 했지만
# 실제 코드 어디에서도 쓰이지 않는 "죽은 import"였어서 정리했습니다.
from pathlib import Path
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QFileDialog

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

        self.node = ros_node
        # ros_node 내부에 이미 정의된 signals 인스턴스를 공유하여 소통 채널 단일화
        self.signals = self.node.signals 

        # UI 파일 로드
        # 1순위: colcon build로 설치된 위치(share 폴더)에서 찾기
        # 2순위: 설치 전이거나 못 찾으면, 소스 코드 폴더의 resource/ 안에서 찾기
        ui_path = None
        if get_package_share_directory is not None:
            try:
                share_dir = Path(get_package_share_directory('YEYU_turtlebot3_pyqt_gui'))
                ui_path = share_dir / "turtlebot3_pyqt_gui2.ui"
            except Exception:
                ui_path = None

        if ui_path is None or not ui_path.exists():
            ui_path = Path(__file__).parent.parent / "resource" / "turtlebot3_pyqt_gui2.ui"

        uic.loadUi(str(ui_path), self)

        self.processes = []  # 실행 중인 외부 프로세스 보관용
        self.trajectories = self.node.trajectories # 노드의 경로 데이터 참조

        self.connect_signals()  # 시그널 - 슬롯 연결 호출
        
        # 콤보박스 변경 이벤트 연결
        self.trajectory_combo.currentTextChanged.connect(self.show_trajectory_info)

    def connect_signals(self):
        """ROS 노드 스레드로부터 발생하는 Qt 시그널들을 GUI 슬롯 함수에 매핑"""
        self.signals.yaml_loaded.connect(self.update_comboboxes)
        self.signals.log_triggered.connect(self.log)
        self.signals.odom_received.connect(self.update_odom_ui)
        self.signals.scan_received.connect(self.update_scan_ui)
        self.signals.battery_received.connect(self.update_battery_ui)
        self.trajectory_button.clicked.connect(self.start_trajectory_navigation)
        self.yaml_load_PB.clicked.connect(self.open_yaml_file)
        self.waypoint_go_PB.clicked.connect(self.start_waypoint_navigation)

        # 비상 정지 버튼: 화면에는 있었지만 눌러도 아무 동작도 안 하던 상태였어서 연결했습니다.
        self.emergency_stop_PB.clicked.connect(self.emergency_stop)

    def open_yaml_file(self):
        """YAML 파일을 선택해 경유점/경로 데이터를 불러옵니다."""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "YAML 파일 선택", "", "YAML Files (*.yaml *.yml)"
        )
        if not file_path:
            return

        self.yaml_path_lineEdit.setText(file_path)
        self.node.load_yaml(file_path)

    def start_waypoint_navigation(self):
        """선택된 단일 경유점으로 이동 시작"""
        wp_name = self.waypoint_combo.currentText()
        if wp_name:
            self.node.go_to_waypoint(wp_name)
        else:
            self.log("선택된 Waypoint가 없습니다.")

    def start_trajectory_navigation(self):
        """순차 목적지 주행 시작"""
        traj_name = self.trajectory_combo.currentText()
        if traj_name:
            self.node.go_to_trajectory(traj_name) # ros_node의 신규 함수 호출
        else:
            self.log("선택된 Trajectory가 없습니다.")

    def update_comboboxes(self, wp_names, traj_names):
        """YAML이 로드되면 UI의 콤보박스 목록을 갱신합니다."""
        self.waypoint_combo.clear()
        self.waypoint_combo.addItems(wp_names)

        self.trajectory_combo.clear()
        self.trajectory_combo.addItems(traj_names)
        
        # 데이터 동기화
        self.trajectories = self.node.trajectories
        self.show_trajectory_info()

    def show_trajectory_info(self):
        """현재 선택된 경로의 순서 정보를 화면(Label)에 표시합니다."""
        traj_name = self.trajectory_combo.currentText()
        if traj_name in self.trajectories:
            wp_names = self.trajectories[traj_name]  # 예: ['point1', 'point2']
            text = ' -> '.join(wp_names)
            self.trajectory_label.setText(text)
        else:
            self.trajectory_label.setText("경로 정보 없음")

    def log(self, text):
        self.log_text.append(text)

    def update_odom_ui(self, x, y, yaw):
        self.odom_x_lcd.display(f'{x:.2f}')
        self.odom_y_lcd.display(f'{y:.2f}')
        self.odom_yaw_lcd.display(f'{yaw:.2f}')

    def update_scan_ui(self, min_scan):
        self.scan_lineEdit.setText(f'{min_scan:.2f} m')

    def update_battery_ui(self, percent, voltage):
        self.battery_lcd.display(f'{percent:.1f}')
        # 예전 주석은 battery_volt_lcd 라는, .ui 파일에 존재하지도 않는 위젯 이름을 쓰고 있었습니다.
        # 실제 .ui 파일에 있는 전압 표시용 위젯 이름은 voltage_lineEdit 입니다.
        self.voltage_lineEdit.setText(f'{voltage:.2f} V')

    def emergency_stop(self):
        """비상 정지 버튼을 누르면 로봇을 즉시 멈춥니다 (속도 0으로 명령 전송)."""
        self.node.publish_cmd(0.0, 0.0)
        self.log('비상 정지! 로봇을 멈췄습니다.')