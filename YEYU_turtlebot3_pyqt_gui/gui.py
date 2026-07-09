#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os        
import signal 
import subprocess
import rclpy   
from pathlib import Path
from PyQt5 import uic
from PyQt5.QtCore import QProcess
from PyQt5.QtWidgets import QWidget, QFileDialog
from qt_signals import RosSignals

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
        # 전압 매핑용 UI 컴포넌트가 존재한다면 아래 주석을 해제하여 사용하세요.
        # self.battery_volt_lcd.display(f'{voltage:.2f}')