#!/usr/bin/env python3                        # OS에게 이 파일은 파이썬3로 실행되어야 하는 스크립트야

import sys
import math
import argparse
import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.utilities import remove_ros_args

from geometry_msgs.msg import PoseStamped     # make_pose() 함수 안에서 사용
from nav2_msgs.action import NavigateToPose   # /navigate_to_pose 의 액션타입
from nav2_msgs.action import FollowWaypoints  # /follow_waypoints

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QLabel,
    QComboBox,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
)


class Nav2WaypointGui(QMainWindow):
    def __init__(self, yaml_file):
        super().__init__()          # 부모클래스 생성자 호출

        self.yaml_file = yaml_file  # 이 노드 객체 생성할 때 받는 인자의 yaml_file을 self.yaml_file에 저장
                                   ''' 인자로 받는 건 __init__ 나가면 사라짐, self.를 붙이면 이 객체 내부 인스턴스 변수가 됨 '''

				# yaml파일에서 가져올 정보
        self.waypoints = {}         # 단일 목적지 저장할 딕셔너리
        self.trajectories = {}      # 경로(목적지들의 묶음) 저장할 딕셔너리

        self.node = rclpy.create_node('simple_nav2_waypoint_gui') # simple_nav2_waypoint_gui라는 노드 생성 -> self.node라 지칭

		# 단일 목적지 주행 액션클라이언트 생성 
        self.navigate_client = ActionClient(
            self.node,
            NavigateToPose,
            'navigate_to_pose'                ''' /붙이면 절대경로, 안붙이면 상대경로 '''
        )
				
		#경유점 순차 주행 액션클라이언트 생성
        self.follow_client = ActionClient(
            self.node,
            FollowWaypoints,
            'follow_waypoints'
        )

		# PyQt 창 생성
        self.setWindowTitle('TurtleBot3 Nav2 Waypoint GUI')
        self.resize(600, 420)

        self.make_gui()          # (1) GUI 화면 레이어드 만드는 함수
        self.load_yaml()         # (2) YAML 파일 읽어서 데이터 채우는 함수

        self.timer = QTimer()    # PyQt5 전용 타이머
        self.timer.timeout.connect(self.ros_spin_once)   # 타이머가 울리면 -> 1. ros_spin_once 실행
        self.timer.start(50)     # 타이머 0.05초마다 울리기 시작

		
	# (1) GUI 화면 레이어드 만드는 함수 ; PyQt에서 .ui파일 만드는 거
    def make_gui(self):
        main_widget = QWidget()                # 메인 위젯 생성
        main_layout = QVBoxLayout()            # 세로 레이아웃 생성

		# --- YAML 파일 표시 구역 ---
        yaml_group = QGroupBox('YAML 파일')    # 'YAML파일' 그룹박스 생성
        yaml_layout = QVBoxLayout()            # 세로레이아웃 생성
        yaml_layout.addWidget(QLabel(self.yaml_file))  # yaml파일명 표시된 label 생성 (in 레이아웃)
        yaml_group.setLayout(yaml_layout)      # 레이아웃 -> 그룹박스

		# --- 단일 주행 구역 ---
        waypoint_group = QGroupBox('Waypoint 단일 주행')
        waypoint_layout = QVBoxLayout()

        self.waypoint_combo = QComboBox()
        self.waypoint_button = QPushButton('선택한 Waypoint로 이동')
        self.waypoint_button.clicked.connect(self.go_to_waypoint)    # waypoint_button 클릭 -> 2. go_to_waypoint 실행

        waypoint_layout.addWidget(QLabel('Waypoint 선택'))
        waypoint_layout.addWidget(self.waypoint_combo)
        waypoint_layout.addWidget(self.waypoint_button)
        waypoint_group.setLayout(waypoint_layout)

		# --- 순차 주행 구역 ---
        trajectory_group = QGroupBox('Trajectory 순차 주행')
        trajectory_layout = QVBoxLayout()

        self.trajectory_combo = QComboBox()
        self.trajectory_label = QLabel('')
        self.trajectory_button = QPushButton('선택한 Trajectory 주행')
        self.trajectory_button.clicked.connect(self.go_to_trajectory)
        self.trajectory_combo.currentIndexChanged.connect(self.show_trajectory_info)

        trajectory_layout.addWidget(QLabel('Trajectory 선택'))
        trajectory_layout.addWidget(self.trajectory_combo)
        trajectory_layout.addWidget(self.trajectory_label)
        trajectory_layout.addWidget(self.trajectory_button)
        trajectory_group.setLayout(trajectory_layout)

		# --- 로그 표시 구역 ---
        log_group = QGroupBox('로그')
        log_layout = QVBoxLayout()

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)

        log_layout.addWidget(self.log_box)
        log_group.setLayout(log_layout)

        main_layout.addWidget(yaml_group)
        main_layout.addWidget(waypoint_group)
        main_layout.addWidget(trajectory_group)
        main_layout.addWidget(log_group)

        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

	# (2) YAML 파일 읽어서 데이터 채우는 함수
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
            self.waypoint_combo.addItem(name)  # GUI 콤보박스(waypoint_combo)에 추가

		# trajectory_list 리스트 -> self.trajectories 딕셔너리로 정
        for traj in trajectory_list:
            name = traj['name']
            wp_names = traj['waypoints']
            self.trajectories[name] = wp_names
            self.trajectory_combo.addItem(name)

        self.show_trajectory_info()          # (3) 첫번째 경로 정보 화면에 띄우는 함수

		# GUI에 로그 출력
        self.log('YAML 로드 완료')
        self.log(f'Waypoint 개수: {len(self.waypoints)}')
        self.log(f'Trajectory 개수: {len(self.trajectories)}')
    
    # 1. timer 울리는 시그널의 슬롯함수    
    def ros_spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
		
	# (3) 첫번째 경로 정보 화면에 띄우는 함수 (load_yaml() 내부에 호출)
    def show_trajectory_info(self):
        traj_name = self.trajectory_combo.currentText()   # 현재 선택된 경로 이름 가져

        if traj_name in self.trajectories:
            wp_names = self.trajectories[traj_name]
            text = ' -> '.join(wp_names)            # 예: ['point1', 'point2'] 상태를 "point1 -> point2" 형태의 문자열로
            self.trajectory_label.setText(text)     # 화면에 경로순서 표

	# go_to_waypoint() 내부에서 호출 ; self.waypoints -> x,y,z,qz,qw로 걸러낸 걸 pose(msg)=완성된 위치 메세지로 담아서 반환
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

	# 2.waypoint_button 클릭의 슬롯함수 ; 단일 목적지 액션 통신 기능
    def go_to_waypoint(self):
        waypoint_name = self.waypoint_combo.currentText()  # GUI창 waypoint_combo에서 선택한 값 -> way

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

        # navigat_client에게 비동기(async) 명령
        future = self.navigate_client.send_goal_async(goal_msg)
        future.add_done_callback(self.waypoint_goal_response) # 서버가 수락거절 응답 오면 -> waypoint_goal_response 실행

    def waypoint_goal_response(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.log('Waypoint goal이 거부되었습니다.')
            return

        self.log('Waypoint goal이 수락되었습니다.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.waypoint_result)

    def waypoint_result(self, future):
        self.log('Waypoint 이동 완료')

    def go_to_trajectory(self):
        traj_name = self.trajectory_combo.currentText()

        if traj_name == '':
            self.log('선택된 trajectory가 없습니다.')
            return

        if traj_name not in self.trajectories:
            self.log('trajectory 정보가 없습니다.')
            return

        if not self.follow_client.wait_for_server(timeout_sec=1.0):
            self.log('/follow_waypoints 액션 서버가 준비되지 않았습니다.')
            return

        waypoint_names = self.trajectories[traj_name]

        poses = []

        for name in waypoint_names:
            if name not in self.waypoints:
                self.log(f'YAML에 없는 waypoint입니다: {name}')
                return

            pose = self.make_pose(name)
            poses.append(pose)

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.log(f'Trajectory 주행 요청: {traj_name}')
        self.log(f'포함된 waypoint 개수: {len(poses)}')

        future = self.follow_client.send_goal_async(goal_msg)
        future.add_done_callback(self.trajectory_goal_response)

    def trajectory_goal_response(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.log('Trajectory goal이 거부되었습니다.')
            return

        self.log('Trajectory goal이 수락되었습니다.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.trajectory_result)

    def trajectory_result(self, future):
        self.log('Trajectory 주행 완료')



    def log(self, msg):
        self.log_box.append(msg)
        self.node.get_logger().info(msg)

    def closeEvent(self, event):
        self.timer.stop()
        self.node.destroy_node()
        event.accept()


def main():
    rclpy.init(args=sys.argv)

    ros_removed_args = remove_ros_args(sys.argv)

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--yaml',
        required=True,
        help='waypoint yaml file path'
    )

    args = parser.parse_args(ros_removed_args[1:])

    app = QApplication(ros_removed_args)

    window = Nav2WaypointGui(args.yaml)
    window.show()

    app.exec_()

    rclpy.shutdown()


if __name__ == '__main__':
    main()