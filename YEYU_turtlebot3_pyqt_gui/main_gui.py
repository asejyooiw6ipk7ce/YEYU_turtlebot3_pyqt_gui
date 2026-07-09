#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main_gui.py
TurtleBot3 PyQt GUI 미니 프로젝트 - 메인 GUI 실행 파일

실행 방법:
    ros2 run turtlebot3_pyqt_gui main_gui
    (또는 ROS2 환경을 source 한 뒤 python3 main_gui.py)

이 파일은 main_gui.ui 를 로드하여 화면을 구성하고,
- ros_worker.RosWorker : ROS2 토픽 구독/Nav2 액션 (경유점, trajectory 이동)
- tts_worker.TTSWorker : gTTS 음성 출력
- subprocess          : bringup / nav2 / teleop 실행 제어
세 축을 GUI 이벤트와 연결합니다.
"""

import os
import signal
import subprocess
import sys
import shutil
from datetime import datetime

import yaml
from PyQt5 import uic
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QApplication, QMainWindow, QListWidgetItem,
                              QMessageBox, QFileDialog)

from ros_worker import RosWorker
from tts_worker import TTSWorker

UI_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'main_gui.ui')
# 폴더 구조: <root>/src/turtlebot3_pyqt_gui/main_gui.py, config는 <root>/config
_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_DIR = os.path.normpath(os.path.join(_PKG_DIR, '..', '..', 'config'))
if not os.path.isdir(CONFIG_DIR):
    # 배포/설치 환경 등 다른 위치에서 실행될 경우 대비 (없으면 기본 경유점 없이 시작)
    _fallback = os.path.join(_PKG_DIR, 'config')
    CONFIG_DIR = _fallback if os.path.isdir(_fallback) else CONFIG_DIR

# 저전압 경고 기준 (TurtleBot3 Burger/Waffle 3셀 리튬 배터리 기준 예시값 - 필요시 조정)
LOW_BATTERY_VOLTAGE = 11.0
LOW_BATTERY_PERCENT = 20.0

# 실행 명령 (실제 환경에 맞게 launch 파일/파라미터를 조정하세요)
BRINGUP_CMD = ['ros2', 'launch', 'turtlebot3_bringup', 'robot.launch.py']
NAV2_CMD = ['ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py',
            'use_sim_time:=False', 'map:=map.yaml']
TELEOP_CMD = ['ros2', 'run', 'turtlebot3_teleop', 'teleop_keyboard']

TTS_LANGS = [('한국어', 'ko'), ('English', 'en'), ('日本語', 'ja'), ('中文', 'zh-CN')]

TTS_PRESETS = {
    'btn_tts_preset1': '터틀봇이 출발합니다.',
    'btn_tts_preset2': '목표 지점에 도착했습니다.',
    'btn_tts_preset3': '배터리를 확인해 주세요.',
    'btn_tts_preset4': '장애물이 감지되었습니다.',
}


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi(UI_PATH, self)
        self.setWindowTitle('TurtleBot3 PyQt GUI Controller')

        # ---- 내부 상태 ----
        self.waypoints = []          # [{'name','x','y','yaw'}, ...]
        self.trajectories = {}       # {trajectory_label: [wp_name, wp_name, ...]}
        self.trajectory_running = False
        self.trajectory_queue = []
        self.trajectory_index = 0
        self.trajectory_total = 0

        self.processes = {'bringup': None, 'nav2': None, 'teleop': None}

        self.tts_thread = None

        # ---- ROS2 워커 시작 ----
        self.ros_worker = RosWorker()
        self._connect_ros_signals()
        self.ros_worker.start()

        # ---- 위젯 이벤트 연결 ----
        self._connect_ui_signals()

        # ---- 초기 데이터 로드 ----
        self._init_tts_lang_combo()
        self._load_default_config()

        self._log('GUI 프로그램 시작')

    # ==================================================================
    # 초기화
    # ==================================================================
    def _connect_ros_signals(self):
        w = self.ros_worker
        w.battery_updated.connect(self.on_battery_updated)
        w.cmdvel_updated.connect(self.on_cmdvel_updated)
        w.odom_updated.connect(self.on_odom_updated)
        w.scan_updated.connect(self.on_scan_updated)
        w.nav_state_updated.connect(self.on_nav_state_updated)
        w.goal_pose_updated.connect(self.on_goal_pose_updated)
        w.goal_reached.connect(self.on_goal_reached)
        w.connection_status.connect(self.on_connection_status)
        w.log_signal.connect(self._log)

    def _connect_ui_signals(self):
        # 경유점
        self.btn_wp_add.clicked.connect(self.add_waypoint)
        self.btn_wp_move.clicked.connect(self.move_selected_waypoint)
        self.btn_wp_delete.clicked.connect(self.delete_selected_waypoint)
        self.btn_wp_clear.clicked.connect(self.clear_waypoints)
        self.btn_wp_save.clicked.connect(self.save_waypoints_dialog)
        self.btn_wp_load.clicked.connect(self.load_waypoints_dialog)

        # trajectory
        self.btn_traj_run.clicked.connect(self.run_selected_trajectory)
        self.btn_traj_stop.clicked.connect(self.stop_trajectory)
        self.combo_trajectory.currentIndexChanged.connect(self.show_trajectory_info)

        # gTTS
        self.btn_tts_speak.clicked.connect(self.speak_tts)
        for btn_name, sentence in TTS_PRESETS.items():
            getattr(self, btn_name).clicked.connect(
                lambda _checked, s=sentence: self.edit_tts_text.setPlainText(s))

        # 프로세스 제어
        self.btn_bringup_start.clicked.connect(lambda: self.start_process('bringup', BRINGUP_CMD))
        self.btn_bringup_stop.clicked.connect(lambda: self.stop_process('bringup'))
        self.btn_nav2_start.clicked.connect(lambda: self.start_process('nav2', NAV2_CMD))
        self.btn_nav2_stop.clicked.connect(lambda: self.stop_process('nav2'))
        self.btn_teleop_start.clicked.connect(self.start_teleop)
        self.btn_teleop_stop.clicked.connect(lambda: self.stop_process('teleop'))
        self.btn_emergency_stop.clicked.connect(self.emergency_stop)

        # 로그
        self.btn_log_save.clicked.connect(self.save_log_dialog)
        self.btn_log_clear.clicked.connect(self.list_log.clear)

    def _init_tts_lang_combo(self):
        self.combo_tts_lang.clear()
        for label, code in TTS_LANGS:
            self.combo_tts_lang.addItem(label, code)

    def _load_default_config(self):
        wp_path = os.path.join(CONFIG_DIR, 'waypoints.yaml')
        traj_path = os.path.join(CONFIG_DIR, 'trajectories.yaml')
        if os.path.exists(wp_path):
            self._load_waypoints_from_file(wp_path)
        if os.path.exists(traj_path):
            self._load_trajectories_from_file(traj_path)

    # ==================================================================
    # 로그 / 상태
    # ==================================================================
    def _log(self, message: str):
        timestamp = datetime.now().strftime('%H:%M:%S')
        item = QListWidgetItem(f'[{timestamp}] {message}')
        self.list_log.addItem(item)
        self.list_log.scrollToBottom()

    def set_robot_status(self, text: str):
        self.lbl_robot_status.setText(text)

    # ==================================================================
    # ROS2 signal 핸들러
    # ==================================================================
    def on_connection_status(self, connected: bool):
        self.lbl_connection_status.setText('ROS2: 연결됨' if connected else 'ROS2: 연결 안됨')
        if connected:
            self.set_robot_status('로봇 연결 완료')

    def on_battery_updated(self, data: dict):
        voltage = data.get('voltage', 0.0)
        percentage = data.get('percentage', 0.0)
        self.lbl_battery_voltage.setText(f'{voltage:.2f} V')
        self.lbl_battery_percentage.setText(f'{percentage:.1f} %')

        if voltage <= 0.0:
            self.lbl_battery_msg.setText('데이터 없음')
        elif voltage < LOW_BATTERY_VOLTAGE or percentage < LOW_BATTERY_PERCENT:
            self.lbl_battery_msg.setText('저전압')
            self.lbl_battery_warning.setText('⚠ 배터리 저전압 경고! 충전이 필요합니다.')
            self._log(f'경고: 배터리 저전압 감지 (voltage={voltage:.2f}V, {percentage:.1f}%)')
        else:
            self.lbl_battery_msg.setText('정상')
            self.lbl_battery_warning.setText('')

    def on_cmdvel_updated(self, linear_x: float, angular_z: float):
        self.lbl_cmdvel_linear.setText(f'{linear_x:.3f}')
        self.lbl_cmdvel_angular.setText(f'{angular_z:.3f}')
        if abs(linear_x) < 1e-3 and abs(angular_z) < 1e-3:
            state = '정지'
        elif abs(linear_x) >= 1e-3 and abs(angular_z) < 1e-3:
            state = '전진' if linear_x > 0 else '후진'
        elif abs(linear_x) < 1e-3 and abs(angular_z) >= 1e-3:
            state = '제자리 회전'
        else:
            state = '이동+회전'
        self.lbl_motion_state.setText(state)

    def on_odom_updated(self, x: float, y: float, yaw_deg: float):
        self.lbl_odom_xy.setText(f'{x:.2f}, {y:.2f}')
        self.lbl_odom_yaw.setText(f'{yaw_deg:.1f}')

    def on_scan_updated(self, min_range: float):
        if min_range == float('inf'):
            self.lbl_lidar_min.setText('--')
            return
        self.lbl_lidar_min.setText(f'{min_range:.2f} m')
        if min_range < 0.25:
            self.set_robot_status('장애물 감지')
            self._log(f'장애물 감지 (최소거리 {min_range:.2f} m)')

    def on_nav_state_updated(self, state: str):
        self.lbl_nav_state.setText(state)

    def on_goal_pose_updated(self, x: float, y: float, yaw: float):
        self.lbl_goal_pose.setText(f'x={x:.2f}, y={y:.2f}, yaw={yaw:.1f}')

    def on_goal_reached(self, x: float, y: float, yaw: float):
        self.set_robot_status('목표 지점 도착')
        # trajectory 실행 중이면 다음 waypoint 진행
        if self.trajectory_running:
            self._advance_trajectory()

    # ==================================================================
    # 경유점 (요구사항 6)
    # ==================================================================
    def add_waypoint(self):
        name = self.edit_wp_name.text().strip()
        if not name:
            QMessageBox.warning(self, '입력 오류', '경유점 이름을 입력하세요.')
            return
        if any(wp['name'] == name for wp in self.waypoints):
            QMessageBox.warning(self, '입력 오류', '이미 존재하는 경유점 이름입니다.')
            return
        wp = {
            'name': name,
            'x': self.spin_wp_x.value(),
            'y': self.spin_wp_y.value(),
            'yaw': self.spin_wp_yaw.value(),
        }
        self.waypoints.append(wp)
        self._refresh_waypoint_list()
        self._log(f'경유점 {name} 등록 완료 (x={wp["x"]:.2f}, y={wp["y"]:.2f}, yaw={wp["yaw"]:.1f})')
        self.edit_wp_name.clear()

    def _refresh_waypoint_list(self):
        self.list_waypoints.clear()
        for wp in self.waypoints:
            text = f'{wp["name"]}  (x={wp["x"]:.2f}, y={wp["y"]:.2f}, yaw={wp["yaw"]:.1f})'
            self.list_waypoints.addItem(QListWidgetItem(text))
        self._refresh_trajectory_combo_source()

    def _selected_waypoint(self):
        row = self.list_waypoints.currentRow()
        if row < 0 or row >= len(self.waypoints):
            return None
        return self.waypoints[row]

    def move_selected_waypoint(self):
        wp = self._selected_waypoint()
        if wp is None:
            QMessageBox.information(self, '선택 필요', '이동할 경유점을 목록에서 선택하세요.')
            return
        self.set_robot_status('경유점 이동 중')
        self._log(f'경유점 {wp["name"]} 이동 시작')
        self.ros_worker.move_to(wp['x'], wp['y'], wp['yaw'])

    def delete_selected_waypoint(self):
        row = self.list_waypoints.currentRow()
        if row < 0:
            return
        removed = self.waypoints.pop(row)
        self._refresh_waypoint_list()
        self._log(f'경유점 {removed["name"]} 삭제됨')

    def clear_waypoints(self):
        if not self.waypoints:
            return
        reply = QMessageBox.question(self, '전체 초기화', '모든 경유점을 삭제하시겠습니까?')
        if reply == QMessageBox.Yes:
            self.waypoints.clear()
            self._refresh_waypoint_list()
            self._log('전체 경유점 초기화됨')

    def save_waypoints_dialog(self):
        path, _ = QFileDialog.getSaveFileName(
            self, '경유점 저장', CONFIG_DIR, 'YAML Files (*.yaml)')
        if path:
            with open(path, 'w', encoding='utf-8') as f:
                yaml.safe_dump({'waypoints': self.waypoints}, f, allow_unicode=True)
            self._log(f'경유점을 파일로 저장했습니다: {path}')

    def load_waypoints_dialog(self):
        path, _ = QFileDialog.getOpenFileName(
            self, '경유점 불러오기', CONFIG_DIR, 'YAML Files (*.yaml)')
        if path:
            self._load_waypoints_from_file(path)

    def _load_waypoints_from_file(self, path):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            self.waypoints = data.get('waypoints', [])
            self._refresh_waypoint_list()
            self._log(f'경유점 파일 로드 완료: {path} ({len(self.waypoints)}개)')
        except Exception as e:  # noqa: BLE001
            self._log(f'경유점 파일 로드 실패: {e}')

    # ==================================================================
    # Trajectory (요구사항 7)
    # ==================================================================
    def _refresh_trajectory_combo_source(self):
        """기본 trajectory가 없으면 현재 등록된 waypoint로 임시 trajectory 2개를 자동 구성"""
        if self.trajectories:
            return
        if len(self.waypoints) >= 2:
            names = [wp['name'] for wp in self.waypoints]
            self.trajectories = {
                'Trajectory 1 - 전체 순회': names,
                'Trajectory 2 - 역순 순회': list(reversed(names)),
            }
            self._refresh_trajectory_combo()

    def _refresh_trajectory_combo(self):
        self.combo_trajectory.clear()
        for label in self.trajectories.keys():
            self.combo_trajectory.addItem(label)

    def _load_trajectories_from_file(self, path):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            self.trajectories = {
                item['name']: item['waypoints'] for item in data.get('trajectories', [])
            }
            self._refresh_trajectory_combo()
            self._log(f'Trajectory 파일 로드 완료: {path} ({len(self.trajectories)}개)')
        except Exception as e:  # noqa: BLE001
            self._log(f'Trajectory 파일 로드 실패: {e}')

    def show_trajectory_info(self):
        label = self.combo_trajectory.currentText()
        wp_names = self.trajectories.get(label, [])
        self.text_traj_info.setPlainText(' -> '.join(wp_names) if wp_names else '(경유점 없음)')

    def run_selected_trajectory(self):
        label = self.combo_trajectory.currentText()
        wp_names = self.trajectories.get(label, [])
        if not wp_names:
            QMessageBox.information(self, '실행 불가', '선택한 trajectory에 경유점이 없습니다.')
            return
        # 이름 -> 좌표 매핑 (경유점은 중복 방문 가능)
        wp_map = {wp['name']: wp for wp in self.waypoints}
        queue = []
        for name in wp_names:
            if name not in wp_map:
                QMessageBox.warning(self, '경유점 없음', f'경유점 "{name}"이 등록되어 있지 않습니다.')
                return
            queue.append(wp_map[name])

        self.trajectory_queue = queue
        self.trajectory_index = 0
        self.trajectory_total = len(queue)
        self.trajectory_running = True
        self.progress_mission.setValue(0)
        self.set_robot_status('trajectory 주행 중')
        self._log(f'{label} 실행 시작 (경유점 {self.trajectory_total}개)')
        self._advance_trajectory()

    def _advance_trajectory(self):
        if not self.trajectory_running:
            return
        if self.trajectory_index >= self.trajectory_total:
            self.trajectory_running = False
            self.progress_mission.setValue(100)
            self.lbl_traj_current_wp.setText('현재 waypoint: 완료')
            self._log('Trajectory 완료')
            self.set_robot_status('목표 지점 도착')
            return

        wp = self.trajectory_queue[self.trajectory_index]
        self.lbl_traj_current_wp.setText(
            f'현재 waypoint: {wp["name"]} ({self.trajectory_index + 1}/{self.trajectory_total})')
        progress = int(self.trajectory_index / self.trajectory_total * 100)
        self.progress_mission.setValue(progress)
        self.trajectory_index += 1
        self.ros_worker.move_to(wp['x'], wp['y'], wp['yaw'])

    def stop_trajectory(self):
        if self.trajectory_running:
            self.trajectory_running = False
            self.ros_worker.cancel_move()
            self._log('Trajectory 주행 정지')
            self.set_robot_status('로봇 연결 완료')

    # ==================================================================
    # gTTS (요구사항 8)
    # ==================================================================
    def speak_tts(self):
        text = self.edit_tts_text.toPlainText().strip()
        if not text:
            QMessageBox.information(self, '입력 필요', '출력할 문장을 입력하세요.')
            return
        lang_code = self.combo_tts_lang.currentData()
        self.lbl_tts_status.setText('음성 변환 중...')
        self.btn_tts_speak.setEnabled(False)

        self.tts_thread = TTSWorker(text, lang_code)
        self.tts_thread.finished_ok.connect(self._on_tts_ok)
        self.tts_thread.finished_error.connect(self._on_tts_error)
        self.tts_thread.start()

    def _on_tts_ok(self, text):
        self.lbl_tts_status.setText('출력 완료')
        self.btn_tts_speak.setEnabled(True)
        self._log(f'TTS 출력 완료: "{text}"')

    def _on_tts_error(self, message):
        self.lbl_tts_status.setText('오류 발생')
        self.btn_tts_speak.setEnabled(True)
        self._log(f'TTS 오류: {message}')

    # ==================================================================
    # 프로세스 제어: bringup / nav2 / teleop (요구사항 9)
    # ==================================================================
    def start_process(self, key, cmd):
        if self.processes.get(key) is not None:
            self._log(f'{key} 이미 실행 중입니다.')
            return
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,  # 프로세스 그룹으로 실행 -> 자식 launch 프로세스까지 종료 가능
            )
            self.processes[key] = proc
            self._set_process_status(key, True)
            self.set_robot_status(f'{key} 실행 중')
            self._log(f'{key} 실행 시작: {" ".join(cmd)}')
        except FileNotFoundError:
            self._log(f'오류: 명령을 찾을 수 없습니다 ({" ".join(cmd)}). ROS2 환경을 source 했는지 확인하세요.')
        except Exception as e:  # noqa: BLE001
            self._log(f'{key} 실행 실패: {e}')

    def start_teleop(self):
        """teleop_keyboard는 터미널 입력이 필요하므로 가능하면 새 터미널에서 실행합니다."""
        if self.processes.get('teleop') is not None:
            self._log('teleop 이미 실행 중입니다.')
            return
        if shutil.which('xterm'):
            cmd = ['xterm', '-e'] + TELEOP_CMD
        elif shutil.which('gnome-terminal'):
            cmd = ['gnome-terminal', '--'] + TELEOP_CMD
        else:
            cmd = TELEOP_CMD
            self._log('경고: 새 터미널 프로그램(xterm/gnome-terminal)을 찾지 못해 teleop을 백그라운드로 실행합니다. '
                       '키보드 입력이 필요하다면 별도 터미널에서 직접 실행하세요.')
        self.start_process('teleop', cmd)

    def stop_process(self, key):
        proc = self.processes.get(key)
        if proc is None:
            self._log(f'{key} 실행 중이 아닙니다.')
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        finally:
            self.processes[key] = None
            self._set_process_status(key, False)
            self._log(f'{key} 종료됨')

    def _set_process_status(self, key, running: bool):
        label = getattr(self, f'lbl_{key}_status', None)
        if label is not None:
            label.setText('실행 중' if running else '중지됨')

    def emergency_stop(self):
        self._log('긴급 정지 실행')
        self.trajectory_running = False
        self.ros_worker.cancel_move()
        self.ros_worker.emergency_stop()
        self.set_robot_status('긴급 정지됨')

    # ==================================================================
    # 로그 저장
    # ==================================================================
    def save_log_dialog(self):
        path, _ = QFileDialog.getSaveFileName(self, '로그 저장', '.', 'Text Files (*.txt)')
        if not path:
            return
        with open(path, 'w', encoding='utf-8') as f:
            for i in range(self.list_log.count()):
                f.write(self.list_log.item(i).text() + '\n')
        self._log(f'로그를 파일로 저장했습니다: {path}')

    # ==================================================================
    # 종료 처리
    # ==================================================================
    def closeEvent(self, event):
        for key in list(self.processes.keys()):
            if self.processes.get(key) is not None:
                self.stop_process(key)
        self.ros_worker.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
