#!/usr/bin/env python3

from PyQt5.QtCore import QObject, pyqtSignal

class RosSignals(QObject):
    # YAML 로드 시 경유점 이름 리스트와 경로 이름 리스트를 모두 전달하도록 설정
    yaml_loaded = pyqtSignal(list, list)  
    log_triggered = pyqtSignal(str)

    # ROS 2 데이터를 GUI 스레드로 안전하게 전달하기 위한 시그널
    odom_received = pyqtSignal(float, float, float)  # x, y, yaw
    scan_received = pyqtSignal(float)                # 최소 거리
    battery_received = pyqtSignal(float, float)      # 퍼센트, 전압