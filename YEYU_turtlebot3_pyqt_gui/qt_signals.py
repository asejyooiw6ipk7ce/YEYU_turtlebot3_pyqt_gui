
from PyQt5.QtCore import QObject, pyqtSignal, QProcess                    # RosSignals 클래스 추가시 필요한 것들  

# ros2 콜백 - PyQt 테이터 전달
class RosSignals(QObject):
	# 문자열 전달할 수 있는 Qt 시그널 정의
    yaml_loaded = pyqtSignal(list)    #yaml_loaded = pyqtSignal(list,list) -> (list)로 변경      
    log_triggered = pyqtSignal(str)
    '''신호통로    이 통로로는 list 데이터만 보낼거야(통로 종류 지정) '''

    # QTimer->멀티스레드로 인해 ui_timer의 슬롯함수를 수정하는 과정에서 생긴 코드
    odom_received = pyqtSignal(float, float, float)  # x, y, yaw 전달
    scan_received = pyqtSignal(float)                # 최소 거리 전달
    battery_received = pyqtSignal(float, float)      # 퍼센트, 전압 전달
