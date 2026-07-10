import os
import sys
import threading   
import rclpy   
from PyQt5.QtWidgets import QApplication

# 지금은 ros2 run이 아니라 python3 main.py로 직접 실행하는 방식이라서
# 점(.) 없는 일반 import를 그대로 씁니다. (이 폴더 안에서 실행하면 문제없이 동작합니다)
from YEYU_turtlebot3_pyqt_gui.src.YEYU_turtlebot3_pyqt_gui.ros_node import TurtleBot3RosNode
from YEYU_turtlebot3_pyqt_gui.src.YEYU_turtlebot3_pyqt_gui.gui import TurtleBot3GUI

def main():
    os.environ['ROS_DOMAIN_ID'] = '40'
    app = QApplication(sys.argv)

    if not rclpy.ok():
        rclpy.init(args=None)

    ros_node = TurtleBot3RosNode()

    # 백그라운드 스레드 생성 및 시작 (rclpy.spin을 통째로 넘김)
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    window = TurtleBot3GUI(ros_node)
    window.show()

    exit_code = app.exec_()

    ros_node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()