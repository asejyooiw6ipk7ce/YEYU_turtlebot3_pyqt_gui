import os
import sys
import threading   
import rclpy   
from PyQt5.QtWidgets import QApplication

from qt_signals import RosSignals
from ros_node import TurtleBot3RosNode
from gui import TurtleBot3GUI

def main():
    app = QApplication(sys.argv)

    if not rclpy.ok():
        rclpy.init(args=None)

    ros_node = TurtleBot3GuiNode()

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