import signal
import subprocess
import sys
import os
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QTextEdit, QVBoxLayout, QLabel
from PyQt5.QtCore import QProcess, QTimer
from pathlib import Path
from PyQt5 import uic
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


ROBOT_USER = "yeyu"
ROBOT_IP = "192.168.230.100"

ROBOT = f"{ROBOT_USER}@{ROBOT_IP}"


class TurtleBot3GUI(QWidget):
    def __init__(self):
        super().__init__()

        ui_path = Path(__file__).parent.parent / "resource" / "turtlebot3_pyqt_gui2.ui"
        uic.loadUi(str(ui_path), self)

        self.processes = []

        self.node = None

        self.connect_signals() 

        # self.robot_state_lineEdit.setText('Status: Unknown')

    def connect_signals(self):
        self.connect_PB.clicked.connect (self.connect_ros)
        self.nav2_PB.clicked.connect(lambda: self.run__command('nav2',['ros2','launch', 'turtlebot3_navigation2', 'navigation2.launch.py', 'use_sim_time:=false']))
        self.rviz_PB.clicked.connect(lambda: self.run__command('rviz',['rviz2']))
        self.teleop_PB.clicked.connect(lambda: self.run__command('teleop',['ros2','run', 'turtlebot3_teleop', 'teleop_keyboard']))
        self.stopall_PB.clicked.connect(self.stop_all_processes)
        self.bringup_PB.clicked.connect(self.bringup_ros)
        self.stop_bringup_PB.clicked.connect(self.bringup_stop)
        self.forward_PB.clicked.connect(lambda: self.send_velocity(self.linear_spinBox.value(),0.0))


    
    def connect_ros(self):

        os.environ['ROS_DOMAIN_ID'] = '40'

        self.robot_state_lineEdit.setText('Connected to ROS')

        if not rclpy.ok():
            rclpy.init(args=None)

        print('ROS 2 connected')
    
    
    def disconnect_ros(self):

        if self.node:
            self.destroy_node()
            self.node = None

        self.robot_state_lineEdit.setText('Disconnected from ROS')

        print ('ROS 2 disconnected')
    
    


    def run__command(self,name,cmd):
        env = os.environ.copy()
        env['TURTLEBOT3_MODEL'] = env.get('TURTLEBOT3_MODEL', 'burger')  # Set default model if not set

        try:
            proc = subprocess.Popen(cmd, env=env, preexec_fn = os.setsid)

            self.processes.append((name, proc))
            print(f'{name} launched: {"".join(cmd)}')

        except FileNotFoundError:
            print(f'Command not found: {cmd[0]}')

        
        except Exception as e:
            print(f'Error launching {name}: {e}')

    
    def stop_all_processes(self):
        self.log_text.append("Stopping bringup...")
        self.run_ssh('~/tb3_scripts/stop_bringup.sh')

        for name, proc in self.processes:
            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                print(f'{name} stopped')
        self.log_text.append(f'{name} stopped')
        print(f'{name} stopped')

        self.processes.clear()
        self.log_text.append("All processes stopped.")

                

    def run_ssh(self,command):
        self.process = QProcess(self)

        ssh_command=[ROBOT, command]

        self.process.readyReadStandardOutput.connect(self.read_stdout)
        self.process.readyReadStandardError.connect(self.read_stderr)

        self.process.start('ssh', ssh_command)

    def read_stdout(self):
        data = self.process.readAllStandardOutput().data().decode()
        data = data.strip()

        if data:
            self.log_text.append(data)

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
            self.log_text.append(data)

    def bringup_ros(self):
        self.log_text.append("Starting bringup...")
        self.run_ssh('~/tb3_scripts/start_bringup.sh')


    def bringup_stop(self):
        self.log_text.append("Stopping bringup...")
        self.run_ssh('~/tb3_scripts/stop_bringup.sh')

    def send_velocity(self,linear,angular):
        self.current_cmd_vel_lineEdit.setText(f"Linear: {linear:.2f}, Angular: {angular:.2f}")
        print(f"cmd_vel - Linear: {linear:.2f}, Angular: {angular:.2f}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TurtleBot3GUI()
    window.show()
    sys.exit(app.exec_())