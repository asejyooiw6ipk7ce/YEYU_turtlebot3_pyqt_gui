import sys
import os                                                                 # os의 환경 변수를 ROS_DOMAIN_ID 값으로 저장하기 위해
import subprocess
import signal 
import time                                              # Ros2노드 단독일때 쓰는중(QTimer대신)
import math                                                               # yaw 계산(도->쿼터니언)
import rclpy                                                              # ros2 client library for python (TurtleBot3GuiNode)
from rclpy.node import Node                                               # Node 클래스 (TurtleBot3GuiNode(Node))
from geometry_msgs.msg import Twist                                       # msg
from nav_msgs.msg import Odometry                                         # msg
from pathlib import Path
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer                                           # PYQT5 루프를 굴리면서 ROS2 루프 굴리기 위해 QTimer 데려옴
from geometry_msgs.msg import PoseWithCovarianceStamped                   # ROS2 내비게이션 시스템에서 로봇 위치와 방향 전달할 때 사용되는 msg규격(초기위치 지정할 때 반드시 이 형식으로 보내야 로봇이 이해)
from rclpy.action import ActionClient                                     # /navigate_to_pose
from nav2_msgs.action import NavigateToPose                               # /navigate_to_pose
from geometry_msgs.msg import PoseStamped                                 # /navigate_to_pose
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import FollowWaypoints                              # follow_waypoints action
from sensor_msgs.msg import BatteryState                                  # /battery_state 토픽 msg


class TurtleBot3GuiNode(Node):
    def __init__(self, namespace=''):                                     # 클래스 생성자. namespace값 인자로 받음(로봇 여러대면 구분하기 위해)
        super().__init__('turtlebot3_burger_gui')                         # 부모 생성자 호출 + 노드 이름 지정

        # battery_state
        self.battery_sub = self.create_subscription(
            BatteryState, 
            '/battery_state',
            self.battery_callback,
            10
        )
        self.last_battery_p = None
        self.last_battery_v = None
        #self.battery_status_msg = "Unknown"
       
        # cmd_topic 발행자 생성
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
    
        # odom_topic 수신자 생성
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.last_odom = None
        
        # initpos_topic 발행자 생성
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # 단일 목표지 이동 액션 클라이언트
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )
        #self.goal_handle = None

        # 여러 경유점 순차 주행 액션 클라이언트
        self.follow_client = ActionClient(
            self,
            FollowWaypoints,
            '/follow_waypoints'
        )


        '''[/scan 수신]'''
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # scan_topic 수신자 정의
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )

        self.last_scan_min = None
        
        

		# battery_status 발행함수 정의
    def battery_callback(self, msg):
        # self.last_battery = msg.percentage * 100.0     #결과 : 8666%
        self.last_battery_p = msg.percentage 
        self.last_battery_v = msg.voltage

        # print(f"로봇이 보낸 실제 power_supply_status 숫자: {msg.power_supply_status}")
        #status_dict = {1: "Charging", 2: "Discharging", 3: "Not Charging", 4: "Full"}
        #self.battery_status_msg = status_dict.get(msg.power_supply_status, "Unknown") # 토픽 msg에서 status_dict[msg.power_supply_status]를 가져오는데 만약 없는 거면 "Unknown"을 띄움
        #print(f"Battery: {self.last_battery_p:.1f}%, Status: {self.battery_status_msg}")

    # cmd_topic 발행함수 정의
    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    # odom_topic 수신받았을 때 콜백함수 정의
    def odom_callback(self, msg):
        self.last_odom = msg        # 받은 msg를 lase_odom에 저장

    # initpos_topic 발행함수 정의
    def publish_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()

        msg.header.frame_id = 'map'                # 이 좌표의 기준 'map'으로 지정
        msg.header.stamp = self.get_clock().now().to_msg() # msg가 발행되는 현재 컴퓨터 시간의 타임스탬프 찍어줌 

        msg.pose.pose.position.x = float(x)        # 입력한 위치를 메세지 주머니에 대입
        msg.pose.pose.position.y = float(y)

        qx, qy, qz, qw = yaw_to_quaternion(yaw)    # 각도->쿼터니언 값 qx,qy,qz,qw에 담아
        msg.pose.pose.orientation.x = qx           # 쿼터니언 값 주머니에 채움
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # 공분산 ; "내가 지금 찍어준 이 위치가 얼마나 불확실한가"에 대한 에러 확률 지표
        ''' -> 자율주행 알고리즘(AMCL)이 이를 기반으로 로봇 주변 파티클 흩뿌려 위치 추정 시작할 수 있게 됨'''
        msg.pose.covariance[0] = 0.25      # X오차
        msg.pose.covariance[7] = 0.25      # y오차
        msg.pose.covariance[35] = 0.0685   # 각도 오차

        # 발행
        self.initial_pose_pub.publish(msg)   
    
    # navpose_topic 액션클라이언트 ; goal 전송 함수
    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)

        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            return False, 'Nav2 action server is not available'

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

        return True, f'Goal sent: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
    
    # navpose_topic 액션클라이언트 ; goal 응답 피드백 함수
    def _goal_response_callback(self, future):
        self.goal_handle = future.result()

        if self.goal_handle and self.goal_handle.accepted:
            self.get_logger().info('Goal accepted')
        else:
            self.get_logger().warn('Goal rejected')

    # navpose_topic 액션클라이언트 ; goal 취소
    def cancel_goal(self):
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
            return True

        return False
    
    # scan_topic 콜백함수
    def scan_callback(self, msg):
        values = [
            v for v in msg.ranges
            if math.isfinite(v) and v > 0.0
        ]

        self.last_scan_min = min(values) if values else None

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot3GuiNode()

    print("=========test=========")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            print(f"[배터리] 잔량: {node.last_battery_p:.2f}% ,")
            print(f"전압: {node.last_battery_v:.2f}V ")
            #print(f"상태: {node.battery_status_msg}")
            print(f"[명령] 최근명령: {node.last_cmd_linear} m/s , {node.last_cmd_angular} rad/s")
            print(f"odom {node}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("테스트 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()