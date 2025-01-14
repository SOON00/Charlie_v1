#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
import math

class JointStatePublisher:
    def __init__(self):
        rospy.init_node('joint_state_publisher')  # 노드 초기화
        self.left_wheel_angle = 0.0  # 왼쪽 바퀴 초기 각도
        self.right_wheel_angle = 0.0  # 오른쪽 바퀴 초기 각도

        self.wheel_radius = 0.1  # 바퀴 반지름 (예시)
        self.axle_length = 0.35  # 바퀴 간 거리 (예시)

        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.rate = rospy.Rate(10)  # 10 Hz

        self.linear_velocity = 0.0  # 초기 선형 속도
        self.angular_velocity = 0.0  # 초기 각속도

    def cmd_vel_callback(self, msg):
        # cmd_vel 메시지에서 선형 속도와 각속도 추출
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def publish_joint_state(self):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.name = ['wheel_left_joint', 'wheel_right_joint']

        while not rospy.is_shutdown():
            # 바퀴 속도 계산
            left_wheel_velocity = (self.linear_velocity - (self.angular_velocity * self.axle_length / 2)) / self.wheel_radius
            right_wheel_velocity = (self.linear_velocity + (self.angular_velocity * self.axle_length / 2)) / self.wheel_radius

            # 바퀴 각도 업데이트
            self.left_wheel_angle += left_wheel_velocity * 0.1  # 0.1초 동안 회전한 각도
            self.right_wheel_angle += right_wheel_velocity * 0.1

            joint_state.position = [self.left_wheel_angle, self.right_wheel_angle]
            joint_state.header.stamp = rospy.Time.now()

            self.pub.publish(joint_state)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        joint_state_publisher = JointStatePublisher()
        joint_state_publisher.publish_joint_state()
    except rospy.ROSInterruptException:
        pass

