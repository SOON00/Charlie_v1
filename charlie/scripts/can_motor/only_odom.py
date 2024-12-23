#!/usr/bin/env python

import rospy
import numpy as np
import math
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, TransformStamped
import tf
import tf.transformations

class Odom_class:
    def __init__(self):
        rospy.init_node('leo_odometry', anonymous=True)

        # 로봇의 기본 파라미터
        self.wheels_separation = 0.48  # 휠 간 거리 (m)
        self.wheels_radius = 0.08255  # 휠 반지름 (m)

        # 상태 변수들
        self.robot_pose = np.array([0.0, 0.0, 0.0])  # 로봇의 위치 (x, y, theta)
        self.robot_vel = np.array([0.0, 0.0, 0.0])  # 로봇의 속도 (linear, angular)
        self.last_theta = 0.0  # 마지막 각도 (theta)
        self.diff_joint_positions = np.array([0.0, 0.0])  # 좌우 휠의 변화량
        self.last_time = rospy.Time.now()  # 시간 초기화

        # Publisher 및 Subscriber
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)

        # 주기적으로 퍼블리시
        self.rate = rospy.Rate(10)  # 10Hz

    def cmd_vel_callback(self, msg):
        """
        cmd_vel 토픽을 구독하여 로봇의 속도를 받아옴
        """
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        current_time = rospy.Time.now()
        duration = (current_time - self.last_time).to_sec()

        self.calculate_odometry(linear_velocity, angular_velocity, duration)
        self.last_time = current_time  # 시간 갱신

    def calculate_odometry(self, linear_velocity, angular_velocity, duration):
        """
        속도 데이터를 기반으로 오도메트리 계산
        """
        # 이동거리와 회전각 계산
        delta_s = linear_velocity * duration
        delta_theta = angular_velocity * duration

        # 로봇의 위치 갱신
        self.robot_pose[0] += delta_s * math.cos(self.robot_pose[2] + (delta_theta / 2.0))
        self.robot_pose[1] += delta_s * math.sin(self.robot_pose[2] + (delta_theta / 2.0))
        self.robot_pose[2] += delta_theta

        # 속도 업데이트
        self.robot_vel[0] = linear_velocity
        self.robot_vel[2] = angular_velocity

    def publish(self):
        """
        계산된 오도메트리를 퍼블리시
        """
        current_time = rospy.Time.now()

        # 오도메트리 메시지 생성
        odom_msg = Odometry()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.header.stamp = current_time

        # 로봇의 위치 업데이트
        odom_msg.pose.pose.position.x = self.robot_pose[0]
        odom_msg.pose.pose.position.y = self.robot_pose[1]
        odom_msg.pose.pose.position.z = 0.0

        # 로봇의 orientation (회전각도) 업데이트
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, self.robot_pose[2])
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # 속도 정보 업데이트
        odom_msg.twist.twist.linear.x = self.robot_vel[0]
        odom_msg.twist.twist.angular.z = self.robot_vel[2]

        # TF 변환 정보 업데이트
        odom_tf = TransformStamped()
        odom_tf.transform.translation.x = odom_msg.pose.pose.position.x
        odom_tf.transform.translation.y = odom_msg.pose.pose.position.y
        odom_tf.transform.translation.z = odom_msg.pose.pose.position.z
        odom_tf.transform.rotation = odom_msg.pose.pose.orientation
        odom_tf.header.frame_id = 'odom'
        odom_tf.child_frame_id = 'base_footprint'
        odom_tf.header.stamp = current_time

        # TF 브로드캐스트를 위한 sendTransform 호출
        self.tf_broadcaster.sendTransform(
            (odom_tf.transform.translation.x, odom_tf.transform.translation.y, odom_tf.transform.translation.z),
            (odom_tf.transform.rotation.x, odom_tf.transform.rotation.y, odom_tf.transform.rotation.z, odom_tf.transform.rotation.w),
            current_time,
            odom_tf.child_frame_id,
            odom_tf.header.frame_id
        )

        # 퍼블리시
        self.odom_pub.publish(odom_msg)

    def run(self):
        """
        주기적으로 odom 메시지를 퍼블리시
        """
        while not rospy.is_shutdown():
            self.publish()
            self.rate.sleep()  # 10Hz 주기로 퍼블리시

if __name__ == '__main__':
    try:
        odom_node = Odom_class()
        odom_node.run()  # 주기적으로 odom 메시지를 퍼블리시
    except rospy.ROSInterruptException:
        pass

