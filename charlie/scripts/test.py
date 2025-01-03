#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import tf
import math

def odom_callback(msg):
    # Odometry 메시지에서 orientation을 추출
    orientation_q = msg.pose.pose.orientation
    
    # 쿼터니언을 yaw (라디안)으로 변환
    yaw = tf.transformations.euler_from_quaternion([0, 0, orientation_q.z, orientation_q.w])[2]
    
    # 라디안을 degree로 변환
    yaw_deg = math.degrees(yaw)
    
    # 출력
    rospy.loginfo("Yaw in degrees: %.2f", yaw_deg)

def listener():
    # ROS 노드 초기화
    rospy.init_node('odom_listener', anonymous=True)

    # /odom 토픽을 구독
    rospy.Subscriber("/odom", Odometry, odom_callback)

    # ROS 이벤트 루프를 시작
    rospy.spin()

if __name__ == '__main__':
    listener()

