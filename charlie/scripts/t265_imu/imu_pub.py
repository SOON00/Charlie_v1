#!/usr/bin/env python
import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

def odom_to_imu(odom_msg):
    imu_msg = Imu()
    imu_msg.header = odom_msg.header

    # T265의 orientation을 가져옵니다.
    orientation = odom_msg.pose.pose.orientation

    # 쿼터니언을 회전 행렬로 변환
    rotation_matrix = tf.transformations.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])

    # Y축 기준으로 90도 회전하는 회전 행렬 생성
    rotation_90_y = tf.transformations.rotation_matrix(np.pi/2, [0, 1, 0])

    # 두 회전 행렬을 곱하여 최종 회전 행렬을 구합니다.
    final_rotation_matrix = np.dot(rotation_90_y, rotation_matrix)

    # 회전 행렬을 다시 쿼터니언으로 변환
    quat = tf.transformations.quaternion_from_matrix(final_rotation_matrix)

    # 새로운 쿼터니언을 IMU 메시지에 적용
    imu_msg.orientation.x = quat[2]
    imu_msg.orientation.y = quat[1]
    imu_msg.orientation.z = quat[0]
    imu_msg.orientation.w = quat[3]

    # IMU 메시지를 발행
    imu_pub.publish(imu_msg)

if __name__ == '__main__':
    rospy.init_node('odom_to_imu')
    imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    rospy.Subscriber('/camera/odom/sample', Odometry, odom_to_imu)
    rospy.spin()

