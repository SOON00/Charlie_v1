#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped

def imu_callback(msg):
    # 현재 시간으로 트랜스폼 생성
    timestamp = rospy.Time.now()

    # laser와 같은 위치 및 오리엔테이션 사용
    position = (-0.07, 0.0, 0.0)  # 위치 정보
    orientation = (0.0, 0.0, 0.0, 1.0)  # laser와 같은 오리엔테이션 (쿼터니언)

    # 트랜스폼 퍼블리시
    broadcaster.sendTransform(
        position,  # 위치 정보
        orientation,  # 회전 정보
        timestamp,  # 현재 시간
        'imu_link',  # 자식 프레임
        'laser'  # 부모 프레임
    )

def imu_to_tf():
    rospy.init_node('imu_to_tf')

    # TF 브로드캐스터 초기화
    global broadcaster
    broadcaster = tf.TransformBroadcaster()

    # IMU 데이터 구독
    rospy.Subscriber('/imu/data', Imu, imu_callback)

    rospy.spin()

if __name__ == '__main__':
    imu_to_tf()

