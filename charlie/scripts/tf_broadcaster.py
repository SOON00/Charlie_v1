#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def static_transforms():
    static_br = tf2_ros.StaticTransformBroadcaster()

    # camera_pose_frame -> laser 변환 (레이저 센서 위치)
    laser_transform = TransformStamped()
    laser_transform.header.stamp = rospy.Time.now()
    laser_transform.header.frame_id = "base_link"
    laser_transform.child_frame_id = "laser"
    laser_transform.transform.translation.x = 0.0
    laser_transform.transform.translation.y = 0.0
    laser_transform.transform.translation.z = 0.25  # 레이저의 높이
    laser_transform.transform.rotation.x = 0.0
    laser_transform.transform.rotation.y = 0.0
    laser_transform.transform.rotation.z = 0.0
    laser_transform.transform.rotation.w = 1.0

    static_br.sendTransform([laser_transform])


def main():
    rospy.init_node("tf_broadcaster", anonymous=True)

    # 정적 변환 브로드캐스트 실행
    static_transforms()

    rospy.spin()

if __name__ == "__main__":
    main()

