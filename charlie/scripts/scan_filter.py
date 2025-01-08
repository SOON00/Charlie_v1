#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import math

def scan_callback(scan_msg):
    # 새로운 LaserScan 메시지 생성
    filtered_scan_msg = LaserScan()
    filtered_scan_msg.header.stamp = rospy.Time.now()
    filtered_scan_msg.header.frame_id = scan_msg.header.frame_id
    
    # 해상도 낮추기
    filtered_scan_msg.angle_min = scan_msg.angle_min
    filtered_scan_msg.angle_max = scan_msg.angle_max
    
    # 1도 간격으로 해상도를 변경 (기존 해상도보다 더 낮은 값으로 설정)
    filtered_scan_msg.angle_increment = math.pi / 180  # 1도 간격
    
    # Range 값 설정
    filtered_scan_msg.range_min = scan_msg.range_min
    filtered_scan_msg.range_max = scan_msg.range_max
    
    # 기존 ranges와 intensities 값에서 일부 샘플링하여 해상도 줄이기
    num_ranges = int((filtered_scan_msg.angle_max - filtered_scan_msg.angle_min) / filtered_scan_msg.angle_increment)
    filtered_scan_msg.ranges = [scan_msg.ranges[i] for i in range(0, len(scan_msg.ranges), int(len(scan_msg.ranges) / num_ranges))]
    filtered_scan_msg.intensities = [scan_msg.intensities[i] for i in range(0, len(scan_msg.intensities), int(len(scan_msg.intensities) / num_ranges))]

    # 필터링된 데이터를 /scan 토픽으로 발행
    scan_pub.publish(filtered_scan_msg)
    #rospy.loginfo("Publishing filtered scan data...")

def laser_scan_subscriber():
    rospy.init_node('filtered_scan_publisher', anonymous=True)

    # RPLidar의 LaserScan 데이터를 구독
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    # 필터링된 데이터를 발행할 Publisher
    global scan_pub
    scan_pub = rospy.Publisher('/scan_filtered', LaserScan, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        laser_scan_subscriber()
    except rospy.ROSInterruptException:
        pass

