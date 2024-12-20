#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

def publish_joint_state():
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(10)  # 10 Hz
    
    joint_state = JointState()
    joint_state.header = Header()
    joint_state.name = ['wheel_left_joint', 'wheel_right_joint']
    
    while not rospy.is_shutdown():
        # 각도 계산 (예시로 시계방향 회전)
        joint_state.position = [math.sin(rospy.get_time()), math.cos(rospy.get_time())]
        joint_state.header.stamp = rospy.Time.now()
        
        pub.publish(joint_state)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_joint_state()
    except rospy.ROSInterruptException:
        pass

