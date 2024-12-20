import rospy
from nav_msgs.msg import Odometry

# 전역 변수 초기화
global odom_pub

def odom_callback(msg):
    """
    /camera/odom/sample 토픽에서 데이터를 받아 /odom으로 재발행
    """
    # 동일한 Odometry 메시지를 /odom으로 발행
    odom_pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("odom_republisher")

    # /odom 토픽 발행자
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    # /camera/odom/sample 토픽 구독자
    rospy.Subscriber('/camera/odom/sample', Odometry, odom_callback)

    rospy.spin()

