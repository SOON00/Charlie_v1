import rospy
from nav_msgs.msg import Odometry
import tf

# 전역 변수 초기화
global odom_pub
global tf_broadcaster

def odom_callback(msg):
    """
    /camera/odom/sample 토픽에서 데이터를 받아 /odom으로 재발행하고
    odom->base_footprint 변환을 퍼블리시
    """
    # 동일한 Odometry 메시지를 /odom으로 발행
    odom_pub.publish(msg)

    # odom->base_footprint 변환 계산
    transform = (
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z
    )
    rotation = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    )

    # TF 브로드캐스터를 통해 odom->base_footprint 변환 퍼블리시
    tf_broadcaster.sendTransform(
        transform,  # 위치
        rotation,   # 회전 (쿼터니언)
        msg.header.stamp,  # 메시지의 시간 스탬프 사용
        "base_footprint",  # child_frame_id
        "odom"             # parent_frame_id
    )

if __name__ == "__main__":
    rospy.init_node("odom_republisher")

    # /odom 토픽 발행자
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

    # /camera/odom/sample 토픽 구독자
    rospy.Subscriber('/camera/odom/sample', Odometry, odom_callback)

    # TF 브로드캐스터 초기화
    tf_broadcaster = tf.TransformBroadcaster()

    rospy.spin()

