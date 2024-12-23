import rospy
import tf
from geometry_msgs.msg import TransformStamped
import tf.transformations as tft

def send_camera_odom_to_map_to_base_transform():
    br = tf.TransformBroadcaster()
    
    # 예시: 카메라의 위치 및 회전 (카메라 기준)
    camera_x = 1.0  # 예시값
    camera_y = 0.0  # 예시값
    camera_z = 0.0  # 예시값
    camera_yaw = 0.0  # 예시값 (회전)

    # 카메라의 위치 및 회전 변환 (quaternion으로 변환)
    quaternion_camera = tft.quaternion_from_euler(0, 0, camera_yaw)
    
    # 1. 'map' → 'camera_odom_frame' 변환
    br.sendTransform((camera_x, camera_y, camera_z),
                     quaternion_camera,
                     rospy.Time.now(),
                     "camera_odom_frame",  # 자식 프레임
                     "map")  # 부모 프레임
    
    # 2. 'camera_odom_frame' → 'odom' 변환 (카메라 기준 오도메트리)
    br.sendTransform((camera_x, camera_y, camera_z),
                     quaternion_camera,
                     rospy.Time.now(),
                     "odom",  # 자식 프레임
                     "camera_odom_frame")  # 부모 프레임

    # 3. 'odom' → 'base_footprint' 변환 (로봇의 실제 위치와 회전)
    br.sendTransform((camera_x, camera_y, camera_z),
                     quaternion_camera,
                     rospy.Time.now(),
                     "base_footprint",  # 자식 프레임
                     "odom")  # 부모 프레임

    # 추가: 조인트 및 링크 변환
    # 'base_link' → 'wheel_left_link' 변환
    wheel_left_x = 0.0
    wheel_left_y = 0.2  # 예시값
    wheel_left_z = 0.1  # 예시값
    wheel_left_roll = -1.57  # 회전 (rpy)

    quaternion_wheel_left = tft.quaternion_from_euler(wheel_left_roll, 0, 0)

    br.sendTransform((wheel_left_x, wheel_left_y, wheel_left_z),
                     quaternion_wheel_left,
                     rospy.Time.now(),
                     "wheel_left_link",  # 자식 프레임
                     "base_link")  # 부모 프레임
    # 추가: wheel_right_joint 변환
    # 'base_link' → 'wheel_right_link' 변환
    wheel_right_x = 0.0
    wheel_right_y = -0.35  # 예시값
    wheel_right_z = 0.1  # 예시값
    wheel_right_roll = -1.57  # 회전 (rpy)

    quaternion_wheel_right = tft.quaternion_from_euler(wheel_right_roll, 0, 0)

    br.sendTransform((wheel_right_x, wheel_right_y, wheel_right_z),
                     quaternion_wheel_right,
                     rospy.Time.now(),
                     "wheel_right_link",  # 자식 프레임
                     "base_link")  # 부모 프레임

if __name__ == "__main__":
    rospy.init_node("camera_odom_to_map_to_base_transformer")
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        send_camera_odom_to_map_to_base_transform()
        rate.sleep()

