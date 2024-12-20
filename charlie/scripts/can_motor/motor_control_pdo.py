import os
import rospy
import numpy as np
import canopen
from std_msgs.msg import Int32, Int32MultiArray, Float32
from geometry_msgs.msg import Twist

WHEELSIZE = 6.5  # 인치 단위
WHEELBASE = 0.4  # 미터 단위

class CANopenNode:
    def __init__(self):
        self.network = canopen.Network()

        # EDS 파일 경로 설정
        eds_file_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            'ZLAC8015D_V2.0.eds'
        )
        
        # CANopen 통신 초기화
        self.network.connect(channel='can0', bustype='socketcan', bitrate=500000)
        self.node = canopen.RemoteNode(1, eds_file_path)
        self.network.add_node(self.node)
        rospy.loginfo('CANopen Node initialized with EDS.')

        self.left_velocity = 0
        self.right_velocity = 0

        # 모터 초기 설정
        self.setup_motor()

        # RPDO 전송 속도 100ms
        self.node.rpdo[1].start(0.1)
        # TPDO 수신 콜백 설정
        self.node.tpdo[1].add_callback(self.process_tpdo)

        # Motor Speed sub
        self.motor_velocity_sub = rospy.Subscriber('/cmd_vel', Twist, self.callback_velocity)
        self.left_speed_publisher = rospy.Publisher('left_motor_speed', Int32, queue_size=10)
        self.right_speed_publisher = rospy.Publisher('right_motor_speed', Int32, queue_size=10)
        self.velocity_publisher = rospy.Publisher('robot_velocity', Float32, queue_size=10)

    def callback_velocity(self, msg):
        # msg.data는 배열 형태로 받아옵니다
        wheel_radius = (WHEELSIZE / 2) * 0.0254

        v = msg.linear.x
        w = msg.angular.z

        v_left = v - (WHEELBASE / 2.0) * w
        v_right = v + (WHEELBASE / 2.0) * w

        left_rad_per_sec = v_left / wheel_radius
        right_rad_per_sec = v_right / wheel_radius

        self.left_velocity = (left_rad_per_sec / (2 * np.pi)) * 60
        self.right_velocity = (right_rad_per_sec / (2 * np.pi)) * 60

        rospy.loginfo(f'Received left motor speed: {self.left_velocity}')
        rospy.loginfo(f'Received right motor speed: {self.right_velocity}')

        # PDO 데이터 전송 및 수신 설정
        self.send_pdo_data()

    def setup_motor(self):
        self.node.nmt.state = 'PRE-OPERATIONAL'
        rospy.loginfo('NMT State set to PRE-OPERATIONAL')

        self.set_synchronous_mode(0x00)
        self.set_mode(0x03)  # 속도 모드 설정
        self.set_left_acceleration(100, 100)  # 왼쪽 모터 가속/감속도 설정
        self.set_right_acceleration(100, 100)  # 오른쪽 모터 가속/감속도 설정
        self.motor_activation()
        rospy.loginfo('Motor Active')

        self.node.tpdo.read()
        self.node.rpdo.read()

        self.node.rpdo[1].clear()
        self.node.rpdo[1].add_variable('Target_velocity', 'Left Motor Target_velocity')  # 왼쪽 모터 속도
        self.node.rpdo[1].add_variable('Target_velocity', 'Right Motor Target_velocity')  # 오른쪽 모터 속도
        self.node.rpdo[1].enabled = True

        self.node.tpdo[1].clear()
        self.node.tpdo[1].add_variable('Velocity_actual_value', 'Left Motor Velocity_actual_value')
        self.node.tpdo[1].add_variable('Velocity_actual_value', 'Right Motor Velocity_actual_value')
        self.node.tpdo[1].enabled = True

        self.node.tpdo[1].trans_type = 254  # 비동기식 전송
        self.node.tpdo[1].event_timer = 100  # 50ms마다 TPDO 전송 (100 * 500µs)

        self.node.tpdo.save()
        self.node.rpdo.save()

        self.node.nmt.state = 'OPERATIONAL'
        rospy.loginfo('NMT State set to OPERATIONAL')

    def set_synchronous_mode(self, mode):
        self.node.sdo[0x200F].raw = mode

    def set_mode(self, mode):
        self.node.sdo[0x6060].raw = mode

    def set_left_acceleration(self, accel, decel):
        self.node.sdo[0x6083][1].raw = accel
        self.node.sdo[0x6084][1].raw = decel

    def set_right_acceleration(self, accel, decel):
        self.node.sdo[0x6083][2].raw = accel
        self.node.sdo[0x6084][2].raw = decel

    def send_pdo_data(self):
        rospy.loginfo('Sending PDO Data...')
        self.node.rpdo[1]['Target_velocity.Left Motor Target_velocity'].raw = self.left_velocity
        self.node.rpdo[1]['Target_velocity.Right Motor Target_velocity'].raw = -1 * self.right_velocity

    def process_tpdo(self, msg):
        left_motor_speed = msg['Velocity_actual_value.Left Motor Velocity_actual_value'].raw
        right_motor_speed = msg['Velocity_actual_value.Right Motor Velocity_actual_value'].raw

        left_motor_speed = convert_to_signed_32bit(left_motor_speed)
        right_motor_speed = convert_to_signed_32bit(right_motor_speed)

        vel = (left_motor_speed - right_motor_speed) / 2

        self.left_speed_publisher.publish(Int32(data=left_motor_speed))
        self.right_speed_publisher.publish(Int32(data=-right_motor_speed))

        self.velocity_publisher.publish(Float32(data=vel))

    def motor_activation(self):
        self.node.sdo[0x6040].raw = 0x06
        self.node.sdo[0x6040].raw = 0x07
        self.node.sdo[0x6040].raw = 0x0F

    def motor_stop(self):
        self.node.sdo[0x6040].raw = 0x00
        rospy.loginfo('Motor Stop')

def convert_to_signed_32bit(value):
    if value >= 2**31:
        value -= 2**32
    return value

def main():
    rospy.init_node('canopen_node')
    node = CANopenNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.motor_stop()
        node.network.disconnect()

if __name__ == '__main__':
    main()

