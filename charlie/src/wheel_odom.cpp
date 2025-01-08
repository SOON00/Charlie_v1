#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <Eigen/Dense>

class OdomClass
{
public:
    OdomClass()
    {
        // ROS 노드 초기화
        ros::NodeHandle nh;
        ros::Time::init();

        // 로봇의 기본 파라미터
        wheels_separation = 0.48;  // 휠 간 거리 (m)
        wheels_radius = 0.085;  // 휠 반지름 (m)

        // 상태 변수들
        robot_pose = Eigen::Vector3d(0.0, 0.0, 0.0);  // 로봇의 위치 (x, y, theta)
        robot_vel = Eigen::Vector3d(0.0, 0.0, 0.0);  // 로봇의 속도 (linear, angular)
        last_time = ros::Time::now();  // 시간 초기화

        // Publisher 및 Subscriber
        odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
        joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
        tf_broadcaster = new tf::TransformBroadcaster();
        cmd_vel_sub = nh.subscribe("cmd_vel", 10, &OdomClass::cmdVelCallback, this);

    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        // cmd_vel 토픽을 구독하여 로봇의 속도를 받아옴
        double linear_velocity = -0.01 * msg->linear.x;
        double angular_velocity = -0.01 * msg->angular.z;
        ros::Time current_time = ros::Time::now();
        double duration = (current_time - last_time).toSec();

        calculateOdometry(linear_velocity, angular_velocity, duration);
        last_time = current_time;  // 시간 갱신
    }

    void calculateOdometry(double linear_velocity, double angular_velocity, double duration)
    {
        // 속도 데이터를 기반으로 오도메트리 계산
        double left_wheel_vel = linear_velocity - (angular_velocity * wheels_separation / 2.0);
        double right_wheel_vel = linear_velocity + (angular_velocity * wheels_separation / 2.0);

        // 이동거리와 회전각 계산
        double delta_s = linear_velocity * duration;
        double delta_theta = angular_velocity * duration;

        // 로봇의 위치 갱신
        robot_pose[0] += delta_s * cos(robot_pose[2] + (delta_theta / 2.0));
        robot_pose[1] += delta_s * sin(robot_pose[2] + (delta_theta / 2.0));
        robot_pose[2] += delta_theta;

        // 속도 업데이트
        robot_vel[0] = linear_velocity;
        robot_vel[2] = angular_velocity;

        // 휠 회전 각도 계산 (이동한 거리 / 휠 반지름)
        diff_joint_positions[0] += left_wheel_vel * duration / wheels_radius;
        diff_joint_positions[1] += right_wheel_vel * duration / wheels_radius;
    }

    void publish()
    {
        ros::Time current_time = ros::Time::now();

        // 오도메트리 메시지 생성
        nav_msgs::Odometry odom_msg;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";
        odom_msg.header.stamp = current_time;

        // 로봇의 위치 업데이트
        odom_msg.pose.pose.position.x = robot_pose[0];
        odom_msg.pose.pose.position.y = -robot_pose[1];
        odom_msg.pose.pose.position.z = 0.0;

        // 로봇의 orientation (회전각도) 업데이트
        tf::Quaternion q = tf::createQuaternionFromYaw(robot_pose[2]);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = -q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        // 속도 정보 업데이트
        odom_msg.twist.twist.linear.x = robot_vel[0];
        odom_msg.twist.twist.angular.z = robot_vel[2];

        // TF 변환 정보 업데이트
        geometry_msgs::TransformStamped odom_tf;
        odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
        odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
        odom_tf.transform.translation.z = odom_msg.pose.pose.position.z;
        odom_tf.transform.rotation = odom_msg.pose.pose.orientation;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_footprint";
        odom_tf.header.stamp = current_time;

        // TF 브로드캐스트를 위한 sendTransform 호출
        tf_broadcaster->sendTransform(odom_tf);

        // JointState 퍼블리시
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.header.stamp = current_time;
        joint_state_msg.name = {"wheel_left_joint", "wheel_right_joint"};
        joint_state_msg.position = {0.0, 0.0};  // 휠의 회전 각도 고정
        joint_state_msg.velocity = {0.0, 0.0};  // 휠 속도 고정
        joint_state_msg.effort = {0.0, 0.0};  // Effort는 0으로 설정

        joint_state_pub.publish(joint_state_msg);

        // 퍼블리시
        odom_pub.publish(odom_msg);
    }

private:
    ros::Publisher odom_pub;
    ros::Publisher joint_state_pub;
    ros::Subscriber cmd_vel_sub;
    tf::TransformBroadcaster* tf_broadcaster;

    double wheels_separation;
    double wheels_radius;
    Eigen::Vector3d robot_pose;
    Eigen::Vector3d robot_vel;
    Eigen::Vector2d diff_joint_positions;
    ros::Time last_time;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "leo_odometry");
    ros::NodeHandle nh;
    ros::Rate rate(10);  // 10Hz frequency
    OdomClass odom_node;
    // 주기적으로 odom 메시지를 퍼블리시
    while (ros::ok())
    {
        odom_node.publish();
        rate.sleep();  // 10Hz 주기로 퍼블리시
        ros::spinOnce();
    }
    return 0;
}

