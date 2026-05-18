#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>

// --- ȫ�ֱ��� ---
mavros_msgs::State current_state;
nav_msgs::Odometry current_odom;
geometry_msgs::PoseStamped target_pose;
geometry_msgs::PoseStamped current_enu_pose;  // ENU ��ǰλ�ˣ����ø� PX4��

double target_x_offset;
double target_y_offset;
double target_z;
double position_tolerance;
double hover_time;
double timeout;
double publish_rate;
std::string odom_topic;

ros::Publisher vision_pose_pub;  // ������������ PX4 ���Ӿ���λ

// --- �ص� ---

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    current_odom = *msg;

    current_enu_pose.header = msg->header;
    current_enu_pose.header.frame_id = "map";
    current_enu_pose.pose = msg->pose.pose;
    vision_pose_pub.publish(current_enu_pose);
}

double get_position_error() {
    double dx = target_pose.pose.position.x - current_enu_pose.pose.position.x;
    double dy = target_pose.pose.position.y - current_enu_pose.pose.position.y;
    double dz = target_pose.pose.position.z - current_enu_pose.pose.position.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "fastlio_px4_waypoint");
    ros::NodeHandle nh("~");

    nh.param<double>("target_x_offset", target_x_offset, 0.0);
    nh.param<double>("target_y_offset", target_y_offset, 0.5);
    nh.param<double>("target_z", target_z, 1.0);
    nh.param<double>("position_tolerance", position_tolerance, 0.2);
    nh.param<double>("hover_time", hover_time, 5.0);
    nh.param<double>("timeout", timeout, 30.0);
    nh.param<double>("publish_rate", publish_rate, 20.0);
    nh.param<std::string>("odom_topic", odom_topic, "/Odometry");

    // ����������ʼ���Ӿ���λ������
    vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 10, odom_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    ros::Rate rate(publish_rate);

    ROS_INFO("Waiting for PX4...");
    while(ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("PX4 Connected!");

    ROS_INFO("Waiting for odometry...");
    while(ros::ok() && current_odom.header.stamp.isZero()) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Odometry OK!");

    // Ŀ��㣨ENU��
    target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Target: (%.2f, %.2f, %.2f)", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);

    for(int i=0; i<100 && ros::ok(); i++) {
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("OFFBOARD Enabled");
    } else {
        ROS_ERROR("Failed to enter OFFBOARD");
        return -1;
    }

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if(arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Armed!");
    } else {
        ROS_ERROR("Failed to arm");
        return -1;
    }

    ROS_INFO("Starting flight...");
    ros::Time start_time = ros::Time::now();

    while(ros::ok()) {
        local_pos_pub.publish(target_pose);

        double error = get_position_error();
        ROS_INFO_THROTTLE(1.0, 
        "Error: %.2f m | Radar: (%.2f, %.2f, %.2f) | FCU: (%.2f, %.2f, %.2f)",
        error,
        current_odom.pose.pose.position.x, 
        current_odom.pose.pose.position.y, 
        current_odom.pose.pose.position.z,
        current_enu_pose.pose.position.x, 
        current_enu_pose.pose.position.y, 
        current_enu_pose.pose.position.z);

        if(error < position_tolerance) {
            ROS_INFO("Waypoint Reached!");
            ros::Duration(hover_time).sleep();
            ROS_INFO("Landing...");

            mavros_msgs::SetMode land_set_mode;
            land_set_mode.request.custom_mode = "LAND";
            set_mode_client.call(land_set_mode);
            break;
        }

        if(ros::Time::now() - start_time > ros::Duration(timeout)) {
            ROS_ERROR("Timeout! Landing...");
            mavros_msgs::SetMode land_set_mode;
            land_set_mode.request.custom_mode = "LAND";
            set_mode_client.call(land_set_mode);
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Waiting for disarm...");
    while(ros::ok() && current_state.armed) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Done!");

    return 0;
}