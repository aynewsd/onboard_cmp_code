#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>

// --- 全局变量声明 ---
// 用于存储飞控当前状态（是否解锁、当前模式、连接状态等）
mavros_msgs::State current_state;
// 用于存储当前的里程计信息（位置、速度）
nav_msgs::Odometry current_odom;
// 用于存储目标位置指令
geometry_msgs::PoseStamped target_pose;

// 参数变量（通过Launch文件或YAML配置）
double target_x_offset;   // 目标点相对于当前位置的X轴偏移量
double target_y_offset;   // 目标点相对于当前位置的Y轴偏移量
double target_z;          // 目标高度
double position_tolerance;// 到达目标点的位置容忍度（误差半径）
double hover_time;        // 到达目标点后的悬停时间
double timeout;           // 飞行任务超时时间（超时自动降落）
double publish_rate;      // 指令发布频率
std::string odom_topic;   // 里程计话题名称（可配置为FastLIO输出的话题）

// --- 回调函数 ---

// 【回调】更新飞控状态
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

// 【回调】更新里程计数据（可能来自GPS或FastLIO等视觉里程计）
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    current_odom = *msg;
}

// --- 工具函数 ---

// 计算当前位置与目标位置之间的直线距离（欧几里得距离）
double get_position_error() {
    double dx = target_pose.pose.position.x - current_odom.pose.pose.position.x;
    double dy = target_pose.pose.position.y - current_odom.pose.pose.position.y;
    double dz = target_pose.pose.position.z - current_odom.pose.pose.position.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

// --- 主程序 ---

int main(int argc, char **argv) {
    // 1. 初始化ROS节点
    ros::init(argc, argv, "fastlio_px4_waypoint");
    // 使用私有句柄 "~" 以便方便地从launch文件读取参数
    ros::NodeHandle nh("~");

    // 2. 加载参数（如果参数服务器没有提供，则使用后面的默认值）
    nh.param<double>("target_x_offset", target_x_offset, 2.0);
    nh.param<double>("target_y_offset", target_y_offset, 0.0);
    nh.param<double>("target_z", target_z, 1.0);
    nh.param<double>("position_tolerance", position_tolerance, 0.2);
    nh.param<double>("hover_time", hover_time, 5.0);
    nh.param<double>("timeout", timeout, 30.0);
    nh.param<double>("publish_rate", publish_rate, 20.0);
    nh.param<std::string>("odom_topic", odom_topic, "/mavros/local_position/odom");

    ROS_INFO("Parameters loaded:");
    ROS_INFO("Target: Current + (%.1f, %.1f, %.1f) m", target_x_offset, target_y_offset, target_z);
    ROS_INFO("Tolerance: %.1f m", position_tolerance);

    // 3. 初始化ROS通信对象（订阅者、发布者、服务客户端）
    // 订阅飞控状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    // 订阅里程计（注意这里使用了参数 odom_topic，可以切换为FastLIO的输出）
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 10, odom_cb);
    // 发布位置设定点给飞控
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    // 服务客户端：解锁/上锁
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    // 服务客户端：切换飞行模式
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // 控制循环频率
    ros::Rate rate(publish_rate);

    // 4. 等待与飞控建立连接
    ROS_INFO("Waiting for PX4...");
    while(ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("PX4 Connected!");

    // 5. 等待里程计数据就绪
    ROS_INFO("Waiting for odometry...");
    while(ros::ok() && current_odom.header.stamp.isZero()) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Odometry OK!");

    // 6. 设定目标点
    // 逻辑：在当前位置的基础上加上偏移量
    target_pose.pose.position.x = current_odom.pose.pose.position.x + target_x_offset;
    target_pose.pose.position.y = current_odom.pose.pose.position.y + target_y_offset;
    target_pose.pose.position.z = target_z;
    target_pose.pose.orientation.w = 1.0; // 朝向默认（四元数）

    ROS_INFO("Target: (%.2f, %.2f, %.2f)", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);

    // 7. 进入OFFBOARD模式前的准备
    // PX4规定：进入OFFBOARD模式前，必须已经持续收到设定点指令（通常要求1秒以上）
    // 这里先发送100次空指令
    for(int i=0; i<100 && ros::ok(); i++) {
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }

    // 8. 切换模式为 OFFBOARD
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("OFFBOARD Enabled");
    } else {
        ROS_ERROR("Failed to enter OFFBOARD");
        return -1; // 切换失败则退出程序
    }

    // 9. 解锁电机 (Arm)
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if(arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Armed!");
    } else {
        ROS_ERROR("Failed to arm");
        return -1; // 解锁失败则退出程序
    }

    // 10. 主控制循环
    ROS_INFO("Starting flight...");
    ros::Time start_time = ros::Time::now(); // 记录开始时间，用于超时判断
    
    while(ros::ok()) {
        // 持续发布目标位置指令（这是必须的，一旦断流飞控可能会退出OFFBOARD）
        local_pos_pub.publish(target_pose);

        // 计算当前误差
        double error = get_position_error();
        ROS_INFO_THROTTLE(1.0, "Error: %.2f m", error); // 1秒打印一次，避免刷屏

        // 检查是否到达目标点
        if(error < position_tolerance) {
            ROS_INFO("Waypoint Reached!");
            ros::Duration(hover_time).sleep(); // 在目标点悬停指定时间
            ROS_INFO("Landing...");

            // 切换到自动降落模式
            mavros_msgs::SetMode land_set_mode;
            land_set_mode.request.custom_mode = "LAND";
            set_mode_client.call(land_set_mode);
            break; // 跳出主循环
        }

        // 检查是否超时（防止飞丢或卡死）
        if(ros::Time::now() - start_time > ros::Duration(timeout)) {
            ROS_ERROR("Timeout! Landing...");
            // 超时紧急降落
            mavros_msgs::SetMode land_set_mode;
            land_set_mode.request.custom_mode = "LAND";
            set_mode_client.call(land_set_mode);
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    // 11. 结束程序前的等待
    // 等待飞机落地并自动上锁（Disarm）
    ROS_INFO("Waiting for disarm...");
    while(ros::ok() && current_state.armed) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Done!");

    return 0;
}
