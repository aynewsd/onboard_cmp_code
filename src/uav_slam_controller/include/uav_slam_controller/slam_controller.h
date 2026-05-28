#ifndef SLAM_CONTROLLER_H
#define SLAM_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>

// TCP通信与线程相关
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <mutex>
#include <vector>
#include <string>
// JSON解析
#include <jsoncpp/json/json.h>

class SlamController
{
public:
    SlamController(ros::NodeHandle& nh);
    ~SlamController();
    void run();

private:
    // -------------------------- 原有飞控核心回调与函数 --------------------------
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    bool setOffboardMode();
    bool armUAV();
    void publishSetpoint();
    geometry_msgs::PoseStamped compensateWToMavros(const geometry_msgs::PoseStamped& pose) const;
    static geometry_msgs::Quaternion yawToQuaternion(double yaw_rad);
    static geometry_msgs::Quaternion quatMultiply(const geometry_msgs::Quaternion& a, const geometry_msgs::Quaternion& b);
    static geometry_msgs::Quaternion quatNormalize(const geometry_msgs::Quaternion& q);

    // -------------------------- 新增TCP地面站交互相关 --------------------------
    bool startTcpServer();               // 启动TCP服务端（机载为服务端，匹配用户示例）
    void tcpReceiveLoop();                // TCP接收线程主循环（后台运行，不阻塞ROS主循环）
    void parseMessage(const std::string& msg);  // 解析地面站发来的JSON消息
    bool sendTcpMessage(const std::string& msg); // 给地面站发送JSON消息（自动加换行符）
    void sendCurrentPosition(double x, double y); // 周期发送当前位置（匹配用户示例格式）
    void sendWaypointReached(int index, double x, double y); // 发送航点到达通知

    // -------------------------- 新增路径执行相关 --------------------------
    bool isWaypointReached(const geometry_msgs::Point& current_pos, const geometry_msgs::Point& target_pos);
    void executeWaypointMission(); // 航点任务执行逻辑

    // -------------------------- ROS句柄与订阅发布 --------------------------
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher setpoint_pub_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arm_client_;

    // -------------------------- 状态变量 --------------------------
    mavros_msgs::State current_state_;
    nav_msgs::Odometry current_odom_;
    geometry_msgs::PoseStamped target_pose_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // -------------------------- 飞控控制参数 --------------------------
    float takeoff_height_;
    float control_rate_;

    // -------------------------- TCP通信参数与变量 --------------------------
    int tcp_port_;
    int server_fd_;
    int client_fd_;
    bool tcp_running_;
    std::thread tcp_thread_;
    std::mutex tcp_mutex_; // 保护客户端socket的线程安全
    std::string recv_buffer_; // 接收缓冲区，解决TCP粘包问题

    // -------------------------- 路径任务参数与变量 --------------------------
    std::vector<geometry_msgs::Point> waypoint_list_;
    int current_waypoint_index_;
    bool is_mission_active_;
    double waypoint_reach_threshold_; // 航点到达阈值（水平距离，单位：米）
    double position_send_interval_;    // 位置上报周期（单位：秒）
    ros::Time last_position_send_time_;
    std::mutex mission_mutex_; // 保护航点列表的线程安全
};

#endif
