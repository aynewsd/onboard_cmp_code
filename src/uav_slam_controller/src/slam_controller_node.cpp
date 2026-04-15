#include "uav_slam_controller/slam_controller.h"
#include <cmath>

// -------------------------- 构造与析构函数 --------------------------
SlamController::SlamController(ros::NodeHandle& nh) 
: nh_(nh), tf_listener_(tf_buffer_), server_fd_(-1), client_fd_(-1), tcp_running_(false),
  current_waypoint_index_(0), is_mission_active_(false)
{
    // 1. 读取ROS参数
    nh_.param<float>("takeoff_height", takeoff_height_, 2.0);
    nh_.param<float>("control_rate", control_rate_, 20.0);
    nh_.param<int>("tcp_port", tcp_port_, 8888); // 匹配用户示例的8888端口
    nh_.param<double>("waypoint_reach_threshold", waypoint_reach_threshold_, 0.3); // 0.3米到达阈值
    nh_.param<double>("position_send_interval", position_send_interval_, 1.0); // 1秒上报一次位置

    // 2. ROS订阅与发布初始化
    state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &SlamController::stateCallback, this);
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/Odometry", 10, &SlamController::odomCallback, this);
    setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    // 3. ROS服务客户端初始化
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arm_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    // 4. 初始化目标位姿（默认起飞悬停点）
    target_pose_.header.frame_id = "map";
    target_pose_.pose.position.x = 0.0;
    target_pose_.pose.position.y = 0.0;
    target_pose_.pose.position.z = takeoff_height_;
    target_pose_.pose.orientation.w = 1.0;

    // 5. 启动TCP服务端与接收线程
    if(startTcpServer())
    {
        tcp_running_ = true;
        tcp_thread_ = std::thread(&SlamController::tcpReceiveLoop, this);
        ROS_INFO("TCP server started, listening on port %d", tcp_port_);
    }
    else
    {
        ROS_ERROR("Failed to start TCP server!");
    }

    // 6. 初始化位置上报时间
    last_position_send_time_ = ros::Time::now();
}

SlamController::~SlamController()
{
    // 停止TCP线程，关闭socket
    tcp_running_ = false;
    if(client_fd_ >= 0) close(client_fd_);
    if(server_fd_ >= 0) close(server_fd_);
    if(tcp_thread_.joinable()) tcp_thread_.join();
}

// -------------------------- 原有飞控核心回调函数 --------------------------
void SlamController::stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_ = *msg;
    // 安全逻辑：如果飞控退出OFFBOARD模式，立即停止航点任务
    if(current_state_.mode != "OFFBOARD" && is_mission_active_)
    {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        is_mission_active_ = false;
        ROS_WARN("OFFBOARD mode lost, mission stopped!");
    }
}

void SlamController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_odom_ = *msg;
}

// -------------------------- 原有飞控控制核心函数 --------------------------
bool SlamController::setOffboardMode()
{
    mavros_msgs::SetMode mode_msg;
    mode_msg.request.custom_mode = "OFFBOARD";
    return set_mode_client_.call(mode_msg) && mode_msg.response.mode_sent;
}

bool SlamController::armUAV()
{
    mavros_msgs::CommandBool arm_msg;
    arm_msg.request.value = true;
    return arm_client_.call(arm_msg) && arm_msg.response.success;
}

void SlamController::publishSetpoint()
{
    target_pose_.header.stamp = ros::Time::now();
    setpoint_pub_.publish(target_pose_);
}

// -------------------------- 新增TCP服务端启动函数 --------------------------
bool SlamController::startTcpServer()
{
    // 创建TCP socket
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if(server_fd_ < 0)
    {
        ROS_ERROR("Failed to create TCP socket");
        return false;
    }

    // 设置socket选项，允许端口复用
    int opt = 1;
    if(setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
    {
        ROS_ERROR("Failed to set socket options");
        close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    // 绑定端口
    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY; // 监听所有网卡（包括WiFi热点）
    server_addr.sin_port = htons(tcp_port_);

    if(bind(server_fd_, (sockaddr*)&server_addr, sizeof(server_addr)) < 0)
    {
        ROS_ERROR("Failed to bind port %d", tcp_port_);
        close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    // 开始监听
    if(listen(server_fd_, 1) < 0) // 只允许一个地面站客户端连接
    {
        ROS_ERROR("Failed to listen on port %d", tcp_port_);
        close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    return true;
}

// -------------------------- 新增TCP接收线程主循环 --------------------------
void SlamController::tcpReceiveLoop()
{
    sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    char buffer[4096];

    while(tcp_running_ && ros::ok())
    {
        // 等待地面站客户端连接（阻塞式，不影响ROS主循环）
        ROS_INFO("Waiting for ground station client connection...");
        int new_client_fd = accept(server_fd_, (sockaddr*)&client_addr, &client_len);
        if(new_client_fd < 0)
        {
            if(tcp_running_) ROS_WARN("Failed to accept client connection");
            continue;
        }

        // 加锁更新客户端socket
        {
            std::lock_guard<std::mutex> lock(tcp_mutex_);
            if(client_fd_ >= 0) close(client_fd_);
            client_fd_ = new_client_fd;
        }
        ROS_INFO("Ground station connected: %s", inet_ntoa(client_addr.sin_addr));

        // 接收客户端数据循环
        while(tcp_running_ && ros::ok())
        {
            memset(buffer, 0, sizeof(buffer));
            int recv_len = recv(client_fd_, buffer, sizeof(buffer)-1, 0);
            if(recv_len <= 0)
            {
                // 客户端断开连接
                ROS_WARN("Ground station disconnected");
                std::lock_guard<std::mutex> lock(tcp_mutex_);
                close(client_fd_);
                client_fd_ = -1;
                break;
            }

            // 追加到接收缓冲区，解决TCP粘包问题
            recv_buffer_ += std::string(buffer, recv_len);

            // 按换行符分割完整的JSON消息（匹配用户地面站的消息格式）
            size_t newline_pos;
            while((newline_pos = recv_buffer_.find('\n')) != std::string::npos)
            {
                std::string complete_msg = recv_buffer_.substr(0, newline_pos);
                recv_buffer_ = recv_buffer_.substr(newline_pos + 1);

                // 解析完整消息
                if(!complete_msg.empty())
                {
                    parseMessage(complete_msg);
                }
            }
        }
    }
}

// -------------------------- 新增JSON消息解析函数 --------------------------
void SlamController::parseMessage(const std::string& msg)
{
    Json::Reader reader;
    Json::Value root;

    // 解析JSON
    if(!reader.parse(msg, root))
    {
        ROS_WARN("Failed to parse JSON message: %s", reader.getFormattedErrorMessages().c_str());
        return;
    }

    // 检查消息类型
    if(!root.isMember("type"))
    {
        ROS_WARN("Received message has no 'type' field");
        return;
    }
    std::string msg_type = root["type"].asString();

    // 处理路径消息（匹配用户地面站send_path的格式）
    if(msg_type == "path")
    {
        if(!root.isMember("path") || !root["path"].isArray())
        {
            ROS_WARN("Path message has no valid 'path' array");
            return;
        }

        // 解析路径点列表
        std::vector<geometry_msgs::Point> new_waypoints;
        for(int i = 0; i < root["path"].size(); i++)
        {
            Json::Value point_json = root["path"][i];
            if(!point_json.isMember("x") || !point_json.isMember("y"))
            {
                ROS_WARN("Waypoint %d has no x/y field, skipped", i);
                continue;
            }

            geometry_msgs::Point waypoint;
            waypoint.x = point_json["x"].asDouble();
            waypoint.y = point_json["y"].asDouble();
            waypoint.z = takeoff_height_; // z轴固定为起飞高度，可扩展为航点带z值
            new_waypoints.push_back(waypoint);
        }

        // 加锁更新航点任务
        {
            std::lock_guard<std::mutex> lock(mission_mutex_);
            waypoint_list_ = new_waypoints;
            current_waypoint_index_ = 0;
            is_mission_active_ = true;
        }
        ROS_INFO("Received new path with %zu waypoints, mission started", new_waypoints.size());
    }
    // 可扩展：处理地面站其他指令（如暂停、继续、返航等）
    else
    {
        ROS_WARN("Received unknown message type: %s", msg_type.c_str());
    }
}

// -------------------------- 新增TCP消息发送函数 --------------------------
bool SlamController::sendTcpMessage(const std::string& msg)
{
    std::lock_guard<std::mutex> lock(tcp_mutex_);
    if(client_fd_ < 0)
    {
        ROS_WARN("No ground station connected, cannot send message");
        return false;
    }

    // 消息末尾加换行符，匹配用户地面站的解析格式
    std::string msg_with_newline = msg + "\n";
    ssize_t send_len = send(client_fd_, msg_with_newline.c_str(), msg_with_newline.size(), MSG_NOSIGNAL);
    if(send_len != msg_with_newline.size())
    {
        ROS_WARN("Failed to send complete message to ground station");
        return false;
    }
    return true;
}

// -------------------------- 新增位置上报函数（匹配用户示例格式） --------------------------
void SlamController::sendCurrentPosition(double x, double y)
{
    Json::Value root;
    root["type"] = "position";
    root["x"] = x;
    root["y"] = y;

    Json::FastWriter writer;
    std::string msg = writer.write(root);
    // 去掉FastWriter自动加的换行符（我们自己统一加）
    if(!msg.empty() && msg.back() == '\n')
    {
        msg.pop_back();
    }

    sendTcpMessage(msg);
    last_position_send_time_ = ros::Time::now();
}

// -------------------------- 新增航点到达通知函数 --------------------------
void SlamController::sendWaypointReached(int index, double x, double y)
{
    Json::Value root;
    root["type"] = "waypoint_reached";
    root["index"] = index;
    root["x"] = x;
    root["y"] = y;
    root["success"] = true;

    Json::FastWriter writer;
    std::string msg = writer.write(root);
    if(!msg.empty() && msg.back() == '\n')
    {
        msg.pop_back();
    }

    sendTcpMessage(msg);
    ROS_INFO("Waypoint %d reached, notification sent to ground station", index);
}

// -------------------------- 航点到达判断函数 --------------------------
bool SlamController::isWaypointReached(const geometry_msgs::Point& current_pos, const geometry_msgs::Point& target_pos)
{
    // 只计算水平距离（x,y），忽略z轴高度
    double dx = current_pos.x - target_pos.x;
    double dy = current_pos.y - target_pos.y;
    double horizontal_dist = sqrt(dx*dx + dy*dy);
    return horizontal_dist <= waypoint_reach_threshold_;
}

// -------------------------- 航点任务执行函数 --------------------------
void SlamController::executeWaypointMission()
{
    std::lock_guard<std::mutex> lock(mission_mutex_);
    // 检查任务是否有效
    if(!is_mission_active_ || waypoint_list_.empty() || current_waypoint_index_ >= waypoint_list_.size())
    {
        return;
    }

    // 获取当前目标航点
    geometry_msgs::Point target_waypoint = waypoint_list_[current_waypoint_index_];
    // 更新飞控目标位姿
    target_pose_.pose.position.x = target_waypoint.x;
    target_pose_.pose.position.y = target_waypoint.y;
    target_pose_.pose.position.z = target_waypoint.z;

    // 检查是否到达当前航点
    geometry_msgs::Point current_pos = current_odom_.pose.pose.position;
    if(isWaypointReached(current_pos, target_waypoint))
    {
        // 发送到达通知
        sendWaypointReached(current_waypoint_index_, current_pos.x, current_pos.y);
        // 切换到下一个航点
        current_waypoint_index_++;
        // 检查是否所有航点执行完毕
        if(current_waypoint_index_ >= waypoint_list_.size())
        {
            is_mission_active_ = false;
            // 发送任务完成通知
            Json::Value root;
            root["type"] = "mission_complete";
            root["total_waypoints"] = (int)waypoint_list_.size();
            Json::FastWriter writer;
            std::string msg = writer.write(root);
            if(!msg.empty() && msg.back() == '\n') msg.pop_back();
            sendTcpMessage(msg);
            ROS_INFO("All waypoints completed, mission finished!");
        }
    }
}

// -------------------------- 主循环函数 --------------------------
void SlamController::run()
{
    ros::Rate rate(control_rate_);

    // 预发布100次设定点，满足PX4进入OFFBOARD模式的要求
    ROS_INFO("Pre-publishing setpoints...");
    for(int i = 0; i < 100 && ros::ok(); i++)
    {
        publishSetpoint();
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Pre-publish finished, system ready");

    // 主控制循环
    while(ros::ok())
    {
        // 1. 飞控状态检查与模式控制
        if(current_state_.mode != "OFFBOARD")
        {
            if(setOffboardMode()) ROS_INFO("Switched to OFFBOARD mode");
        }
        if(!current_state_.armed)
        {
            if(armUAV()) ROS_INFO("UAV armed successfully");
        }

        // 2. 航点任务执行
        executeWaypointMission();

        // 3. 周期发布飞控设定点（必须>2Hz，否则飞控退出OFFBOARD）
        publishSetpoint();

        // 4. 周期上报当前位置给地面站（匹配用户示例的1秒周期）
        if((ros::Time::now() - last_position_send_time_).toSec() >= position_send_interval_)
        {
            double current_x = current_odom_.pose.pose.position.x;
            double current_y = current_odom_.pose.pose.position.y;
            sendCurrentPosition(current_x, current_y);
        }

        ros::spinOnce();
        rate.sleep();
    }
}

// -------------------------- 主函数 --------------------------
int main(int argc, char**argv)
{
    ros::init(argc, argv, "slam_controller_node");
    ros::NodeHandle nh("~");
    SlamController controller(nh);
    controller.run();
    return 0;
}