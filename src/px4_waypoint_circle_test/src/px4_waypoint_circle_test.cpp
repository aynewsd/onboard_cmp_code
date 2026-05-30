#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <numeric>
#include <string>
#include <vector>

// ============================================================
// 说明
// ============================================================
// 本节点用于 PX4 + MAVROS 的 OFFBOARD 位置控制测试，流程尽量精简：
// 1) 进入 OFFBOARD 并解锁
// 2) 飞到圆周起点：(circle_radius, 0, circle_z)
// 3) 以固定世界系圆心 (0, 0, circle_z)，半径 circle_radius 绕圈飞行 N 圈
// 4) 输出每圈耗时与平均每圈耗时，然后切 AUTO.LAND
//
// 注意：
// - PX4 进入 OFFBOARD 前必须先持续收到 setpoint（通常 >= 1s 且频率 >= 2Hz）。
// - 本示例采用：预发送 setpoint -> ARM -> 循环请求 OFFBOARD，直到 mode==OFFBOARD。
// - 里程计默认使用 /Odometry，即工程统一后的 W 世界系：x前、y左、z上。
// - 发布到 /mavros/setpoint_position/local 前仍保留航点补偿：
//   W -> MAVROS输入中间系M：x'=-y, y'=x, z'=z。
//   MAVROS 后续会隐含把该输入转换到 PX4 内部坐标，不能把 W 系航点原样发给 MAVROS。
//
// 代码风格参考 fastlio_px4_waypoint.cpp：使用全局变量保存 state/odom。
// ============================================================

// -----------------------------
// 全局状态（与 fastlio_px4_waypoint.cpp 保持一致的简洁写法）
// -----------------------------
mavros_msgs::State g_state;        // 飞控状态：连接/模式/是否解锁等
nav_msgs::Odometry g_odom;         // 当前里程计（W世界系：x前、y左、z上）
bool g_odom_ready = false;         // 里程计是否已就绪（收到过非零时间戳）

// Callbacks
void stateCb(const mavros_msgs::State::ConstPtr& msg)
{
  // 保存飞控状态（connected/mode/armed...）
  g_state = *msg;
}

void odomCb(const nav_msgs::Odometry::ConstPtr& msg)
{
  // 保存里程计（位置/姿态/速度...）
  g_odom = *msg;
  // 以时间戳是否为 0 判断是否已有有效数据
  g_odom_ready = !g_odom.header.stamp.isZero();
}

// 计算当前位置与目标位置之间的 3D 欧氏距离（用于“到点判断”）
double positionError(const geometry_msgs::PoseStamped& target)
{
  const double dx = target.pose.position.x - g_odom.pose.pose.position.x;
  const double dy = target.pose.position.y - g_odom.pose.pose.position.y;
  const double dz = target.pose.position.z - g_odom.pose.pose.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// 发布 setpoint（每次发布都刷新 header.stamp）
// - PX4 OFFBOARD 要求：持续流式发送 setpoint，一旦断流可能退出 OFFBOARD
void publishSetpoint(ros::Publisher& pub, geometry_msgs::PoseStamped& sp)
{
  // 先更新时间戳（保持内部sp仍用W系表达）
  sp.header.stamp = ros::Time::now();
  sp.header.frame_id = "map";

  // 坐标补偿：W -> MAVROS期望local（抵消顺时针90°偏转）
  geometry_msgs::PoseStamped out = sp;
  const double x = sp.pose.position.x;
  const double y = sp.pose.position.y;
  out.pose.position.x = -y;
  out.pose.position.y = x;
  out.pose.position.z = sp.pose.position.z;

  // 姿态同样补偿：左乘 +90° yaw
  tf2::Quaternion q_in(sp.pose.orientation.x, sp.pose.orientation.y, sp.pose.orientation.z, sp.pose.orientation.w);
  tf2::Quaternion q_rot;
  q_rot.setRPY(0.0, 0.0, M_PI_2);
  tf2::Quaternion q_out = q_rot * q_in;
  q_out.normalize();
  out.pose.orientation.x = q_out.x();
  out.pose.orientation.y = q_out.y();
  out.pose.orientation.z = q_out.z();
  out.pose.orientation.w = q_out.w();

  pub.publish(out);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "px4_waypoint_circle_test");
  ros::NodeHandle nh("~");

  // -----------------------------
  // 参数（全部可通过 YAML/launch 覆盖）
  // -----------------------------
  std::string odom_topic = "/Odometry";
  double publish_rate = 20.0;
  double position_tolerance = 0.25;
  double circle_z = 1.4;
  double circle_radius = 1.0;
  int circle_laps = 10;
  double circle_period = 10.0;
  double timeout = 300.0;

  // 从参数服务器读取（如果没有则用默认值）
  nh.param<std::string>("odom_topic", odom_topic, odom_topic);
  nh.param<double>("publish_rate", publish_rate, publish_rate);
  nh.param<double>("position_tolerance", position_tolerance, position_tolerance);
  nh.param<double>("circle_z", circle_z, circle_z);
  nh.param<double>("circle_radius", circle_radius, circle_radius);
  nh.param<int>("circle_laps", circle_laps, circle_laps);
  nh.param<double>("circle_period", circle_period, circle_period);
  nh.param<double>("timeout", timeout, timeout);

  ROS_INFO("Params: odom=%s rate=%.1fHz tol=%.2fm", odom_topic.c_str(), publish_rate, position_tolerance);
  ROS_INFO("Circle center=(0.00, 0.00, %.2f), r=%.2fm laps=%d period=%.2fs",
           circle_z, circle_radius, circle_laps, circle_period);

  // -----------------------------
  // ROS 通信：订阅 state/odom，发布 setpoint，调用解锁与模式切换服务
  // -----------------------------
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, stateCb);
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 50, odomCb);
  ros::Publisher sp_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 20);

  ros::ServiceClient arm_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  ros::ServiceClient mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

  ros::Rate rate(publish_rate);

  // -----------------------------
  // 等待：飞控连接 & 里程计就绪
  // -----------------------------
  ROS_INFO("Waiting for FCU connection...");
  while (ros::ok() && !g_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("FCU connected.");

  ROS_INFO("Waiting for odometry on %s ...", odom_topic.c_str());
  while (ros::ok() && !g_odom_ready)
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Odometry ready.");

  // -----------------------------
  // 定义关键 setpoint（坐标系：W，x前、y左、z上）
  // 说明：
  // - 圆心固定为 W 系 (0,0,circle_z)，不再使用“相对起飞点”的航点。
  // - 圆周起点固定为 W 系 (circle_radius,0,circle_z)。
  // - 实际发布前由 publishSetpoint() 统一做 W -> MAVROS输入中间系M 的航点补偿。
  // -----------------------------
  const double circle_center_x = 0.0;
  const double circle_center_y = 0.0;
  const double circle_center_z = circle_z;

  geometry_msgs::PoseStamped sp_circle_start;
  sp_circle_start.pose.position.x = circle_center_x + circle_radius;
  sp_circle_start.pose.position.y = circle_center_y;
  sp_circle_start.pose.position.z = circle_center_z;
  sp_circle_start.pose.orientation.w = 1.0;

  // 当前指令 setpoint（先用圆周起点作为初始值）
  geometry_msgs::PoseStamped sp_cmd = sp_circle_start;

  // -----------------------------
  // OFFBOARD 前置要求：预发送 setpoint
  // PX4 规定：进入 OFFBOARD 之前必须已经收到一段时间的 setpoint 流
  // 这里默认预发送约 2 秒（publish_rate*2），且至少 20 次
  // -----------------------------
  ROS_INFO("Pre-streaming setpoints...");
  const int pre_count = static_cast<int>(std::max(20.0, publish_rate * 2.0));  // at least ~2 seconds
  for (int i = 0; ros::ok() && i < pre_count; ++i)
  {
    publishSetpoint(sp_pub, sp_cmd);
    ros::spinOnce();
    rate.sleep();
  }

  // -----------------------------
  // 解锁（ARM）与切换 OFFBOARD
  // 说明：
  // - 有的 PX4 配置下“先 ARM 再 OFFBOARD”更稳定（尤其是仿真/无 RC 场景）
  // - 切换 OFFBOARD 需要持续发布 setpoint，否则会被拒绝或快速退出
  // - 因此这里采用：持续发布 setpoint + 每秒请求一次 OFFBOARD，直到 state.mode=="OFFBOARD"
  // -----------------------------
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  if (!arm_client.call(arm_cmd) || !arm_cmd.response.success)
  {
    ROS_ERROR("Failed to arm.");
    return 1;
  }
  ROS_INFO("ARMED.");

  ROS_INFO("Requesting OFFBOARD...");
  mavros_msgs::SetMode offb;
  offb.request.custom_mode = "OFFBOARD";
  ros::Time last_mode_req = ros::Time(0);

  while (ros::ok() && g_state.mode != "OFFBOARD")
  {
    // 持续发 setpoint（不能断）
    publishSetpoint(sp_pub, sp_cmd);

    // 以 1Hz 请求模式，避免刷服务
    if (last_mode_req.isZero() || (ros::Time::now() - last_mode_req).toSec() > 1.0)
    {
      (void)mode_client.call(offb);
      last_mode_req = ros::Time::now();
    }

    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("OFFBOARD enabled.");

  // -----------------------------
  // 任务阶段（最小状态机）
  // GOTO_START: 飞到圆周起点
  // CIRCLE: 以固定圆心 (0,0,circle_z) 绕 circle_laps 圈
  // LAND: 切 AUTO.LAND
  // -----------------------------
  enum class Stage
  {
    GOTO_START,
    CIRCLE,
    LAND,
    DONE
  };

  Stage stage = Stage::GOTO_START;
  ros::Time mission_start = ros::Time::now();
  ros::Time stage_start = mission_start;

  // 绕圈计时统计
  // omega = 2*pi / circle_period  (rad/s)
  const double omega = 2.0 * M_PI / std::max(0.1, circle_period);  // rad/s
  ros::Time circle_start;
  ros::Time last_lap_time;
  int completed_laps = 0;
  std::vector<double> lap_times;

  ROS_INFO("Mission start.");

  while (ros::ok())
  {
    // 全局超时保护：避免卡死一直飞
    if (stage != Stage::CIRCLE && (ros::Time::now() - mission_start).toSec() > timeout)
    {
      ROS_ERROR("Timeout, switching to AUTO.LAND.");
      stage = Stage::LAND;
    }

    // 核心原则：以固定频率持续发布 setpoint（OFFBOARD 必需）
    switch (stage)
    {
      case Stage::GOTO_START:
      {
        // 目标：直接飞到圆周起点 (r,0,circle_z)，后续才开始计圈。
        sp_cmd = sp_circle_start;
        publishSetpoint(sp_pub, sp_cmd);

        const double err = positionError(sp_circle_start);
        ROS_INFO_THROTTLE(1.0, "GOTO_START err=%.2f", err);
        ROS_INFO_THROTTLE(1.0, "Position: x=%.2f, y=%.2f, z=%.2f",g_odom.pose.pose.position.x,g_odom.pose.pose.position.y,g_odom.pose.pose.position.z);
        if (err < position_tolerance)
        {
          ROS_INFO("Circle start reached.");
          stage = Stage::CIRCLE;
          stage_start = ros::Time::now();
          circle_start = stage_start;
          last_lap_time = stage_start;
          completed_laps = 0;
          lap_times.clear();
        }
        break;
      }
      case Stage::CIRCLE:
      {
        // 绕圈：圆心固定为 W 系 (0,0,circle_z)，半径 circle_radius。
        // 圆周参数方程（W）：
        //   x = cx + r*cos(theta)
        //   y = cy + r*sin(theta)
        // theta 随时间增长：theta = omega * t
        const double t = (ros::Time::now() - circle_start).toSec();
        const double theta = omega * t;

        // 生成圆周上的目标点
        sp_cmd = sp_circle_start;
        sp_cmd.pose.position.x = circle_center_x + circle_radius * std::cos(theta);
        sp_cmd.pose.position.y = circle_center_y + circle_radius * std::sin(theta);
        sp_cmd.pose.position.z = circle_center_z;
        publishSetpoint(sp_pub, sp_cmd);

        // 计算圈数与每圈用时
        // - lap_index = floor(theta / 2pi)
        // - 当 lap_index 增加时，认为完成了一圈
        const int lap_index = static_cast<int>(std::floor(theta / (2.0 * M_PI)));
        while (completed_laps < std::min(lap_index, circle_laps))
        {
          const double lap_dt = (ros::Time::now() - last_lap_time).toSec();
          lap_times.push_back(lap_dt);
          completed_laps++;
          last_lap_time = ros::Time::now();
          ROS_INFO("Lap %d time: %.3f s", completed_laps, lap_dt);
        }

        if (completed_laps >= circle_laps)
        {
          // 输出平均每圈耗时
          const double sum = std::accumulate(lap_times.begin(), lap_times.end(), 0.0);
          const double avg = (lap_times.empty()) ? 0.0 : (sum / lap_times.size());
          ROS_INFO("Circle done. Average lap time: %.3f s (laps=%zu)", avg, lap_times.size());
          stage = Stage::LAND;
          stage_start = ros::Time::now();
        }

        break;
      }
      case Stage::LAND:
      {
        // 降落：一边继续发布最后的 setpoint（保持 OFFBOARD 输出不中断）
        // 一边请求 PX4 进入 AUTO.LAND（PX4 模式名为 "AUTO.LAND"）
        publishSetpoint(sp_pub, sp_cmd);

        mavros_msgs::SetMode land;
        land.request.custom_mode = "AUTO.LAND";
        (void)mode_client.call(land);

        // 一旦飞控自动上锁（armed=false）则认为结束
        if (!g_state.armed)
        {
          ROS_INFO("Disarmed, DONE.");
          stage = Stage::DONE;
        }
        break;
      }
      case Stage::DONE:
      default:
        break;
    }

    ros::spinOnce();
    rate.sleep();

    if (stage == Stage::DONE) break;
  }

  return 0;
}
