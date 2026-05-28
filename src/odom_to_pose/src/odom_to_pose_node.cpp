#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <string>

class OdomToPose
{
public:
  explicit OdomToPose(ros::NodeHandle& nh) : nh_(nh)
  {
    nh_.param<std::string>("input_topic", input_topic_, input_topic_);
    nh_.param<std::string>("output_odometry_topic", output_odometry_topic_, output_odometry_topic_);
    nh_.param<std::string>("output_vision_topic", output_vision_topic_, output_vision_topic_);
    nh_.param<std::string>("output_frame_id", output_frame_id_, output_frame_id_);
    nh_.param<bool>("yaw_only", yaw_only_, yaw_only_);

    // 雷达安装外参（雷达坐标系下，机体原点的位置）
    nh_.param<double>("body_in_lidar_x", body_in_lidar_x_, body_in_lidar_x_);
    nh_.param<double>("body_in_lidar_y", body_in_lidar_y_, body_in_lidar_y_);
    nh_.param<double>("body_in_lidar_z", body_in_lidar_z_, body_in_lidar_z_);

    // 雷达坐标系 -> 机体坐标系的固定偏航（默认 180deg）
    nh_.param<double>("lidar_to_body_yaw_rad", lidar_to_body_yaw_rad_, lidar_to_body_yaw_rad_);

    sub_ = nh_.subscribe<nav_msgs::Odometry>(input_topic_, 50, &OdomToPose::callback, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(output_odometry_topic_, 50);
    vision_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(output_vision_topic_, 50);

    ROS_INFO("odom_to_pose: %s -> %s + %s", input_topic_.c_str(), output_odometry_topic_.c_str(), output_vision_topic_.c_str());
  }

private:
  static geometry_msgs::Quaternion toMsg(const tf2::Quaternion& q)
  {
    geometry_msgs::Quaternion msg;
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();
    return msg;
  }

  static tf2::Quaternion fromMsg(const geometry_msgs::Quaternion& q)
  {
    return tf2::Quaternion(q.x, q.y, q.z, q.w);
  }

  static tf2::Quaternion yawQuat(double yaw_rad)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_rad);
    q.normalize();
    return q;
  }

  static geometry_msgs::Quaternion yawOnly(const geometry_msgs::Quaternion& q_in)
  {
    tf2::Quaternion q = fromMsg(q_in);
    tf2::Matrix3x3 m(q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    m.getRPY(roll, pitch, yaw);
    return toMsg(yawQuat(yaw));
  }

  static geometry_msgs::Quaternion multiply(const geometry_msgs::Quaternion& a,
                                            const geometry_msgs::Quaternion& b)
  {
    geometry_msgs::Quaternion out;
    out.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    out.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    out.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    out.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return out;
  }

  static geometry_msgs::Quaternion normalize(const geometry_msgs::Quaternion& q)
  {
    const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (n < 1e-12) return q;
    geometry_msgs::Quaternion out = q;
    out.x /= n;
    out.y /= n;
    out.z /= n;
    out.w /= n;
    return out;
  }

  static geometry_msgs::PoseStamped poseFromOdom(const nav_msgs::Odometry& odom)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    return pose;
  }

  geometry_msgs::PoseStamped fastlioToWorldPose(const nav_msgs::Odometry& in) const
  {
    geometry_msgs::PoseStamped out = poseFromOdom(in);

    // /fastlio/odom 已经是第一帧世界系下的位姿；
    // 这里做的只是“雷达安装位姿”到“机体位姿”的静态刚体修正。
    // 约定：
    //   雷达: x后 y右 z上
    //   机体: x前 y左 z上
    // 所以固定偏航差为 180deg。
    const geometry_msgs::Quaternion q_sensor_world = in.pose.pose.orientation;
    const geometry_msgs::Quaternion q_body_sensor = toMsg(yawQuat(lidar_to_body_yaw_rad_));
    out.pose.orientation = normalize(multiply(q_sensor_world, q_body_sensor));

    // 位置外参：把“机体原点”从雷达原点平移出来。
    // 公式：p_body_world = p_lidar_world + R_world_lidar * t_lidar_body
    const tf2::Quaternion q_world_sensor = fromMsg(in.pose.pose.orientation);
    const tf2::Matrix3x3 rot_world_sensor(q_world_sensor);
    const tf2::Vector3 t_sensor_body(body_in_lidar_x_, body_in_lidar_y_, body_in_lidar_z_);
    const tf2::Vector3 t_world_body = rot_world_sensor * t_sensor_body;

    out.pose.position.x = in.pose.pose.position.x + t_world_body.x();
    out.pose.position.y = in.pose.pose.position.y + t_world_body.y();
    out.pose.position.z = in.pose.pose.position.z + t_world_body.z();

    return out;
  }

  geometry_msgs::PoseStamped worldToMavrosVision(const geometry_msgs::PoseStamped& in) const
  {
    geometry_msgs::PoseStamped out = in;

    // 这里的 W 系是 x前 y左 z上。
    // MAVROS 视觉输入按 ROS 里程计习惯走 ENU 语义，再由 MAVROS 内部转 PX4。
    // 所以这里先把 W 变成 MAVROS 期望的 ENU 位置/姿态补偿。
    out.pose.position.x = in.pose.position.y;
    out.pose.position.y = -in.pose.position.x;
    out.pose.position.z = in.pose.position.z;
    // 正确：FLU -> ENU
    tf2::Quaternion q_world_old_body = fromMsg(in.pose.orientation);
    tf2::Quaternion q_world_new_world_old = yawQuat(-M_PI_2);
    tf2::Quaternion q_world_new_body = q_world_new_world_old * q_world_old_body;
    q_world_new_body.normalize();

    out.pose.orientation = toMsg(q_world_new_body);
    return out;
  }

  void callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    geometry_msgs::PoseStamped world_pose = fastlioToWorldPose(*msg);

    nav_msgs::Odometry corrected_odom = *msg;
    corrected_odom.header.stamp = msg->header.stamp;
    corrected_odom.header.frame_id = output_frame_id_;
    corrected_odom.child_frame_id = output_frame_id_;
    corrected_odom.pose.pose = world_pose.pose;
    odom_pub_.publish(corrected_odom);

    geometry_msgs::PoseStamped vision = worldToMavrosVision(world_pose);
    vision.header.stamp = msg->header.stamp;
    vision.header.frame_id = output_frame_id_;
    if (yaw_only_) vision.pose.orientation = yawOnly(vision.pose.orientation);
    vision_pub_.publish(vision);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher odom_pub_;
  ros::Publisher vision_pub_;

  std::string input_topic_{"/fastlio/odom"};
  std::string output_odometry_topic_{"/Odometry"};
  std::string output_vision_topic_{"/mavros/vision_pose/pose"};
  std::string output_frame_id_{"map"};
  bool yaw_only_{false};

  double body_in_lidar_x_{0.0};
  double body_in_lidar_y_{0.0};
  double body_in_lidar_z_{-0.08};
  double lidar_to_body_yaw_rad_{M_PI};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_to_pose");
  ros::NodeHandle nh("~");
  OdomToPose node(nh);
  ros::spin();
  return 0;
}
