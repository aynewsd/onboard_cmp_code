#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

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
    nh_.param<double>("sensor_to_body_offset_x", sensor_to_body_offset_x_, sensor_to_body_offset_x_);
    nh_.param<double>("sensor_to_body_offset_y", sensor_to_body_offset_y_, sensor_to_body_offset_y_);
    nh_.param<double>("sensor_to_body_offset_z", sensor_to_body_offset_z_, sensor_to_body_offset_z_);
    nh_.param<bool>("apply_sensor_to_body_yaw", apply_sensor_to_body_yaw_, apply_sensor_to_body_yaw_);
    nh_.param<double>("sensor_to_body_yaw_rad", sensor_to_body_yaw_rad_, sensor_to_body_yaw_rad_);

    sub_ = nh_.subscribe<nav_msgs::Odometry>(input_topic_, 50, &OdomToPose::cb, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(output_odometry_topic_, 50);
    vision_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(output_vision_topic_, 50);

    ROS_INFO("odom_to_pose: %s -> %s + %s", input_topic_.c_str(), output_odometry_topic_.c_str(), output_vision_topic_.c_str());
  }

private:
  static geometry_msgs::Quaternion toMsg(const tf2::Quaternion& q)
  {
    geometry_msgs::Quaternion m;
    m.x = q.x();
    m.y = q.y();
    m.z = q.z();
    m.w = q.w();
    return m;
  }

  static geometry_msgs::Quaternion yawOnly(const geometry_msgs::Quaternion& q_in)
  {
    tf2::Quaternion q(q_in.x, q_in.y, q_in.z, q_in.w);
    tf2::Matrix3x3 m(q);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    m.getRPY(roll, pitch, yaw);
    tf2::Quaternion q_yaw;
    q_yaw.setRPY(0.0, 0.0, yaw);
    q_yaw.normalize();
    return toMsg(q_yaw);
  }

  static tf2::Quaternion makeYawQuaternion(double yaw_rad)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_rad);
    q.normalize();
    return q;
  }

  static geometry_msgs::Quaternion transformOrientation(const geometry_msgs::Quaternion& in,
                                                        double sensor_to_body_yaw_rad)
  {
    tf2::Quaternion q_in(in.x, in.y, in.z, in.w);
    tf2::Quaternion q_body_in_sensor = makeYawQuaternion(sensor_to_body_yaw_rad);

    // /fastlio/odom 的姿态先按“雷达体坐标”理解。
    // 若雷达坐标与机体系存在固定偏航差，则在这里补上 q_sensor_to_body。
    tf2::Quaternion q_out = q_in * q_body_in_sensor;
    q_out.normalize();
    return toMsg(q_out);
  }

  static void transformVector(double x_in, double y_in, double z_in,
                              double& x_out, double& y_out, double& z_out)
  {
    x_out = -x_in;
    y_out = -y_in;
    z_out = z_in;
  }

  // W -> MAVROS local compensation:
  // MAVROS 隐含会把 ROS 的 ENU 视觉/位置输入转成 PX4 的 NED/FRD。
  // 我们的任务世界 W 为 x前 y左 z上，所以发给 MAVROS 前先做 W -> ENU 补偿。
  static void compensateWorldToMavrosPosition(double x_in, double y_in, double z_in,
                                              double& x_out, double& y_out, double& z_out)
  {
    x_out = -y_in;
    y_out = x_in;
    z_out = z_in;
  }

  static geometry_msgs::Quaternion compensateWorldToMavrosOrientation(const geometry_msgs::Quaternion& in)
  {
    tf2::Quaternion q_in(in.x, in.y, in.z, in.w);
    tf2::Quaternion q_rot;
    q_rot.setRPY(0.0, 0.0, M_PI_2);  // +90 deg about Z
    tf2::Quaternion q_out = q_rot * q_in;
    q_out.normalize();
    return toMsg(q_out);
  }

  nav_msgs::Odometry transformOdom(const nav_msgs::Odometry& in) const
  {
    nav_msgs::Odometry out = in;
    if (apply_sensor_to_body_yaw_)
    {
      out.pose.pose.orientation = transformOrientation(in.pose.pose.orientation, sensor_to_body_yaw_rad_);
    }
    else
    {
      out.pose.pose.orientation = in.pose.pose.orientation;
    }

    const tf2::Quaternion q_sensor_in_world(in.pose.pose.orientation.x,
                                            in.pose.pose.orientation.y,
                                            in.pose.pose.orientation.z,
                                            in.pose.pose.orientation.w);
    const tf2::Matrix3x3 rot_sensor_in_world(q_sensor_in_world);
    const tf2::Vector3 offset_sensor(sensor_to_body_offset_x_,
                                     sensor_to_body_offset_y_,
                                     sensor_to_body_offset_z_);
    const tf2::Vector3 offset_world = rot_sensor_in_world * offset_sensor;

    out.pose.pose.position.x = in.pose.pose.position.x + offset_world.x();
    out.pose.pose.position.y = in.pose.pose.position.y + offset_world.y();
    out.pose.pose.position.z = in.pose.pose.position.z + offset_world.z();

    transformVector(in.twist.twist.linear.x,
                    in.twist.twist.linear.y,
                    in.twist.twist.linear.z,
                    out.twist.twist.linear.x,
                    out.twist.twist.linear.y,
                    out.twist.twist.linear.z);
    transformVector(in.twist.twist.angular.x,
                    in.twist.twist.angular.y,
                    in.twist.twist.angular.z,
                    out.twist.twist.angular.x,
                    out.twist.twist.angular.y,
                    out.twist.twist.angular.z);

    return out;
  }

  geometry_msgs::PoseStamped transformVisionPose(const nav_msgs::Odometry& in) const
  {
    geometry_msgs::PoseStamped out;
    const nav_msgs::Odometry corrected = transformOdom(in);
    out.header = corrected.header;
    compensateWorldToMavrosPosition(corrected.pose.pose.position.x,
                                    corrected.pose.pose.position.y,
                                    corrected.pose.pose.position.z,
                                    out.pose.position.x,
                                    out.pose.position.y,
                                    out.pose.position.z);
    out.pose.orientation = compensateWorldToMavrosOrientation(corrected.pose.pose.orientation);
    return out;
  }

  void cb(const nav_msgs::Odometry::ConstPtr& msg)
  {
    nav_msgs::Odometry corrected_odom = transformOdom(*msg);
    if (!output_frame_id_.empty())
    {
      corrected_odom.header.frame_id = output_frame_id_;
      corrected_odom.child_frame_id = output_frame_id_;
    }
    odom_pub_.publish(corrected_odom);

    geometry_msgs::PoseStamped vision = transformVisionPose(*msg);
    if (!output_frame_id_.empty()) vision.header.frame_id = output_frame_id_;
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
  double sensor_to_body_offset_x_{0.0};
  double sensor_to_body_offset_y_{0.0};
  double sensor_to_body_offset_z_{-0.08};
  bool apply_sensor_to_body_yaw_{true};
  double sensor_to_body_yaw_rad_{M_PI};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_to_pose");
  ros::NodeHandle nh("~");
  OdomToPose node(nh);
  ros::spin();
  return 0;
}
