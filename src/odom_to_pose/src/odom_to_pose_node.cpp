#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <string>

class OdomToPose
{
public:
  explicit OdomToPose(ros::NodeHandle& nh) : nh_(nh)
  {
    nh_.param<std::string>("input_topic", input_topic_, input_topic_);
    nh_.param<std::string>("output_topic", output_topic_, output_topic_);
    nh_.param<std::string>("output_frame_id", output_frame_id_, output_frame_id_);
    nh_.param<bool>("yaw_only", yaw_only_, yaw_only_);

    sub_ = nh_.subscribe<nav_msgs::Odometry>(input_topic_, 50, &OdomToPose::cb, this);
    pub_ = nh_.advertise<geometry_msgs::PoseStamped>(output_topic_, 50);

    ROS_INFO("odom_to_pose: %s -> %s", input_topic_.c_str(), output_topic_.c_str());
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

  void cb(const nav_msgs::Odometry::ConstPtr& msg)
  {
    geometry_msgs::PoseStamped out;
    out.header = msg->header;
    if (!output_frame_id_.empty()) out.header.frame_id = output_frame_id_;

    // Forward pose (position + orientation)
    out.pose = msg->pose.pose;
    if (yaw_only_) out.pose.orientation = yawOnly(out.pose.orientation);

    pub_.publish(out);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  std::string input_topic_{"/Odometry"};
  std::string output_topic_{"/mavros/vision_pose/pose"};
  std::string output_frame_id_{"map"};
  bool yaw_only_{false};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_to_pose");
  ros::NodeHandle nh("~");
  OdomToPose node(nh);
  ros::spin();
  return 0;
}

