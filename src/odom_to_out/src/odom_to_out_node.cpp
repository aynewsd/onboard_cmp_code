#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <string>

class OdomToOut
{
public:
  explicit OdomToOut(ros::NodeHandle& nh) : nh_(nh)
  {
    nh_.param<std::string>("input_topic", input_topic_, input_topic_);
    nh_.param<std::string>("output_topic", output_topic_, output_topic_);
    nh_.param<std::string>("output_frame_id", output_frame_id_, output_frame_id_);
    nh_.param<std::string>("output_child_frame_id", output_child_frame_id_, output_child_frame_id_);
    nh_.param<bool>("convert_twist", convert_twist_, convert_twist_);

    sub_ = nh_.subscribe<nav_msgs::Odometry>(input_topic_, 50, &OdomToOut::cb, this);
    pub_ = nh_.advertise<nav_msgs::Odometry>(output_topic_, 50);

    ROS_INFO("odom_to_out: %s -> %s (ENU->NED)", input_topic_.c_str(), output_topic_.c_str());
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

  // Convert ENU vector (x=E,y=N,z=U) to NED (x=N,y=E,z=D)
  static void enuToNedVec(double x_enu, double y_enu, double z_enu, double& x_ned, double& y_ned, double& z_ned)
  {
    x_ned = y_enu;
    y_ned = x_enu;
    z_ned = -z_enu;
  }

  static tf2::Quaternion enuToNedQuat(const tf2::Quaternion& q_enu)
  {
    // Rotation matrix that maps ENU basis vectors into NED basis vectors:
    // [x_ned y_ned z_ned]^T = R_ned_enu * [x_enu y_enu z_enu]^T
    // R_ned_enu = [[0,1,0],[1,0,0],[0,0,-1]]
    const tf2::Matrix3x3 R_ned_enu(0, 1, 0,
                                   1, 0, 0,
                                   0, 0, -1);

    tf2::Matrix3x3 R_enu(q_enu);
    tf2::Matrix3x3 R_ned = R_ned_enu * R_enu;
    tf2::Quaternion q_ned;
    R_ned.getRotation(q_ned);
    q_ned.normalize();
    return q_ned;
  }

  void cb(const nav_msgs::Odometry::ConstPtr& msg)
  {
    nav_msgs::Odometry out = *msg;

    if (!output_frame_id_.empty()) out.header.frame_id = output_frame_id_;
    if (!output_child_frame_id_.empty()) out.child_frame_id = output_child_frame_id_;

    // Pose position
    enuToNedVec(msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z,
                out.pose.pose.position.x,
                out.pose.pose.position.y,
                out.pose.pose.position.z);

    // Pose orientation
    tf2::Quaternion q_enu(msg->pose.pose.orientation.x,
                          msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z,
                          msg->pose.pose.orientation.w);
    const tf2::Quaternion q_ned = enuToNedQuat(q_enu);
    out.pose.pose.orientation = toMsg(q_ned);

    // Twist (optional)
    if (convert_twist_)
    {
      enuToNedVec(msg->twist.twist.linear.x,
                  msg->twist.twist.linear.y,
                  msg->twist.twist.linear.z,
                  out.twist.twist.linear.x,
                  out.twist.twist.linear.y,
                  out.twist.twist.linear.z);

      enuToNedVec(msg->twist.twist.angular.x,
                  msg->twist.twist.angular.y,
                  msg->twist.twist.angular.z,
                  out.twist.twist.angular.x,
                  out.twist.twist.angular.y,
                  out.twist.twist.angular.z);
    }

    pub_.publish(out);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  std::string input_topic_{"/Odometry"};
  std::string output_topic_{"/mavros/odometry/out"};
  std::string output_frame_id_{"map"};
  std::string output_child_frame_id_{""};
  bool convert_twist_{true};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_to_out");
  ros::NodeHandle nh("~");
  OdomToOut node(nh);
  ros::spin();
  return 0;
}

