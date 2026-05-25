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
    nh_.param<std::string>("output_odometry_topic", output_odometry_topic_, output_odometry_topic_);
    nh_.param<std::string>("output_vision_topic", output_vision_topic_, output_vision_topic_);
    nh_.param<std::string>("output_frame_id", output_frame_id_, output_frame_id_);
    nh_.param<bool>("yaw_only", yaw_only_, yaw_only_);

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

  // Static rigid transform from /fastlio/odom to the corrected /Odometry frame.
  // Requirement:
  //   - new origin is 0.08 m below the /fastlio/odom origin
  //   - rotate around Z by 180 deg
  //
  // For coordinates expressed in the new frame:
  //   x' = -x
  //   y' = -y
  //   z' =  z + 0.08
  static void transformPosition(double x_in, double y_in, double z_in,
                                double& x_out, double& y_out, double& z_out)
  {
    x_out = -x_in;
    y_out = -y_in;
    z_out = z_in + 0.08;
  }

  static void transformVector(double x_in, double y_in, double z_in,
                              double& x_out, double& y_out, double& z_out)
  {
    x_out = -x_in;
    y_out = -y_in;
    z_out = z_in;
  }

  static geometry_msgs::Quaternion transformOrientation(const geometry_msgs::Quaternion& in)
  {
    // Rotate the frame by 180 deg around +Z.
    tf2::Quaternion q_in(in.x, in.y, in.z, in.w);
    tf2::Quaternion q_rot;
    q_rot.setRPY(0.0, 0.0, M_PI);
    tf2::Quaternion q_out = q_rot * q_in;
    q_out.normalize();
    return toMsg(q_out);
  }

  static nav_msgs::Odometry transformOdom(const nav_msgs::Odometry& in)
  {
    nav_msgs::Odometry out = in;
    transformPosition(in.pose.pose.position.x,
                      in.pose.pose.position.y,
                      in.pose.pose.position.z,
                      out.pose.pose.position.x,
                      out.pose.pose.position.y,
                      out.pose.pose.position.z);
    out.pose.pose.orientation = transformOrientation(in.pose.pose.orientation);

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

  static geometry_msgs::PoseStamped transformVisionPose(const nav_msgs::Odometry& in)
  {
    geometry_msgs::PoseStamped out;
    out.header = in.header;
    out.pose = transformOdom(in).pose.pose;
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
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_to_pose");
  ros::NodeHandle nh("~");
  OdomToPose node(nh);
  ros::spin();
  return 0;
}
