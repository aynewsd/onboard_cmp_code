#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

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

    // T^P_B: body origin/orientation expressed in lidar frame P.
    // P: x back, y right, z up. B: x front, y left, z up.
    nh_.param<double>("body_in_lidar_x", body_in_lidar_x_, body_in_lidar_x_);
    nh_.param<double>("body_in_lidar_y", body_in_lidar_y_, body_in_lidar_y_);
    nh_.param<double>("body_in_lidar_z", body_in_lidar_z_, body_in_lidar_z_);
    nh_.param<double>("lidar_to_body_yaw_rad", lidar_to_body_yaw_rad_, lidar_to_body_yaw_rad_);

    // 工程修正偏置：在严格坐标变换算出 T^W_B 后，再给 /Odometry 的位置加一个常量偏置。
    // 这个参数用于补偿实测中的整体定位零偏，不参与雷达-机体外参计算，避免破坏物理外参。
    // 因为 /mavros/vision_pose/pose 来自同一个修正后的 T^W_B，所以 PX4收到的位置也会同步修正。
    nh_.param<double>("odometry_offset_x", odometry_offset_x_, odometry_offset_x_);
    nh_.param<double>("odometry_offset_y", odometry_offset_y_, odometry_offset_y_);
    nh_.param<double>("odometry_offset_z", odometry_offset_z_, odometry_offset_z_);

    // T^M_W: W coordinates converted into the MAVROS input frame M.
    // MAVROS then converts ENU-like input to PX4 NED internally.
    // To make PX4 see (x, -y, z/down handled by MAVROS), publish M = Rz(+90deg) * W:
    // (x, y, z)_W -> (-y, x, z)_M.
    nh_.param<double>("world_to_mavros_yaw_rad", world_to_mavros_yaw_rad_, world_to_mavros_yaw_rad_);

    sub_ = nh_.subscribe<nav_msgs::Odometry>(input_topic_, 50, &OdomToPose::callback, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(output_odometry_topic_, 50);
    vision_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(output_vision_topic_, 50);

    ROS_INFO("odom_to_pose: %s -> %s + %s",
             input_topic_.c_str(), output_odometry_topic_.c_str(), output_vision_topic_.c_str());
  }

private:
  struct Transform
  {
    tf2::Quaternion q;  // R^parent_child
    tf2::Vector3 t;     // p^parent_child
  };

  static tf2::Quaternion yawQuat(double yaw_rad)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_rad);
    q.normalize();
    return q;
  }

  static tf2::Quaternion fromMsg(const geometry_msgs::Quaternion& q)
  {
    tf2::Quaternion out(q.x, q.y, q.z, q.w);
    out.normalize();
    return out;
  }

  static geometry_msgs::Quaternion toMsg(const tf2::Quaternion& q)
  {
    tf2::Quaternion n = q;
    n.normalize();
    geometry_msgs::Quaternion msg;
    msg.x = n.x();
    msg.y = n.y();
    msg.z = n.z();
    msg.w = n.w();
    return msg;
  }

  static tf2::Vector3 rotate(const tf2::Quaternion& q, const tf2::Vector3& v)
  {
    return tf2::Matrix3x3(q) * v;
  }

  static Transform compose(const Transform& a_b, const Transform& b_c)
  {
    // T^A_C = T^A_B * T^B_C
    Transform a_c;
    a_c.q = a_b.q * b_c.q;
    a_c.q.normalize();
    a_c.t = a_b.t + rotate(a_b.q, b_c.t);
    return a_c;
  }

  static Transform inverse(const Transform& a_b)
  {
    // T^B_A = inv(T^A_B)
    Transform b_a;
    b_a.q = a_b.q.inverse();
    b_a.q.normalize();
    b_a.t = rotate(b_a.q, -a_b.t);
    return b_a;
  }

  static Transform transformFromOdom(const nav_msgs::Odometry& odom)
  {
    Transform out;
    out.q = fromMsg(odom.pose.pose.orientation);
    out.t = tf2::Vector3(odom.pose.pose.position.x,
                         odom.pose.pose.position.y,
                         odom.pose.pose.position.z);
    return out;
  }

  static geometry_msgs::Pose poseFromTransform(const Transform& transform)
  {
    geometry_msgs::Pose pose;
    pose.position.x = transform.t.x();
    pose.position.y = transform.t.y();
    pose.position.z = transform.t.z();
    pose.orientation = toMsg(transform.q);
    return pose;
  }

  static geometry_msgs::Quaternion yawOnly(const geometry_msgs::Quaternion& q_in)
  {
    tf2::Matrix3x3 m(fromMsg(q_in));
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    m.getRPY(roll, pitch, yaw);
    return toMsg(yawQuat(yaw));
  }

  Transform staticTPB() const
  {
    // T^P_B: P is 8 cm above B, so B origin is z=-0.08 in P by default.
    Transform p_b;
    p_b.q = yawQuat(lidar_to_body_yaw_rad_);
    p_b.t = tf2::Vector3(body_in_lidar_x_, body_in_lidar_y_, body_in_lidar_z_);
    return p_b;
  }

  Transform staticTOW() const
  {
    // Initial condition: P coincides with O and B coincides with W.
    // Therefore the world offset T^O_W is the same rigid transform as T^P_B.
    return staticTPB();
  }

  Transform staticTMW() const
  {
    Transform m_w;
    m_w.q = yawQuat(world_to_mavros_yaw_rad_);
    m_w.t = tf2::Vector3(0.0, 0.0, 0.0);
    return m_w;
  }

  Transform fastlioToWorldBody(const nav_msgs::Odometry& in) const
  {
    // Known:
    //   /fastlio/odom = T^O_P
    //   static T^P_B
    //   static T^O_W
    // Need:
    //   /Odometry = T^W_B = T^W_O * T^O_P * T^P_B
    const Transform o_p = transformFromOdom(in);
    const Transform p_b = staticTPB();
    const Transform w_o = inverse(staticTOW());
    return compose(compose(w_o, o_p), p_b);
  }

  Transform worldBodyToMavrosBody(const Transform& w_b) const
  {
    // Need:
    //   /mavros/vision_pose/pose = T^M_B = T^M_W * T^W_B
    return compose(staticTMW(), w_b);
  }

  Transform applyOdometryOffset(const Transform& w_b) const
  {
    // 工程偏置定义在 W 世界系下，直接修正机体原点 p^W_B。
    // 只修正位置，不修正姿态；例如 /Odometry.z 实测低 0.12m，则设置 odometry_offset_z=0.12。
    Transform corrected = w_b;
    corrected.t += tf2::Vector3(odometry_offset_x_, odometry_offset_y_, odometry_offset_z_);
    return corrected;
  }

  void callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    const Transform w_b = applyOdometryOffset(fastlioToWorldBody(*msg));
    const Transform m_b = worldBodyToMavrosBody(w_b);

    nav_msgs::Odometry odom = *msg;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = output_frame_id_;
    odom.child_frame_id = "base_link";
    odom.pose.pose = poseFromTransform(w_b);
    odom_pub_.publish(odom);

    geometry_msgs::PoseStamped vision;
    vision.header.stamp = msg->header.stamp;
    vision.header.frame_id = output_frame_id_;
    vision.pose = poseFromTransform(m_b);
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
  double odometry_offset_x_{0.0};
  double odometry_offset_y_{0.0};
  double odometry_offset_z_{0.0};
  double world_to_mavros_yaw_rad_{M_PI_2};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_to_pose");
  ros::NodeHandle nh("~");
  OdomToPose node(nh);
  ros::spin();
  return 0;
}
