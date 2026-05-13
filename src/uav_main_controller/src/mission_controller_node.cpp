#include "uav_main_controller/mission_controller.h"

#include <xmlrpcpp/XmlRpcValue.h>

#include <algorithm>
#include <cmath>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = 1.57079632679489661923;

double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

MissionController::MapPointMm loadPoint2(const ros::NodeHandle& nh, const std::string& key, double def_x, double def_y)
{
  std::vector<double> v;
  if (nh.getParam(key, v) && v.size() >= 2) return {v[0], v[1]};
  return {def_x, def_y};
}

std::vector<MissionController::MapPointMm> loadPointList2(const ros::NodeHandle& nh, const std::string& key)
{
  std::vector<MissionController::MapPointMm> out;
  XmlRpc::XmlRpcValue arr;
  if (!nh.getParam(key, arr) || arr.getType() != XmlRpc::XmlRpcValue::TypeArray) return out;
  for (int i = 0; i < arr.size(); ++i)
  {
    if (arr[i].getType() != XmlRpc::XmlRpcValue::TypeArray || arr[i].size() < 2) continue;
    out.push_back({static_cast<double>(arr[i][0]), static_cast<double>(arr[i][1])});
  }
  return out;
}
} // namespace

MissionController::MissionController(ros::NodeHandle& nh) : nh_(nh)
{
  nh_.param<std::string>("odom_topic", odom_topic_, odom_topic_);

  nh_.param<double>("control_rate", control_rate_, control_rate_);
  nh_.param<double>("reach_tolerance_xy", reach_tol_xy_, reach_tol_xy_);
  nh_.param<double>("reach_tolerance_z", reach_tol_z_, reach_tol_z_);
  nh_.param<double>("prestream_time_s", prestream_time_s_, prestream_time_s_);
  nh_.param<double>("max_xy_speed", max_xy_speed_, max_xy_speed_);
  nh_.param<double>("max_z_speed", max_z_speed_, max_z_speed_);

  nh_.param<double>("cruise_height", cruise_height_, cruise_height_);
  nh_.param<double>("drop_height", drop_height_, drop_height_);
  nh_.param<double>("ring_height", ring_height_, ring_height_);

  nh_.param<double>("segment_timeout_s", segment_timeout_s_, segment_timeout_s_);
  nh_.param<double>("circle_duration_s", circle_duration_s_, circle_duration_s_);
  nh_.param<double>("circle_radius_m", circle_radius_m_, circle_radius_m_);
  nh_.param<double>("circle_period_s", circle_period_s_, circle_period_s_);
  nh_.param<double>("hover_qr_s", hover_qr_s_, hover_qr_s_);
  nh_.param<double>("hover_image_s", hover_image_s_, hover_image_s_);
  nh_.param<double>("hover_special_s", hover_special_s_, hover_special_s_);
  nh_.param<double>("hover_ring_view_s", hover_ring_view_s_, hover_ring_view_s_);
  nh_.param<double>("servo_open_time_s", servo_open_time_s_, servo_open_time_s_);

  nh_.param<double>("failsafe_hover_timeout_s", failsafe_hover_timeout_s_, failsafe_hover_timeout_s_);
  nh_.param<double>("odom_timeout_s", odom_timeout_s_, odom_timeout_s_);

  map_origin_mm_ = loadPoint2(nh_, "map_origin_mm", map_origin_mm_.x_mm, map_origin_mm_.y_mm);
  nh_.param<bool>("map_to_enu_y_inverted", map_to_enu_y_inverted_, map_to_enu_y_inverted_);

  takeoff_mm_ = loadPoint2(nh_, "takeoff_mm", takeoff_mm_.x_mm, takeoff_mm_.y_mm);
  obstacle_mm_ = loadPoint2(nh_, "obstacle_mm", obstacle_mm_.x_mm, obstacle_mm_.y_mm);
  qrcode_mm_ = loadPoint2(nh_, "qrcode_mm", qrcode_mm_.x_mm, qrcode_mm_.y_mm);
  special_target_mm_ = loadPoint2(nh_, "special_target_mm", special_target_mm_.x_mm, special_target_mm_.y_mm);
  ring_view_mm_ = loadPoint2(nh_, "ring_view_mm", ring_view_mm_.x_mm, ring_view_mm_.y_mm);
  ring_default_mm_ = loadPoint2(nh_, "ring_default_mm", ring_default_mm_.x_mm, ring_default_mm_.y_mm);
  land_left_mm_ = loadPoint2(nh_, "land_left_mm", land_left_mm_.x_mm, land_left_mm_.y_mm);
  land_right_mm_ = loadPoint2(nh_, "land_right_mm", land_right_mm_.x_mm, land_right_mm_.y_mm);
  nh_.param<std::string>("default_land_side", default_land_side_, default_land_side_);

  image_targets_mm_ = loadPointList2(nh_, "image_targets_mm");
  if (image_targets_mm_.size() != 4)
  {
    ROS_WARN("image_targets_mm should have 4 points; got %zu, will use defaults", image_targets_mm_.size());
    image_targets_mm_.clear();
    image_targets_mm_.push_back({5100.0, 4600.0});
    image_targets_mm_.push_back({3300.0, 4600.0});
    image_targets_mm_.push_back({5100.0, 1400.0});
    image_targets_mm_.push_back({3300.0, 1400.0});
  }

  state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &MissionController::stateCallback, this);
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 20, &MissionController::odomCallback, this);

  setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 20);
  mission_state_pub_ = nh_.advertise<std_msgs::String>("/mission/state", 10, true);
  target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mission/target_pose", 10);
  servo_pub_ = nh_.advertise<servo_controller::ServoCmd>("/servo_control", 10);

  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  arm_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

  if (control_rate_ < 2.0) ROS_WARN("control_rate < 2Hz may break OFFBOARD");
  if (cruise_height_ < 1.2) ROS_WARN("cruise_height < 1.2m violates requirement");
  if (drop_height_ > 0.8) ROS_WARN("drop_height > 0.8m violates requirement");

  phase_enter_time_ = ros::Time::now();
  segment_start_time_ = phase_enter_time_;
  setpoint_ = makePose(0.0, 0.0, cruise_height_, 0.0);
  goal_pose_ = setpoint_;
  hold_pose_ = setpoint_;
  last_offboard_request_time_ = ros::Time(0);
  resume_phase_ = Phase::TAKEOFF;

  publishMissionState("INIT");
}

void MissionController::stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
  current_state_ = *msg;
}

void MissionController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_odom_ = *msg;
  last_odom_time_ = ros::Time::now();
}

bool MissionController::setMode(const std::string& mode)
{
  mavros_msgs::SetMode mode_msg;
  mode_msg.request.custom_mode = mode;
  return set_mode_client_.call(mode_msg) && mode_msg.response.mode_sent;
}

bool MissionController::arm(bool arm_value)
{
  mavros_msgs::CommandBool arm_msg;
  arm_msg.request.value = arm_value;
  return arm_client_.call(arm_msg) && arm_msg.response.success;
}

void MissionController::publishMissionState(const std::string& text)
{
  std_msgs::String msg;
  msg.data = text;
  mission_state_pub_.publish(msg);
  ROS_INFO("%s", text.c_str());
}

geometry_msgs::Quaternion MissionController::yawToQuaternion(double yaw_rad)
{
  geometry_msgs::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw_rad * 0.5);
  q.w = std::cos(yaw_rad * 0.5);
  return q;
}

geometry_msgs::PoseStamped MissionController::makePose(double x, double y, double z, double yaw_rad) const
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation = yawToQuaternion(yaw_rad);
  return pose;
}

MissionController::EnuPoint MissionController::mapToEnu(const MapPointMm& p_mm, double z_m) const
{
  const double dx_m = (p_mm.x_mm - map_origin_mm_.x_mm) / 1000.0;
  double dy_m = (p_mm.y_mm - map_origin_mm_.y_mm) / 1000.0;
  if (map_to_enu_y_inverted_) dy_m = -dy_m;
  return {dx_m, dy_m, z_m};
}

bool MissionController::reached(const geometry_msgs::PoseStamped& target) const
{
  const double dx = target.pose.position.x - current_odom_.pose.pose.position.x;
  const double dy = target.pose.position.y - current_odom_.pose.pose.position.y;
  const double dz = target.pose.position.z - current_odom_.pose.pose.position.z;
  const double dxy = std::sqrt(dx * dx + dy * dy);
  return (dxy <= reach_tol_xy_) && (std::fabs(dz) <= reach_tol_z_);
}

geometry_msgs::PoseStamped MissionController::stepToward(const geometry_msgs::PoseStamped& from,
                                                        const geometry_msgs::PoseStamped& to,
                                                        double dt) const
{
  geometry_msgs::PoseStamped out = from;
  const double dx = to.pose.position.x - from.pose.position.x;
  const double dy = to.pose.position.y - from.pose.position.y;
  const double dz = to.pose.position.z - from.pose.position.z;

  const double dist_xy = std::sqrt(dx * dx + dy * dy);
  const double max_step_xy = std::max(0.0, max_xy_speed_) * dt;
  const double max_step_z = std::max(0.0, max_z_speed_) * dt;

  if (dist_xy > 1e-6)
  {
    const double scale = std::min(1.0, max_step_xy / dist_xy);
    out.pose.position.x += dx * scale;
    out.pose.position.y += dy * scale;
  }
  else
  {
    out.pose.position.x = to.pose.position.x;
    out.pose.position.y = to.pose.position.y;
  }

  const double step_z = clamp(dz, -max_step_z, max_step_z);
  out.pose.position.z += step_z;

  out.pose.orientation = to.pose.orientation;
  return out;
}

bool MissionController::odomHealthy(const ros::Time& now) const
{
  if (last_odom_time_.isZero()) return false;
  return (now - last_odom_time_).toSec() <= odom_timeout_s_;
}

void MissionController::publishSetpointNow(const geometry_msgs::PoseStamped& pose)
{
  geometry_msgs::PoseStamped p = pose;
  p.header.stamp = ros::Time::now();
  setpoint_pub_.publish(p);
  target_pose_pub_.publish(p);
}

void MissionController::enterPhase(Phase next, const std::string& reason)
{
  phase_ = next;
  phase_enter_time_ = ros::Time::now();
  segment_start_time_ = phase_enter_time_;
  publishMissionState(reason);
}

void MissionController::triggerFailsafeHover(const std::string& reason)
{
  resume_phase_ = phase_;
  last_offboard_request_time_ = ros::Time(0);
  hold_pose_ = makePose(current_odom_.pose.pose.position.x,
                        current_odom_.pose.pose.position.y,
                        current_odom_.pose.pose.position.z,
                        0.0);
  goal_pose_ = hold_pose_;
  failsafe_start_time_ = ros::Time::now();
  enterPhase(Phase::FAILSAFE_HOVER, "FAILSAFE_HOVER: " + reason);
}

void MissionController::startServoDrop(int8_t servo_id, const ros::Time& now)
{
  if (servo_open_)
  {
    ROS_WARN("Servo drop requested while another servo is open (active=%d)", static_cast<int>(active_servo_id_));
    return;
  }

  servo_controller::ServoCmd cmd;
  cmd.header.stamp = now;
  cmd.servo_id = servo_id;
  cmd.position = 1;
  servo_pub_.publish(cmd);

  active_servo_id_ = servo_id;
  servo_open_ = true;
  servo_open_until_ = now + ros::Duration(servo_open_time_s_);
}

void MissionController::tickServo(const ros::Time& now)
{
  if (!servo_open_) return;
  if (servo_open_until_.isZero()) return;
  if (now < servo_open_until_) return;

  servo_controller::ServoCmd cmd;
  cmd.header.stamp = now;
  cmd.servo_id = active_servo_id_;
  cmd.position = 0;
  servo_pub_.publish(cmd);

  servo_open_ = false;
  active_servo_id_ = 0;
  servo_open_until_ = ros::Time(0);
}

void MissionController::tick(const ros::Time& now, double dt)
{
  tickServo(now);

  const std::string land_mode = "AUTO.LAND";  // PX4 mode name

  if (phase_ != Phase::WAIT_FCU && phase_ != Phase::WAIT_ODOM && !odomHealthy(now))
  {
    triggerFailsafeHover("ODOM timeout");
    return;
  }

  // If operator takes over by leaving OFFBOARD, stop mission logic.
  // Note: before OFFBOARD is successfully set, FCU mode is expected to be something else,
  // so do NOT enter WAIT_MANUAL during PREOFFBOARD_STREAM/SET_OFFBOARD/ARM.
  const bool manual_takeover_sensitive =
      phase_ != Phase::WAIT_FCU && phase_ != Phase::WAIT_ODOM && phase_ != Phase::PREOFFBOARD_STREAM &&
      phase_ != Phase::SET_OFFBOARD && phase_ != Phase::ARM && phase_ != Phase::LAND && phase_ != Phase::DONE &&
      phase_ != Phase::WAIT_MANUAL && phase_ != Phase::FAILSAFE_HOVER;

  const bool not_offboard = !current_state_.mode.empty() && current_state_.mode != "OFFBOARD";
  if (manual_takeover_sensitive && not_offboard)
  {
    // Distinguish operator takeover vs OFFBOARD loss/failsafe:
    // - manual_input==true: user is actively controlling -> stop fighting (WAIT_MANUAL)
    // - manual_input==false: likely OFFBOARD was lost automatically -> failsafe hover (keep trying / wait takeover)
    resume_phase_ = phase_;
    if (current_state_.manual_input)
    {
      enterPhase(Phase::WAIT_MANUAL, "WAIT_MANUAL: FCU mode=" + current_state_.mode);
    }
    else
    {
      triggerFailsafeHover("OFFBOARD lost: FCU mode=" + current_state_.mode);
    }
    return;
  }

  if (phase_ == Phase::WAIT_MANUAL)
  {
    // If user switches back to OFFBOARD, resume mission from last phase.
    if (!current_state_.mode.empty() && current_state_.mode == "OFFBOARD" && !current_state_.manual_input)
    {
      // Reset setpoint close to current position to avoid a jump.
      setpoint_ = makePose(current_odom_.pose.pose.position.x,
                           current_odom_.pose.pose.position.y,
                           current_odom_.pose.pose.position.z,
                           0.0);
      goal_pose_ = setpoint_;
      enterPhase(resume_phase_, "RESUME: OFFBOARD restored");
      return;
    }

    if ((now - phase_enter_time_).toSec() > segment_timeout_s_)
    {
      publishMissionState("WAIT_MANUAL timeout: switch to OFFBOARD to resume, or LAND manually");
      phase_enter_time_ = now;
    }
    return;
  }

  // Segment timeout applies only to transit-like phases.
  // Phases with explicit durations (circle/hover) are excluded to avoid boundary preemption.
  const bool timeout_sensitive =
      phase_ == Phase::TAKEOFF || phase_ == Phase::GOTO_OBSTACLE || phase_ == Phase::GOTO_QRCODE ||
      phase_ == Phase::GOTO_IMAGE_1 || phase_ == Phase::GOTO_IMAGE_2 || phase_ == Phase::GOTO_IMAGE_3 ||
      phase_ == Phase::GOTO_IMAGE_4 || phase_ == Phase::GOTO_SPECIAL || phase_ == Phase::GOTO_RING_VIEW ||
      phase_ == Phase::PASS_RING || phase_ == Phase::GOTO_LAND;

  if (timeout_sensitive && (now - segment_start_time_).toSec() > segment_timeout_s_)
  {
    triggerFailsafeHover("segment timeout");
    return;
  }

  switch (phase_)
  {
    case Phase::WAIT_FCU:
    {
      publishSetpointNow(setpoint_);
      if (current_state_.connected) enterPhase(Phase::WAIT_ODOM, "WAIT_ODOM");
      break;
    }
    case Phase::WAIT_ODOM:
    {
      if (odomHealthy(now))
      {
        // Initialize setpoint close to the vehicle to avoid a large jump during pre-stream.
        setpoint_ = makePose(current_odom_.pose.pose.position.x,
                             current_odom_.pose.pose.position.y,
                             current_odom_.pose.pose.position.z,
                             0.0);
        goal_pose_ = setpoint_;
        publishSetpointNow(setpoint_);
        enterPhase(Phase::PREOFFBOARD_STREAM, "PREOFFBOARD_STREAM");
      }
      else
      {
        publishSetpointNow(setpoint_);
      }
      break;
    }
    case Phase::PREOFFBOARD_STREAM:
    {
      publishSetpointNow(setpoint_);
      if ((now - phase_enter_time_).toSec() >= prestream_time_s_) enterPhase(Phase::ARM, "ARM");
      break;
    }
    case Phase::ARM:
    {
      publishSetpointNow(setpoint_);
      if (arm(true)) enterPhase(Phase::SET_OFFBOARD, "SET_OFFBOARD");
      else triggerFailsafeHover("failed to ARM");
      break;
    }
    case Phase::SET_OFFBOARD:
    {
      publishSetpointNow(setpoint_);
      if (!current_state_.mode.empty() && current_state_.mode == "OFFBOARD")
      {
        enterPhase(Phase::TAKEOFF, "TAKEOFF");
        break;
      }

      // Try to switch to OFFBOARD periodically while continuing to stream setpoints.
      if (last_offboard_request_time_.isZero() || (now - last_offboard_request_time_).toSec() > 1.0)
      {
        (void)setMode("OFFBOARD");
        last_offboard_request_time_ = now;
      }
      break;
    }
    case Phase::TAKEOFF:
    {
      goal_pose_ = makePose(current_odom_.pose.pose.position.x,
                            current_odom_.pose.pose.position.y,
                            cruise_height_,
                            0.0);
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if (reached(goal_pose_)) enterPhase(Phase::GOTO_OBSTACLE, "GOTO_OBSTACLE");
      break;
    }
    case Phase::GOTO_OBSTACLE:
    {
      const auto p = mapToEnu(obstacle_mm_, cruise_height_);
      goal_pose_ = makePose(p.x, p.y, p.z, 0.0);
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if (reached(goal_pose_)) enterPhase(Phase::CIRCLE_OBSTACLE, "CIRCLE_OBSTACLE");
      break;
    }
    case Phase::CIRCLE_OBSTACLE:
    {
      const auto c = mapToEnu(obstacle_mm_, cruise_height_);
      const double t = (now - phase_enter_time_).toSec();
      const double w = 2.0 * kPi / std::max(0.1, circle_period_s_);
      const double angle = w * t;
      goal_pose_ = makePose(c.x + circle_radius_m_ * std::cos(angle),
                            c.y + circle_radius_m_ * std::sin(angle),
                            cruise_height_,
                            angle + kHalfPi);
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if (t >= circle_duration_s_) enterPhase(Phase::GOTO_QRCODE, "GOTO_QRCODE");
      break;
    }
    case Phase::GOTO_QRCODE:
    {
      const auto p = mapToEnu(qrcode_mm_, cruise_height_);
      goal_pose_ = makePose(p.x, p.y, p.z, 0.0);
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if (reached(goal_pose_)) enterPhase(Phase::HOVER_QRCODE, "HOVER_QRCODE");
      break;
    }
    case Phase::HOVER_QRCODE:
    {
      publishSetpointNow(setpoint_);
      if ((now - phase_enter_time_).toSec() >= hover_qr_s_) enterPhase(Phase::GOTO_IMAGE_1, "GOTO_IMAGE_1");
      break;
    }
    case Phase::GOTO_IMAGE_1:
    {
      const auto p = mapToEnu(image_targets_mm_.at(0), cruise_height_);
      goal_pose_ = makePose(p.x, p.y, p.z, 0.0);
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if (reached(goal_pose_)) enterPhase(Phase::HOVER_IMAGE_1_DROP_1, "HOVER_IMAGE_1_DROP_1");
      break;
    }
    case Phase::HOVER_IMAGE_1_DROP_1:
    {
      if (!servo_1_done_)
      {
        const auto low = mapToEnu(image_targets_mm_.at(0), drop_height_);
        goal_pose_ = makePose(low.x, low.y, low.z, 0.0);
        setpoint_ = stepToward(setpoint_, goal_pose_, dt);
        publishSetpointNow(setpoint_);
        if (reached(goal_pose_))
        {
          startServoDrop(1, now);
          servo_1_done_ = true;
          segment_start_time_ = now;
        }
        break;
      }

      const auto p = mapToEnu(image_targets_mm_.at(0), cruise_height_);
      goal_pose_ = makePose(p.x, p.y, p.z, 0.0);
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if ((now - phase_enter_time_).toSec() >= hover_image_s_) enterPhase(Phase::GOTO_IMAGE_2, "GOTO_IMAGE_2");
      break;
    }
    case Phase::GOTO_IMAGE_2:
    {
      const auto p = mapToEnu(image_targets_mm_.at(1), cruise_height_);
      goal_pose_ = makePose(p.x, p.y, p.z, 0.0);
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if (reached(goal_pose_)) enterPhase(Phase::HOVER_IMAGE_2_DROP_2, "HOVER_IMAGE_2_DROP_2");
      break;
    }
    case Phase::HOVER_IMAGE_2_DROP_2:
    {
      if (!servo_2_done_)
      {
        const auto low = mapToEnu(image_targets_mm_.at(1), drop_height_);
        goal_pose_ = makePose(low.x, low.y, low.z, 0.0);
        setpoint_ = stepToward(setpoint_, goal_pose_, dt);
        publishSetpointNow(setpoint_);
        if (reached(goal_pose_))
        {
          startServoDrop(2, now);
          servo_2_done_ = true;
          segment_start_time_ = now;
        }
        break;
      }

      const auto p = mapToEnu(image_targets_mm_.at(1), cruise_height_);
      goal_pose_ = makePose(p.x, p.y, p.z, 0.0);
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if ((now - phase_enter_time_).toSec() >= hover_image_s_) enterPhase(Phase::GOTO_IMAGE_3, "GOTO_IMAGE_3");
      break;
    }
    case Phase::GOTO_IMAGE_3:
    {
      const auto p = mapToEnu(image_targets_mm_.at(2), cruise_height_);
      goal_pose_ = makePose(p.x, p.y, p.z, 0.0);
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if (reached(goal_pose_)) enterPhase(Phase::HOVER_IMAGE_3, "HOVER_IMAGE_3");
      break;
    }
    case Phase::HOVER_IMAGE_3:
    {
      publishSetpointNow(setpoint_);
      if ((now - phase_enter_time_).toSec() >= hover_image_s_) enterPhase(Phase::GOTO_IMAGE_4, "GOTO_IMAGE_4");
      break;
    }
    case Phase::GOTO_IMAGE_4:
    {
      const auto p = mapToEnu(image_targets_mm_.at(3), cruise_height_);
      goal_pose_ = makePose(p.x, p.y, p.z, 0.0);
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if (reached(goal_pose_)) enterPhase(Phase::HOVER_IMAGE_4, "HOVER_IMAGE_4");
      break;
    }
    case Phase::HOVER_IMAGE_4:
    {
      publishSetpointNow(setpoint_);
      if ((now - phase_enter_time_).toSec() >= hover_image_s_) enterPhase(Phase::GOTO_SPECIAL, "GOTO_SPECIAL");
      break;
    }
    case Phase::GOTO_SPECIAL:
    {
      const auto p = mapToEnu(special_target_mm_, cruise_height_);
      goal_pose_ = makePose(p.x, p.y, p.z, 0.0);
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if (reached(goal_pose_)) enterPhase(Phase::HOVER_SPECIAL_DROP_3, "HOVER_SPECIAL_DROP_3");
      break;
    }
    case Phase::HOVER_SPECIAL_DROP_3:
    {
      if (!servo_3_done_)
      {
        const auto low = mapToEnu(special_target_mm_, drop_height_);
        goal_pose_ = makePose(low.x, low.y, low.z, 0.0);
        setpoint_ = stepToward(setpoint_, goal_pose_, dt);
        publishSetpointNow(setpoint_);
        if (reached(goal_pose_))
        {
          startServoDrop(3, now);
          servo_3_done_ = true;
          segment_start_time_ = now;
        }
        break;
      }

      const auto p = mapToEnu(special_target_mm_, cruise_height_);
      goal_pose_ = makePose(p.x, p.y, p.z, 0.0);
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if ((now - phase_enter_time_).toSec() >= hover_special_s_) enterPhase(Phase::GOTO_RING_VIEW, "GOTO_RING_VIEW");
      break;
    }
    case Phase::GOTO_RING_VIEW:
    {
      const auto p = mapToEnu(ring_view_mm_, cruise_height_);
      const double yaw = map_to_enu_y_inverted_ ? -kHalfPi : kHalfPi;  // face map +Y
      goal_pose_ = makePose(p.x, p.y, p.z, yaw);
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if (reached(goal_pose_)) enterPhase(Phase::HOVER_RING_VIEW, "HOVER_RING_VIEW");
      break;
    }
    case Phase::HOVER_RING_VIEW:
    {
      publishSetpointNow(setpoint_);
      if ((now - phase_enter_time_).toSec() >= hover_ring_view_s_) enterPhase(Phase::PASS_RING, "PASS_RING");
      break;
    }
    case Phase::PASS_RING:
    {
      const auto ring = mapToEnu(ring_default_mm_, ring_height_);
      goal_pose_ = makePose(ring.x, ring.y, ring.z, 0.0);
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if (reached(goal_pose_)) enterPhase(Phase::GOTO_LAND, "GOTO_LAND");
      break;
    }
    case Phase::GOTO_LAND:
    {
      const MapPointMm land = (default_land_side_ == "right") ? land_right_mm_ : land_left_mm_;
      const auto p = mapToEnu(land, cruise_height_);
      goal_pose_ = makePose(p.x, p.y, p.z, 0.0);
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if (reached(goal_pose_)) enterPhase(Phase::LAND, "LAND");
      break;
    }
    case Phase::LAND:
    {
      publishSetpointNow(setpoint_);
      setMode(land_mode);
      if (!current_state_.armed) enterPhase(Phase::DONE, "DONE");
      break;
    }
    case Phase::FAILSAFE_HOVER:
    {
      // If operator is actively controlling, stop publishing setpoints.
      if (current_state_.manual_input && (!current_state_.mode.empty() && current_state_.mode != "OFFBOARD"))
      {
        enterPhase(Phase::WAIT_MANUAL, "WAIT_MANUAL: operator takeover after failsafe");
        break;
      }

      // If OFFBOARD was lost automatically, keep streaming a hold setpoint and try to re-enter OFFBOARD.
      if (!current_state_.mode.empty() && current_state_.mode != "OFFBOARD")
      {
        if (last_offboard_request_time_.isZero() || (now - last_offboard_request_time_).toSec() > 1.0)
        {
          (void)setMode("OFFBOARD");
          last_offboard_request_time_ = now;
        }
      }

      goal_pose_ = hold_pose_;
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);

      if ((now - failsafe_start_time_).toSec() >= failsafe_hover_timeout_s_)
      {
        publishMissionState("FAILSAFE timeout -> LAND");
        setMode(land_mode);
        enterPhase(Phase::LAND, "LAND");
      }
      break;
    }
    case Phase::DONE:
    default:
      break;
  }
}

void MissionController::run()
{
  ros::Rate rate(control_rate_);
  ros::Time last = ros::Time::now();

  while (ros::ok())
  {
    ros::spinOnce();
    const ros::Time now = ros::Time::now();
    const double dt = std::max(0.0, (now - last).toSec());
    last = now;

    tick(now, dt);
    rate.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_main_controller");
  ros::NodeHandle nh("~");
  MissionController controller(nh);
  controller.run();
  return 0;
}
