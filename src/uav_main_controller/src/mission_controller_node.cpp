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

  nh_.param<double>("obstacle_size_m", obstacle_size_m_, obstacle_size_m_);
  nh_.param<double>("obstacle_clearance_m", obstacle_clearance_m_, obstacle_clearance_m_);
  nh_.param<bool>("avoid_ring_zone", avoid_ring_zone_, avoid_ring_zone_);

  nh_.param<bool>("rc_drop_select_enabled", rc_drop_select_enabled_, rc_drop_select_enabled_);
  nh_.param<double>("rc_timeout_s", rc_timeout_s_, rc_timeout_s_);
  std::string rc_topic = "/mavros/rc/in";
  nh_.param<std::string>("rc_topic", rc_topic, rc_topic);

  map_origin_mm_ = loadPoint2(nh_, "map_origin_mm", map_origin_mm_.x_mm, map_origin_mm_.y_mm);
  // Backward compatible param name:
  // - new: map_to_w_y_inverted
  // - old: map_to_enu_y_inverted
  if (!nh_.getParam("map_to_w_y_inverted", map_to_w_y_inverted_))
  {
    nh_.param<bool>("map_to_enu_y_inverted", map_to_w_y_inverted_, map_to_w_y_inverted_);
  }

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
  rc_sub_ = nh_.subscribe<mavros_msgs::RCIn>(rc_topic, 10, &MissionController::rcInCallback, this);

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
  last_rc_time_ = ros::Time(0);
  selected_drop1_ = 1;
  selected_drop2_ = 2;
  rc_drop_selected_locked_ = false;
  selected_land_side_ = default_land_side_;

  publishMissionState("INIT");
}

double MissionController::distPointToSeg2d(double px, double py, double ax, double ay, double bx, double by)
{
  const double abx = bx - ax;
  const double aby = by - ay;
  const double apx = px - ax;
  const double apy = py - ay;
  const double ab2 = abx * abx + aby * aby;
  if (ab2 < 1e-9) return std::sqrt(apx * apx + apy * apy);
  double t = (apx * abx + apy * aby) / ab2;
  t = std::max(0.0, std::min(1.0, t));
  const double cx = ax + t * abx;
  const double cy = ay + t * aby;
  const double dx = px - cx;
  const double dy = py - cy;
  return std::sqrt(dx * dx + dy * dy);
}

bool MissionController::segIntersectsRect2d(double ax, double ay, double bx, double by, const Rect2D& r)
{
  // Quick reject by bounding box overlap
  const double minx = std::min(ax, bx);
  const double maxx = std::max(ax, bx);
  const double miny = std::min(ay, by);
  const double maxy = std::max(ay, by);
  if (maxx < r.min_x || minx > r.max_x || maxy < r.min_y || miny > r.max_y) return false;

  // Conservative sampling is enough for our coarse planning (fast + robust for axis-aligned rect).
  const int steps = 60;
  for (int i = 0; i <= steps; ++i)
  {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const double x = ax + t * (bx - ax);
    const double y = ay + t * (by - ay);
    if (x >= r.min_x && x <= r.max_x && y >= r.min_y && y <= r.max_y) return true;
  }
  return false;
}

bool MissionController::needsDetour(const geometry_msgs::PoseStamped& from,
                                    const geometry_msgs::PoseStamped& to,
                                    bool avoid_ring_zone) const
{
  const double ax = from.pose.position.x;
  const double ay = from.pose.position.y;
  const double bx = to.pose.position.x;
  const double by = to.pose.position.y;

  // 1) Obstacle avoidance: treat obstacle as a circle with radius = half_size + clearance.
  const auto obs = mapToWorld(obstacle_mm_, cruise_height_);
  const double obs_r = 0.5 * obstacle_size_m_ + obstacle_clearance_m_;
  const double d_obs = distPointToSeg2d(obs.x, obs.y, ax, ay, bx, by);
  const bool hit_obstacle = d_obs < obs_r;

  // 2) Ring random-zone avoidance: rectangle in map coords [(6300,6000),(8700,4200)] mm.
  // Convert to world frame W rectangle according to current map->W rule (including y inversion).
  Rect2D ring_rect;
  if (avoid_ring_zone)
  {
    const MapPointMm p1{6300.0, 6000.0};
    const MapPointMm p2{8700.0, 4200.0};
    const auto e1 = mapToWorld(p1, 0.0);
    const auto e2 = mapToWorld(p2, 0.0);
    ring_rect.min_x = std::min(e1.x, e2.x);
    ring_rect.max_x = std::max(e1.x, e2.x);
    ring_rect.min_y = std::min(e1.y, e2.y);
    ring_rect.max_y = std::max(e1.y, e2.y);
  }
  const bool hit_ring_rect = avoid_ring_zone && segIntersectsRect2d(ax, ay, bx, by, ring_rect);

  return hit_obstacle || hit_ring_rect;
}

geometry_msgs::PoseStamped MissionController::computeDetour(const geometry_msgs::PoseStamped& from,
                                                           const geometry_msgs::PoseStamped& to,
                                                           bool avoid_ring_zone) const
{
  // Detour strategy:
  // - 单点绕行，不做全局规划。
  // - 优先把绕行点放在“目标点前方”的同一条 x 上，即先斜飞到绕行点，再直飞目标点。
  //   这样比“先横移再前进”更不容易再次擦到障碍物。
  geometry_msgs::PoseStamped detour = to;

  const double ax = from.pose.position.x;
  const double ay = from.pose.position.y;
  const double bx = to.pose.position.x;
  const double by = to.pose.position.y;

  double detour_y = ay;

  // Recompute hit flags to decide which detour to apply.
  const auto obs = mapToWorld(obstacle_mm_, cruise_height_);
  const double obs_r = 0.5 * obstacle_size_m_ + obstacle_clearance_m_;
  const bool hit_obstacle = distPointToSeg2d(obs.x, obs.y, ax, ay, bx, by) < obs_r;

  Rect2D ring_rect;
  const bool want_ring = avoid_ring_zone;
  if (want_ring)
  {
    const MapPointMm p1{6300.0, 6000.0};
    const MapPointMm p2{8700.0, 4200.0};
    const auto e1 = mapToWorld(p1, 0.0);
    const auto e2 = mapToWorld(p2, 0.0);
    ring_rect.min_x = std::min(e1.x, e2.x);
    ring_rect.max_x = std::max(e1.x, e2.x);
    ring_rect.min_y = std::min(e1.y, e2.y);
    ring_rect.max_y = std::max(e1.y, e2.y);
  }
  const bool hit_ring_rect = want_ring && segIntersectsRect2d(ax, ay, bx, by, ring_rect);

  auto candidateIsSafe = [&](double cand_y) -> bool {
    geometry_msgs::PoseStamped cand = to;
    cand.pose.position.x = to.pose.position.x;
    cand.pose.position.y = cand_y;
    cand.pose.position.z = to.pose.position.z;
    return !needsDetour(from, cand, avoid_ring_zone) && !needsDetour(cand, to, avoid_ring_zone);
  };

  bool found = false;
  if (hit_obstacle)
  {
    const double up_y = obs.y + (obs_r + 0.5);
    const double down_y = obs.y - (obs_r + 0.5);
    if (candidateIsSafe(up_y))
    {
      detour_y = up_y;
      found = true;
    }
    else if (candidateIsSafe(down_y))
    {
      detour_y = down_y;
      found = true;
    }
  }
  if (!found && hit_ring_rect)
  {
    // Go above or below the rectangle boundary and prefer the side that is path-safe.
    const double above = ring_rect.max_y + 0.5;
    const double below = ring_rect.min_y - 0.5;
    if (candidateIsSafe(above))
    {
      detour_y = above;
      found = true;
    }
    else if (candidateIsSafe(below))
    {
      detour_y = below;
      found = true;
    }
  }

  if (!found)
  {
    // Fallback: keep a conservative y-offset if both preferred candidates still intersect.
    if (hit_obstacle)
    {
      const double dir = (ay >= obs.y) ? 1.0 : -1.0;
      detour_y = obs.y + dir * (obs_r + 0.5);
    }
    else if (hit_ring_rect)
    {
      detour_y = (ay > ring_rect.max_y) ? (ring_rect.max_y + 0.5) : (ring_rect.min_y - 0.5);
    }
  }

  detour.pose.position.x = to.pose.position.x;
  detour.pose.position.y = detour_y;
  return detour;
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

void MissionController::rcInCallback(const mavros_msgs::RCIn::ConstPtr& msg)
{
  last_rc_ = *msg;
  last_rc_time_ = ros::Time::now();
}

MissionController::RcDropSelection MissionController::parseRcDropSelection(const mavros_msgs::RCIn& rc_msgs)
{
  // - ok=false 表示不更新selected_drop1_/2_和selected_land_side_
  // - 你后续在这里从/mavros/rc/in解析出：
  //   1) 两个投放点(IMAGE_n)
  //   2) 最终降落左/右("left"/"right")
  auto segment = [](int x) -> int{
    int ans = 1;
    if(x >= 999 && x <= 1010){
      ans=1;
    }else if( x >1010 && x <= 1499){
      ans=2;
    }else if( x > 1499 && x <= 1980){
      ans=3;
    }else if( x > 1980 && x <= 1999){
      ans=4;
    }else{
      ans=0;
    }
    return ans;
  };
  RcDropSelection out;

  // 注意：该函数在头文件中被声明为static，因此这里不能访问任何成员变量
  // （例如 rc_drop_select_enabled_）。开关逻辑应在调用处处理。

  if (rc_msgs.channels.size() <= 8)
  {
    out.ok = false;
    return out;
  }

  out.drop1_image = segment(rc_msgs.channels[7]);
  out.drop2_image = segment(rc_msgs.channels[8]);
  out.land_side = (rc_msgs.channels[6] == 999) ? std::string("left") : std::string("right");

  // 合法性：投放点必须在[1..4]范围内
  out.ok = (out.drop1_image >= 1 && out.drop1_image <= 4 && out.drop2_image >= 1 && out.drop2_image <= 4);
  return out;
}

void MissionController::resetImageHoverStage()
{
  image_hover_stage_ = ImageHoverStage::HOVER;
  image_hover_stage_start_time_ = ros::Time(0);
}

bool MissionController::tickHoverImageRcMode(int image_index, const ros::Time& now, double dt)
{
  // image_index: 1..4
  const int idx = std::max(1, std::min(4, image_index)) - 1;

  // 先判定本IMAGE是否需要投放（RC模式下允许drop1/drop2都落在同一张IMAGE）
  const bool need_drop1_here = (!servo_1_done_ && selected_drop1_ == image_index);
  const bool need_drop2_here = (!servo_2_done_ && selected_drop2_ == image_index);
  const bool need_any_drop_here = need_drop1_here || need_drop2_here;

  // 阶段计时起点（进入某阶段的第一帧）
  if (image_hover_stage_start_time_.isZero()) image_hover_stage_start_time_ = now;

  switch (image_hover_stage_)
  {
    case ImageHoverStage::HOVER:
    {
      // 1) 先在巡航高度悬停几秒（按hover_image_s_）
      // 2) 如果本IMAGE不需要投放，则直接完成（由外层进入下一个状态）
      // 注意：这里不做水平移动，只保持当前位置（setpoint_）
      publishSetpointNow(setpoint_);

      if ((now - image_hover_stage_start_time_).toSec() < hover_image_s_) return false;

      if (!need_any_drop_here)
      {
        image_hover_stage_ = ImageHoverStage::DONE;
        return true;
      }

      image_hover_stage_ = ImageHoverStage::DESCEND;
      image_hover_stage_start_time_ = now;
      return false;
    }
    case ImageHoverStage::DESCEND:
    {
      // 原地下降到投放高度（不允许边移动边下降）
      const auto low = mapToWorld(image_targets_mm_.at(idx), drop_height_);
      goal_pose_ = makePose(low.x, low.y, low.z, 0.0);
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if (!reached(goal_pose_)) return false;

      image_hover_stage_ = ImageHoverStage::DROP;
      image_hover_stage_start_time_ = now;
      return false;
    }
    case ImageHoverStage::DROP:
    {
      // 执行投放（允许投放1/2任意组合）
      // 这里不再下发下降/上升目标，只维持当前位置（低高度点）
      publishSetpointNow(setpoint_);

      if (need_drop1_here)
      {
        startServoDrop(1, now);
        servo_1_done_ = true;
      }
      if (need_drop2_here)
      {
        startServoDrop(2, now);
        servo_2_done_ = true;
      }

      image_hover_stage_ = ImageHoverStage::ASCEND;
      image_hover_stage_start_time_ = now;
      return false;
    }
    case ImageHoverStage::ASCEND:
    {
      // 投放后必须先原地抬升回巡航高度，再进入后续移动
      const auto high = mapToWorld(image_targets_mm_.at(idx), cruise_height_);
      goal_pose_ = makePose(high.x, high.y, high.z, 0.0);
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if (!reached(goal_pose_)) return false;

      image_hover_stage_ = ImageHoverStage::DONE;
      return true;
    }
    case ImageHoverStage::DONE:
      return true;
  }

  return false;
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

namespace
{
geometry_msgs::Quaternion quatMultiply(const geometry_msgs::Quaternion& a, const geometry_msgs::Quaternion& b)
{
  geometry_msgs::Quaternion out;
  out.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  out.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  out.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  out.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return out;
}

geometry_msgs::Quaternion quatNormalize(const geometry_msgs::Quaternion& q)
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

geometry_msgs::PoseStamped compensateWToMavros(const geometry_msgs::PoseStamped& in,
                                               const geometry_msgs::Quaternion& q_rot_ccw90)
{
  // Observed behavior: commanding (x,y) in W results in (y,-x) in W (CW 90 deg).
  // Cancel by rotating setpoints CCW 90 deg before publishing to MAVROS.
  geometry_msgs::PoseStamped out = in;
  const double x = in.pose.position.x;
  const double y = in.pose.position.y;
  out.pose.position.x = -y;
  out.pose.position.y = x;
  out.pose.position.z = in.pose.position.z;

  out.pose.orientation = quatNormalize(quatMultiply(q_rot_ccw90, in.pose.orientation));
  return out;
}
}  // namespace

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

MissionController::WorldPoint MissionController::mapToWorld(const MapPointMm& p_mm, double z_m) const
{
  const double dx_m = (p_mm.x_mm - map_origin_mm_.x_mm) / 1000.0;
  double dy_m = (p_mm.y_mm - map_origin_mm_.y_mm) / 1000.0;
  if (map_to_w_y_inverted_) dy_m = -dy_m;
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

  // Publish raw W-frame setpoint for debugging/visualization.
  target_pose_pub_.publish(p);

  // Publish compensated setpoint to MAVROS.
  const geometry_msgs::Quaternion q_rot = yawToQuaternion(kHalfPi);  // +90deg about +Z
  const geometry_msgs::PoseStamped sp_mavros = compensateWToMavros(p, q_rot);
  setpoint_pub_.publish(sp_mavros);
}

void MissionController::enterPhase(Phase next, const std::string& reason)
{
  phase_ = next;
  phase_enter_time_ = ros::Time::now();
  segment_start_time_ = phase_enter_time_;
  detour_active_ = false;
  // HOVER_IMAGE使用内部小状态机：每次进入新的Phase都重置，避免跨IMAGE串状态
  resetImageHoverStage();
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

  // ODOM超时只在正常任务/起飞/移动阶段触发一次FAILSAFE_HOVER。
  // 进入FAILSAFE_HOVER后即使里程计仍然超时，也不能反复重置failsafe_start_time_，
  // 否则会一直悬停，永远等不到failsafe_hover_timeout_s_后的自动降落。
  const bool odom_timeout_sensitive =
      phase_ != Phase::WAIT_FCU && phase_ != Phase::WAIT_ODOM && phase_ != Phase::FAILSAFE_HOVER &&
      phase_ != Phase::WAIT_MANUAL && phase_ != Phase::LAND && phase_ != Phase::DONE;

  if (odom_timeout_sensitive && !odomHealthy(now))
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
      // Safety: do NOT fly to the obstacle center.
      // Fly to a nearby entry point on the circle with radius circle_radius_m_.
      // Use the "east" point (angle=0) as the default entry point: (cx + r, cy).
      const auto c = mapToWorld(obstacle_mm_, cruise_height_);
      final_pose_ = makePose(c.x + circle_radius_m_, c.y, c.z, 0.0);
      // While approaching obstacle entry point, still avoid ring zone if enabled.
      if (!detour_active_ && needsDetour(setpoint_, final_pose_, avoid_ring_zone_))
      {
        detour_pose_ = computeDetour(setpoint_, final_pose_, avoid_ring_zone_);
        detour_active_ = true;
      }
      goal_pose_ = detour_active_ ? detour_pose_ : final_pose_;
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if (reached(goal_pose_))
      {
        if (detour_active_)
        {
          detour_active_ = false;
          segment_start_time_ = now;
        }
        else
        {
          enterPhase(Phase::CIRCLE_OBSTACLE, "CIRCLE_OBSTACLE");
        }
      }
      break;
    }
    case Phase::CIRCLE_OBSTACLE:
    {
      const auto c = mapToWorld(obstacle_mm_, cruise_height_);
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
      const auto p = mapToWorld(qrcode_mm_, cruise_height_);
      final_pose_ = makePose(p.x, p.y, p.z, 0.0);
      if (!detour_active_ && needsDetour(setpoint_, final_pose_, avoid_ring_zone_))
      {
        detour_pose_ = computeDetour(setpoint_, final_pose_, avoid_ring_zone_);
        detour_active_ = true;
      }
      goal_pose_ = detour_active_ ? detour_pose_ : final_pose_;
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if (reached(goal_pose_))
      {
        if (detour_active_)
        {
          detour_active_ = false;  // switch to final on next tick
          segment_start_time_ = now;
        }
        else
        {
          enterPhase(Phase::HOVER_QRCODE, "HOVER_QRCODE");
        }
      }
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
      const auto p = mapToWorld(image_targets_mm_.at(0), cruise_height_);
      final_pose_ = makePose(p.x, p.y, p.z, 0.0);
      if (!detour_active_ && needsDetour(setpoint_, final_pose_, avoid_ring_zone_))
      {
        detour_pose_ = computeDetour(setpoint_, final_pose_, avoid_ring_zone_);
        detour_active_ = true;
      }
      goal_pose_ = detour_active_ ? detour_pose_ : final_pose_;
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if (reached(goal_pose_))
      {
        if (detour_active_)
        {
          detour_active_ = false;
          segment_start_time_ = now;
        }
        else
        {
          enterPhase(Phase::HOVER_IMAGE_1, "HOVER_IMAGE_1");
        }
      }
      break;
    }
    case Phase::HOVER_IMAGE_1:
    {
      // 在到达IMAGE_1后解析一次RC（如果你填了parseRcDropSelection），并锁定结果
      if (rc_drop_select_enabled_ && !rc_drop_selected_locked_)
      {
        const bool rc_fresh = !last_rc_time_.isZero() && (now - last_rc_time_).toSec() <= rc_timeout_s_;
        if (rc_fresh)
        {
          const RcDropSelection sel = parseRcDropSelection(last_rc_);
          if (sel.ok)
          {
            selected_drop1_ = std::max(1, std::min(4, sel.drop1_image));
            selected_drop2_ = std::max(1, std::min(4, sel.drop2_image));
            if (selected_drop1_ == selected_drop2_)
            {
              ROS_WARN("RC selected duplicate drop IMAGE_%d, fallback to IMAGE_1 and IMAGE_2", selected_drop1_);
              selected_drop1_ = 1;
              selected_drop2_ = 2;
            }
            selected_land_side_ = sel.land_side;
            ROS_INFO("RC selection locked: drop1=IMAGE_%d drop2=IMAGE_%d land_side=%s",
                     selected_drop1_, selected_drop2_, selected_land_side_.c_str());
          }
          else
          {
            ROS_WARN("RC selection parse failed, keep defaults drop1=IMAGE_%d drop2=IMAGE_%d land_side=%s",
                     selected_drop1_, selected_drop2_, selected_land_side_.c_str());
          }
        }
        else
        {
          ROS_WARN("RC selection not fresh at IMAGE_1, keep defaults drop1=IMAGE_%d drop2=IMAGE_%d land_side=%s",
                   selected_drop1_, selected_drop2_, selected_land_side_.c_str());
        }
        rc_drop_selected_locked_ = true;
      }

      if (!rc_drop_select_enabled_)
      {
        // 视觉模式：你说“先不管”，这里暂时保持最简单行为：原地悬停hover_image_s_后走下一段
        publishSetpointNow(setpoint_);
        if ((now - phase_enter_time_).toSec() >= hover_image_s_) enterPhase(Phase::GOTO_IMAGE_2, "GOTO_IMAGE_2");
        break;
      }

      if (tickHoverImageRcMode(1, now, dt)) enterPhase(Phase::GOTO_IMAGE_2, "GOTO_IMAGE_2");
      break;
    }
    case Phase::GOTO_IMAGE_2:
    {
      const auto p = mapToWorld(image_targets_mm_.at(1), cruise_height_);
      final_pose_ = makePose(p.x, p.y, p.z, 0.0);
      if (!detour_active_ && needsDetour(setpoint_, final_pose_, avoid_ring_zone_))
      {
        detour_pose_ = computeDetour(setpoint_, final_pose_, avoid_ring_zone_);
        detour_active_ = true;
      }
      goal_pose_ = detour_active_ ? detour_pose_ : final_pose_;
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if (reached(goal_pose_))
      {
        if (detour_active_)
        {
          detour_active_ = false;
          segment_start_time_ = now;
        }
        else
        {
          enterPhase(Phase::HOVER_IMAGE_2, "HOVER_IMAGE_2");
        }
      }
      break;
    }
    case Phase::HOVER_IMAGE_2:
    {
      if (!rc_drop_select_enabled_)
      {
        publishSetpointNow(setpoint_);
        if ((now - phase_enter_time_).toSec() >= hover_image_s_) enterPhase(Phase::GOTO_IMAGE_3, "GOTO_IMAGE_3");
        break;
      }

      if (tickHoverImageRcMode(2, now, dt)) enterPhase(Phase::GOTO_IMAGE_3, "GOTO_IMAGE_3");
      break;
    }
    case Phase::GOTO_IMAGE_3:
    {
      const auto p = mapToWorld(image_targets_mm_.at(2), cruise_height_);
      final_pose_ = makePose(p.x, p.y, p.z, 0.0);
      if (!detour_active_ && needsDetour(setpoint_, final_pose_, avoid_ring_zone_))
      {
        detour_pose_ = computeDetour(setpoint_, final_pose_, avoid_ring_zone_);
        detour_active_ = true;
      }
      goal_pose_ = detour_active_ ? detour_pose_ : final_pose_;
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if (reached(goal_pose_))
      {
        if (detour_active_)
        {
          detour_active_ = false;
          segment_start_time_ = now;
        }
        else
        {
          enterPhase(Phase::HOVER_IMAGE_3, "HOVER_IMAGE_3");
        }
      }
      break;
    }
    case Phase::HOVER_IMAGE_3:
    {
      if (!rc_drop_select_enabled_)
      {
        publishSetpointNow(setpoint_);
        if ((now - phase_enter_time_).toSec() >= hover_image_s_) enterPhase(Phase::GOTO_IMAGE_4, "GOTO_IMAGE_4");
        break;
      }

      if (tickHoverImageRcMode(3, now, dt)) enterPhase(Phase::GOTO_IMAGE_4, "GOTO_IMAGE_4");
      break;
    }
    case Phase::GOTO_IMAGE_4:
    {
      const auto p = mapToWorld(image_targets_mm_.at(3), cruise_height_);
      final_pose_ = makePose(p.x, p.y, p.z, 0.0);
      if (!detour_active_ && needsDetour(setpoint_, final_pose_, avoid_ring_zone_))
      {
        detour_pose_ = computeDetour(setpoint_, final_pose_, avoid_ring_zone_);
        detour_active_ = true;
      }
      goal_pose_ = detour_active_ ? detour_pose_ : final_pose_;
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if (reached(goal_pose_))
      {
        if (detour_active_)
        {
          detour_active_ = false;
          segment_start_time_ = now;
        }
        else
        {
          enterPhase(Phase::HOVER_IMAGE_4, "HOVER_IMAGE_4");
        }
      }
      break;
    }
    case Phase::HOVER_IMAGE_4:
    {
      if (!rc_drop_select_enabled_)
      {
        publishSetpointNow(setpoint_);
        if ((now - phase_enter_time_).toSec() >= hover_image_s_) enterPhase(Phase::GOTO_SPECIAL, "GOTO_SPECIAL");
        break;
      }

      if (tickHoverImageRcMode(4, now, dt)) enterPhase(Phase::GOTO_SPECIAL, "GOTO_SPECIAL");
      break;
    }
    case Phase::GOTO_SPECIAL:
    {
      const auto p = mapToWorld(special_target_mm_, cruise_height_);
      final_pose_ = makePose(p.x, p.y, p.z, 0.0);
      if (!detour_active_ && needsDetour(setpoint_, final_pose_, avoid_ring_zone_))
      {
        detour_pose_ = computeDetour(setpoint_, final_pose_, avoid_ring_zone_);
        detour_active_ = true;
      }
      goal_pose_ = detour_active_ ? detour_pose_ : final_pose_;
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if (reached(goal_pose_))
      {
        if (detour_active_)
        {
          detour_active_ = false;
          segment_start_time_ = now;
        }
        else
        {
          enterPhase(Phase::HOVER_SPECIAL_DROP_3, "HOVER_SPECIAL_DROP_3");
        }
      }
      break;
    }
    case Phase::HOVER_SPECIAL_DROP_3:
    {
      if (!servo_3_done_)
      {
        const auto low = mapToWorld(special_target_mm_, drop_height_);
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

      const auto p = mapToWorld(special_target_mm_, cruise_height_);
      goal_pose_ = makePose(p.x, p.y, p.z, 0.0);
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if ((now - phase_enter_time_).toSec() >= hover_special_s_) enterPhase(Phase::GOTO_RING_VIEW, "GOTO_RING_VIEW");
      break;
    }
    case Phase::GOTO_RING_VIEW:
    {
      const auto p = mapToWorld(ring_view_mm_, cruise_height_);
      const double yaw = map_to_w_y_inverted_ ? -kHalfPi : kHalfPi;  // face map +Y
      final_pose_ = makePose(p.x, p.y, p.z, yaw);
      if (!detour_active_ && needsDetour(setpoint_, final_pose_, avoid_ring_zone_))
      {
        detour_pose_ = computeDetour(setpoint_, final_pose_, avoid_ring_zone_);
        detour_active_ = true;
      }
      goal_pose_ = detour_active_ ? detour_pose_ : final_pose_;
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if (reached(goal_pose_))
      {
        if (detour_active_)
        {
          detour_active_ = false;
          segment_start_time_ = now;
        }
        else
        {
          enterPhase(Phase::HOVER_RING_VIEW, "HOVER_RING_VIEW");
        }
      }
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
      const auto ring = mapToWorld(ring_default_mm_, ring_height_);
      final_pose_ = makePose(ring.x, ring.y, ring.z, 0.0);
      // PASS_RING is intended to go through the ring zone, so disable ring-zone avoidance here.
      const bool avoid_ring = false;
      if (!detour_active_ && needsDetour(setpoint_, final_pose_, avoid_ring))
      {
        detour_pose_ = computeDetour(setpoint_, final_pose_, avoid_ring);
        detour_active_ = true;
      }
      goal_pose_ = detour_active_ ? detour_pose_ : final_pose_;
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if (reached(goal_pose_))
      {
        if (detour_active_)
        {
          detour_active_ = false;
          segment_start_time_ = now;
        }
        else
        {
          enterPhase(Phase::GOTO_LAND, "GOTO_LAND");
        }
      }
      break;
    }
    case Phase::GOTO_LAND:
    {
      // Land side can be selected by RC (selected_land_side_), otherwise uses default_land_side_.
      const std::string land_side = selected_land_side_.empty() ? default_land_side_ : selected_land_side_;
      const MapPointMm land = (land_side == "right") ? land_right_mm_ : land_left_mm_;
      const auto p = mapToWorld(land, cruise_height_);
      final_pose_ = makePose(p.x, p.y, p.z, 0.0);
      if (!detour_active_ && needsDetour(setpoint_, final_pose_, avoid_ring_zone_))
      {
        detour_pose_ = computeDetour(setpoint_, final_pose_, avoid_ring_zone_);
        detour_active_ = true;
      }
      goal_pose_ = detour_active_ ? detour_pose_ : final_pose_;
      setpoint_ = stepToward(setpoint_, goal_pose_, dt);
      publishSetpointNow(setpoint_);
      if (reached(goal_pose_))
      {
        if (detour_active_)
        {
          detour_active_ = false;
          segment_start_time_ = now;
        }
        else
        {
          enterPhase(Phase::LAND, "LAND");
        }
      }
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
