#ifndef UAV_MAIN_CONTROLLER_MISSION_CONTROLLER_H
#define UAV_MAIN_CONTROLLER_MISSION_CONTROLLER_H

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <servo_controller/ServoCmd.h>
#include <std_msgs/String.h>

#include <cstdint>
#include <string>
#include <vector>

class MissionController
{
public:
  explicit MissionController(ros::NodeHandle& nh);
  void run();

  struct MapPointMm
  {
    double x_mm{0.0};
    double y_mm{0.0};
  };

  struct EnuPoint
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  struct Rect2D
  {
    double min_x{0.0};
    double max_x{0.0};
    double min_y{0.0};
    double max_y{0.0};
  };

private:
  enum class Phase
  {
    WAIT_FCU,
    WAIT_ODOM,
    PREOFFBOARD_STREAM,
    SET_OFFBOARD,
    ARM,
    TAKEOFF,
    GOTO_OBSTACLE,
    CIRCLE_OBSTACLE,
    GOTO_QRCODE,
    HOVER_QRCODE,
    GOTO_IMAGE_1,
    HOVER_IMAGE_1,
    GOTO_IMAGE_2,
    HOVER_IMAGE_2,
    GOTO_IMAGE_3,
    HOVER_IMAGE_3,
    GOTO_IMAGE_4,
    HOVER_IMAGE_4,
    GOTO_SPECIAL,
    HOVER_SPECIAL_DROP_3,
    GOTO_RING_VIEW,
    HOVER_RING_VIEW,
    PASS_RING,
    GOTO_LAND,
    LAND,
    FAILSAFE_HOVER,
    WAIT_MANUAL,
    DONE
  };

  void stateCallback(const mavros_msgs::State::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void rcInCallback(const mavros_msgs::RCIn::ConstPtr& msg);

  bool setMode(const std::string& mode);
  bool arm(bool arm_value);

  void publishMissionState(const std::string& text);
  void publishSetpointNow(const geometry_msgs::PoseStamped& pose);
  geometry_msgs::PoseStamped makePose(double x, double y, double z, double yaw_rad) const;
  static geometry_msgs::Quaternion yawToQuaternion(double yaw_rad);

  EnuPoint mapToEnu(const MapPointMm& p_mm, double z_m) const;
  bool reached(const geometry_msgs::PoseStamped& target) const;
  geometry_msgs::PoseStamped stepToward(const geometry_msgs::PoseStamped& from,
                                        const geometry_msgs::PoseStamped& to,
                                        double dt) const;
  bool odomHealthy(const ros::Time& now) const;

  // Simple path planning helpers (2D)
  static double distPointToSeg2d(double px, double py, double ax, double ay, double bx, double by);
  static bool segIntersectsRect2d(double ax, double ay, double bx, double by, const Rect2D& r);
  bool needsDetour(const geometry_msgs::PoseStamped& from,
                   const geometry_msgs::PoseStamped& to,
                   bool avoid_ring_zone) const;
  geometry_msgs::PoseStamped computeDetour(const geometry_msgs::PoseStamped& from,
                                          const geometry_msgs::PoseStamped& to,
                                          bool avoid_ring_zone) const;

  void enterPhase(Phase next, const std::string& reason);
  void tick(const ros::Time& now, double dt);
  void triggerFailsafeHover(const std::string& reason);

  void startServoDrop(int8_t servo_id, const ros::Time& now);
  void tickServo(const ros::Time& now);

  // -----------------------------
  // RC解析（单独拎出来，你后续填充实现）
  // -----------------------------
  struct RcDropSelection
  {
    bool ok{false};
    int drop1_image{1};                 // 1..4
    int drop2_image{2};                 // 1..4
    std::string land_side{"left"};      // "left" or "right"
  };
  static RcDropSelection parseRcDropSelection(const mavros_msgs::RCIn& rc);

  // 统一的IMAGE悬停/投放处理（RC模式）：image_index = 1..4
  // 返回true表示本IMAGE流程已完成，可以切换到下一段航线；false表示仍在本阶段处理中。
  bool tickHoverImageRcMode(int image_index, const ros::Time& now, double dt);
  void resetImageHoverStage();

private:
  enum class ImageHoverStage
  {
    HOVER,    // 在巡航高度悬停一段时间
    DESCEND,  // 原地下降到投放高度
    DROP,     // 执行投放（可投放1/2任意组合）
    ASCEND,   // 原地抬升回巡航高度
    DONE      // 本IMAGE处理完成
  };

  ros::NodeHandle nh_;
  ros::Subscriber state_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber rc_sub_;
  ros::Publisher setpoint_pub_;
  ros::Publisher mission_state_pub_;
  ros::Publisher target_pose_pub_;
  ros::Publisher servo_pub_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceClient arm_client_;

  mavros_msgs::State current_state_;
  nav_msgs::Odometry current_odom_;
  ros::Time last_odom_time_;
  mavros_msgs::RCIn last_rc_;
  ros::Time last_rc_time_;

  std::string odom_topic_{"/Odometry"};
  double control_rate_{20.0};
  double reach_tol_xy_{0.25};
  double reach_tol_z_{0.20};
  double prestream_time_s_{2.0};
  double max_xy_speed_{1.0};
  double max_z_speed_{0.6};
  double cruise_height_{1.4};
  double drop_height_{0.6};
  double ring_height_{1.6};

  double segment_timeout_s_{60.0};
  double circle_duration_s_{10.0};
  double circle_radius_m_{1.2};
  double circle_period_s_{10.0};
  double hover_qr_s_{3.0};
  double hover_image_s_{3.0};
  double hover_special_s_{3.0};
  double hover_ring_view_s_{5.0};
  double servo_open_time_s_{0.8};

  double failsafe_hover_timeout_s_{10.0};
  double odom_timeout_s_{0.5};

  MapPointMm map_origin_mm_{1500.0, 3000.0};
  bool map_to_enu_y_inverted_{true};

  MapPointMm takeoff_mm_{1500.0, 3000.0};
  MapPointMm obstacle_mm_{5800.0, 3000.0};
  MapPointMm qrcode_mm_{3300.0, 3000.0};
  std::vector<MapPointMm> image_targets_mm_;
  MapPointMm special_target_mm_{7500.0, 2000.0};
  MapPointMm ring_view_mm_{7500.0, 4000.0};
  MapPointMm ring_default_mm_{7500.0, 4600.0};
  MapPointMm land_left_mm_{1500.0, 1400.0};
  MapPointMm land_right_mm_{1500.0, 4600.0};
  std::string default_land_side_{"left"};

  Phase phase_{Phase::WAIT_FCU};
  ros::Time phase_enter_time_;
  ros::Time segment_start_time_;
  ros::Time failsafe_start_time_;
  ros::Time last_offboard_request_time_;
  Phase resume_phase_{Phase::WAIT_FCU};

  geometry_msgs::PoseStamped setpoint_;
  geometry_msgs::PoseStamped goal_pose_;
  geometry_msgs::PoseStamped hold_pose_;

  bool servo_1_done_{false};
  bool servo_2_done_{false};
  bool servo_3_done_{false};

  int8_t active_servo_id_{0};
  bool servo_open_{false};
  ros::Time servo_open_until_;

  // Avoidance parameters
  double obstacle_size_m_{0.6};          // obstacle footprint (square), meters
  double obstacle_clearance_m_{0.4};     // extra clearance, meters
  bool avoid_ring_zone_{true};

  // Two-segment path following for detours
  bool detour_active_{false};
  geometry_msgs::PoseStamped detour_pose_;
  geometry_msgs::PoseStamped final_pose_;

  // Drop target selection (IMAGE_n) via RC at HOVER_QRCODE
  bool rc_drop_select_enabled_{true};
  double rc_timeout_s_{1.0};
  int selected_drop1_{1};  // IMAGE index 1..4
  int selected_drop2_{2};  // IMAGE index 1..4
  // RC选择只在首次到达IMAGE_1时解析并锁定，后续不再更新（避免飞行中抖动）。
  bool rc_drop_selected_locked_{false};

  // 最终降落左右（RC解析得到；默认使用mission.yaml里的default_land_side）
  std::string selected_land_side_;

  // HOVER_IMAGE内部小状态机（仅RC模式用）
  ImageHoverStage image_hover_stage_{ImageHoverStage::HOVER};
  ros::Time image_hover_stage_start_time_;
};

#endif
