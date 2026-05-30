# uav_main_controller

ROS1 Noetic 主控包：基于 MAVROS/PX4 的 OFFBOARD 位置控制，按任务流程执行绕障、悬停占位、3 次投放、圆环通过、降落。

## 节点

- `mission_controller_node`：发布 `/mavros/setpoint_position/local`，并通过 `/mavros/set_mode`、`/mavros/cmd/arming` 控制 PX4。

## 配置

- `config/mission.yaml`
  - `odom_topic`：默认 `/Odometry`，即 `odom_to_pose` 输出的 W 世界系里程计。
  - 高度约束：`cruise_height`（建议 1.4m，且 >= 1.2m），`drop_height`（建议 0.6m，且 <= 0.8m）。
  - 任务点均为“地图坐标系(mm)”：如 `takeoff_mm`、`obstacle_mm`、`image_targets_mm` 等。
  - 坐标转换约定：地图 `x` 与世界系 `W` 的 `x` 对齐；地图 `y` 与世界系 `W` 的 `y` 反向（`map_to_w_y_inverted: true`）。
  - RC投放选择：如果两个投放点解析到同一个 `IMAGE_n`，自动回退为 `IMAGE_1` 和 `IMAGE_2`。

## 投放接口

- 发布话题：`/servo_control`（`servo_controller/ServoCmd`）
  - `servo_id`: 1/2/3
  - `position`: 1 打开，0 关闭

## 异常处理

- 里程计超时或分段超时：进入 `FAILSAFE_HOVER` 悬停等待接管；超过 `failsafe_hover_timeout_s` 后自动切 `LAND`。
- 人为切出 `OFFBOARD`：进入 `WAIT_MANUAL`，不再“抢控制”，等待人工处理/接管。

## 启动

```bash
cd /home/demo/uav_ws
source devel/setup.bash
roslaunch uav_main_controller mission.launch
```
