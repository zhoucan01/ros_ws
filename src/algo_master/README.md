# algo_master

`algo_master` 是导航系统和下位机串口协议之间的桥接节点，负责把 ROS 侧的导航/状态信息下发给下位机，同时把下位机回传的导航帧、决策帧转换成 ROS 话题。

当前版本里，它除了原来的速度、位置、路径状态回传之外，还已经接入了过洞聚合消息 `TunnelMonitor`，可以把“是否需要过洞”和“过洞目标朝向差值”继续转发到下位机。

另外，当前版本还打通了两条姿态相关链路：

- 下发链路：行为树期望姿态 -> `NavigationPLCSendMsg.sentry_attitude_switch`
- 上行链路：下位机/裁判系统真实姿态 -> `RefereeRaw.msg.real_sentry_attitude_switch`

## 1. 包内结构

```text
algo_master/
├─ config/
│  └─ algo_master_params.yaml
├─ configs/
│  └─ serial_config.json
├─ include/algo_master/
│  ├─ circular_buffer.hpp
│  ├─ constants.hpp
│  └─ serialport.hpp
├─ launch/
│  └─ algo_master_launch.py
├─ msg/
│  ├─ PLC2Imu.msg
│  └─ PLC2Target.msg
└─ src/
   └─ algo_master_node.cpp
```

主要文件：

- 节点实现：[algo_master_node.cpp](/home/zc/ros_ws/src/algo_master/src/algo_master_node.cpp)
- 串口协议结构体：[serialport.hpp](/home/zc/ros_ws/src/algo_master/include/algo_master/serialport.hpp)
- launch 文件：[algo_master_launch.py](/home/zc/ros_ws/src/algo_master/launch/algo_master_launch.py)

## 2. 当前节点做什么

`algo_master_node` 当前主要做这几件事：

1. 从串口读取导航帧和决策帧。
2. 把串口数据发布成 ROS 话题，供导航、行为树和可视化使用。
3. 订阅 `cmd_vel`、`odometry`、`plan`、`plc2target`、`TunnelMonitor` 等话题。
4. 把底盘控制量、当前位置、路径状态、过洞状态打包回传给下位机。
5. 可选地把 `plc2target` 转成 Nav2 的 `goal_pose` 或 `NavigateToPose` action。

节点名：

- `algo_master`

默认 launch 命名空间：

- `red_standard_robot1`

## 3. 串口接收协议

串口定义在 [serialport.hpp](/home/zc/ros_ws/src/algo_master/include/algo_master/serialport.hpp)。

### 3.1 导航接收帧 `NavSerialMsg`

```text
0xA5
msg_id = 0x01
target_msg
imu_msg
Enemy_Pos
0xAA
```

其中：

- `target_msg`
  - `float x`
  - `float y`
- `imu_msg`
  - `float m_ImuYaw`
  - `float m_ImuPitch`
- `Enemy_Pos`
  - `uint8 If_vision_on`
  - `int16 Enemy_x`
  - `int16 Enemy_y`

### 3.2 决策接收帧 `DecisionSerialMsg`

```text
0xA5
msg_id = 0x02
Decision_Data
Referee_Raw_Data
0xAA
```

`Decision_Data` 包含：

- `Decision_data_1`
- `Decision_data_2`
- `Decision_data_3`
- `Decision_data_4`
- `game_remain_time`
- `game_state`

`Referee_Raw_Data` 包含：

- `uint16 projectile_allowance_17mm`
- `uint16 current_hp`
- `uint16 my_base_hp`
- `uint16 we_outpost_hp`
- `uint16 enemy_outpost_hp`
- `uint16 game_remain_time`
- `uint8 game_state`
- `uint8 real_sentry_attitude_switch`
- `int16 enemy_hero_x`
- `int16 enemy_hero_y`

接收线程流程：

1. `ReadRawBuf()` 读取原始字节流。
2. `ProcRawBuf()` 按帧头 `0xA5`、`msg_id`、固定长度、帧尾 `0xAA` 解包。
3. 导航帧进入 `msgSerialNavRecv`。
4. 决策帧进入 `msgSerialDecisionRecv`。

## 4. 串口发送协议

发送结构体是 `NavigationPLCSendMsg`。

当前字段包括：

- `linear_vel_x`
- `linear_vel_y`
- `angular_z`
- `cur_x`
- `cur_y`
- `has_path_`
- `get_goal`
- `if_control`
- `seq`
- `arrive_flag`
- `close_flag`
- `need_tunnel`
- `tunnel_yaw_error`
- `if_on_attack`
- `sentry_attitude_switch`

其中这两个是本轮新增字段：

- `need_tunnel`
  - 当前是否需要进入过洞相关模式
  - 目前定义为 `will_pass_tunnel || in_tunnel || tunnel_recovery_active`
- `tunnel_yaw_error`
  - 当前过洞目标朝向与机器人当前朝向的差值
  - 直接来自 `TunnelMonitor.tunnel_yaw_error`

另外还有：

- `if_on_attack`
  - 当前最终输出目标是否为追击目标
- `sentry_attitude_switch`
  - 上位机当前下发给下位机的目标姿态

发送触发时机：

- 订阅到 `cmd_vel` 时，立即组包并通过串口发送给下位机。

## 5. 过洞数据是怎么接进来的

`algo_master` 现在订阅：

- `sentry_decision_msg/msg/TunnelMonitor`

默认订阅话题：

- `tunnel_monitor`

代码位置：

- [algo_master_node.cpp](/home/zc/ros_ws/src/algo_master/src/algo_master_node.cpp)

当前接入逻辑：

- `need_tunnel = will_pass_tunnel || in_tunnel || tunnel_recovery_active`
- `tunnel_yaw_error = TunnelMonitor.tunnel_yaw_error`

然后这两个值会被写进 `NavigationPLCSendMsg`，跟随 `cmd_vel` 一起下发给下位机。

## 6. ROS 接口

### 6.1 发布的话题

- `plc2imu`
  - 类型：`algo_master/msg/PLC2Imu`
  - 来源：导航串口帧中的 `imu_msg`

- `serial/gimbal_joint_state`
  - 类型：`sensor_msgs/msg/JointState`
  - 来源：`plc2imu`
  - 用于云台关节可视化

- `decision_msg`
  - 类型：`sentry_decision_msg/msg/SentryDecision`
  - 来源：决策串口帧中的 `Decision_Data`
  - 当前主要保留“下位机已经算好的决策 bool”

- `referee_raw_msg`
  - 类型：`sentry_decision_msg/msg/RefereeRaw`
  - 来源：决策串口帧中的 `Referee_Raw_Data`
  - 当前同时承载裁判系统原始量、比赛时间/状态，以及下位机回传的真实姿态 `real_sentry_attitude_switch`

- `enemy_msg`
  - 类型：`sentry_decision_msg/msg/EnemyPos`
  - 来源：导航串口帧中的 `Enemy_Pos`

- `current_pos_msg`
  - 类型：`geometry_msgs/msg/PointStamped`
  - 来源：`odometry -> tf -> map`

- `goal_pose`
  - 类型：`geometry_msgs/msg/PoseStamped`
  - 来源：`plc2target`
  - 仅在 `use_goal_pose_topic = true` 时发布

### 6.2 订阅的话题

- `cmd_vel`
  - 类型：`geometry_msgs/msg/Twist`
  - 用于串口回传底盘控制量

- `plc2target`
  - 类型：`algo_master/msg/PLC2Target`
  - 决策层给出的导航目标

- `plc2imu`
  - 类型：`algo_master/msg/PLC2Imu`
  - 内部再次订阅，用来转成 `JointState`

- `odometry`
  - 类型：`nav_msgs/msg/Odometry`
  - 当前代码会从 `odom` 通过 TF 转换到 `map`

- `plan`
  - 类型：`nav_msgs/msg/Path`
  - 用来判断当前是否存在全局路径

- `global_costmap/costmap`
  - 类型：`nav_msgs/msg/OccupancyGrid`
  - 当前只取了 `frame_id`

- `tunnel_monitor`
  - 类型：`sentry_decision_msg/msg/TunnelMonitor`
  - 用来获取过洞状态和朝向差值，并继续转发到下位机

- `attitude_switch`
  - 类型：`sentry_decision_msg/msg/AttitudeSwitch`
  - 用来接收行为树期望姿态，并继续转发到下位机串口发送包

### 6.3 Action Client

- `navigate_to_pose`
  - 类型：`nav2_msgs/action/NavigateToPose`
  - 仅在 `use_action_goal = true` 时发送

## 7. 参数

参数文件在：[algo_master_params.yaml](/home/zc/ros_ws/src/algo_master/config/algo_master_params.yaml)

当前主要参数：

- `arrived_threshold`
  - 到点阈值
  - 默认 `0.3`

- `close_threshold`
  - 接近阈值
  - 默认 `1.0`

- `use_goal_pose_topic`
  - 是否把 `plc2target` 转成 `goal_pose`
  - 默认 `true`

- `use_action_goal`
  - 是否直接调用 Nav2 的 `NavigateToPose`
  - 默认 `true`

- `tunnel_monitor_topic`
  - 过洞聚合消息订阅话题
  - 默认 `tunnel_monitor`

串口参数文件在：[serial_config.json](/home/zc/ros_ws/src/algo_master/configs/serial_config.json)

主要字段：

- `portName`
- `baudrate`
- `parity`
- `dataBit`
- `stopBit`
- `synchronize`
- `sendInterval`

## 8. 运行方式

### 8.1 使用 launch

```bash
ros2 launch algo_master algo_master_launch.py
```

可传参数：

```bash
ros2 launch algo_master algo_master_launch.py \
  namespace:=red_standard_robot1 \
  params_file:=/your/path/algo_master_params.yaml
```

### 8.2 单独运行节点

```bash
ros2 run algo_master algo_master_node --ros-args -r __ns:=/red_standard_robot1
```

## 9. 和下位机对接时必须注意的事

这次改动会影响串口发送结构体长度。

原因是 `NavigationPLCSendMsg` 新增了：

- `uint8_t need_tunnel`
- `float tunnel_yaw_error`

这意味着：

- 如果下位机还按旧结构体解析，后面的字段会错位
- 下位机必须同步修改协议定义和解包顺序

建议下位机同学同步检查：

1. 结构体字段顺序是否完全一致。
2. 是否仍然按 1 字节对齐解析。
3. 总帧长是否同步更新。
4. `need_tunnel` 是否按 `uint8_t` 处理。
5. `tunnel_yaw_error` 是否按 `float` 处理。

## 10. 一句话总结

`algo_master` 现在除了原来的导航串口桥接功能外，还已经把过洞逻辑的关键输出接进来了：

- 是否需要过洞：`need_tunnel`
- 过洞目标朝向差值：`tunnel_yaw_error`

这样下位机只要继续收 `NavigationPLCSendMsg`，就能直接拿到过洞相关控制参考。 
