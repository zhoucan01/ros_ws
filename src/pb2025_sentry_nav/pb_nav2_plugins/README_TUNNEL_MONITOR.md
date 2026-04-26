# tunnel_region_monitor 说明

## 1. 这个节点是干什么的

`tunnel_region_monitor` 是一个“过洞辅助决策节点”，它本身不直接控制底盘，也不直接发串口命令，主要负责：

- 从 TF 获取机器人当前在 `map` 系下的位置和朝向
- 根据全局路径 `plan` 判断这次导航是否要经过某个狗洞
- 判断机器人当前处于洞外、洞口准备区还是洞体内部
- 计算过洞方向 `tunnel_target_yaw`
- 在长时间卡住时触发恢复逻辑，生成一个“退回入口外”的临时导航目标
- 发布调试可视化，方便在 RViz 中看 trigger、tunnel、方向箭头和 recovery 点

可以把它理解成：  
导航系统里的“过洞状态监控器 + 恢复触发器”。

## 2. 文件位置

- 节点实现：
  [src/tunnel_region_monitor.cpp](./src/tunnel_region_monitor.cpp)
- BT 临时切换目标：
  [src/bt_nodes/select_pose_by_topic.cpp](./src/bt_nodes/select_pose_by_topic.cpp)
- `NavigateToPose` 行为树：
  [navigate_to_pose_w_replanning_and_recovery.xml](../pb2025_nav_bringup/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml)
- 参数配置：
  [reality/nav2_params.yaml](../pb2025_nav_bringup/config/reality/nav2_params.yaml)
  [simulation/nav2_params.yaml](../pb2025_nav_bringup/config/simulation/nav2_params.yaml)
- `sentry_bt` 订阅配置：
  [sentry_bt_params.yaml](../../sentry_bt/config/sentry_bt_params.yaml)

## 3. trigger 和 tunnel 是什么

这套逻辑里一个洞有两层区域：

- `trigger`
  含义是“过洞准备区”
- `tunnel`
  含义是“真正洞体区”

建议理解为：

- `trigger` 用来提前准备
  机器人靠近洞口时就能提前进入过洞模式，而不用等到完全钻进洞里
- `tunnel` 用来确认已经进洞
  只有进入这个区域才认为 `in_tunnel = true`

常见用法：

- `trigger` 稍微画大一点
  覆盖洞口前后少量范围
- `tunnel` 画成真正通道本体

这些区域参数需要严格按洞编号一一对应。
如果 `trigger / tunnel / entry / exit` 任意一组数组长度不一致，当前实现会在启动阶段直接 `fatal + 退出`，不会再自动补齐。

## 4. 整体工作流程

节点的主循环在 `update()` 里，大致流程如下：

1. 从 TF 查询 `map -> gimbal_yaw_fake`
2. 更新机器人当前全局坐标和朝向
3. 判断是否进入某个 `tunnel` 区域
4. 根据当前全局路径判断会经过哪个洞
5. 计算该洞的目标方向 `tunnel_target_yaw`
6. 判断是否已经进入 `trigger` 准备区
7. 如果长时间没有实际位移，则触发 recovery
8. 发布所有状态和可视化 marker

## 5. 目标洞是怎么选出来的

不是按配置顺序硬选第一个洞，而是按当前全局路径去扫：

- 遍历 `plan` 里的路径点
- 统计路径点是否落进某个洞的 `trigger` 区
- 只要命中点数达到 `min_path_points_in_region`，就认为这次路径会经过该洞
- 当前实现按洞编号顺序扫描，先满足阈值的那个洞会被选为目标洞

对应输出：

- `will_pass_tunnel`
  这次全局路径是否会经过洞
- `target_tunnel_id`
  当前选中的目标洞编号

这里有一个容易混淆的点：

- “选哪个洞”不是看机器人当前离哪个洞最近
- 而是看当前全局路径会穿过哪个洞的 `trigger` 区

也就是说，目标洞的来源是：

- 路径遍历命中 `trigger`
- 达到 `min_path_points_in_region`
- 该洞被认定为当前要通过的洞

后续状态机、方向计算、recovery 都是在这个“目标洞 / 跟踪洞”的基础上继续运行。

## 6. 过洞方向是怎么来的

每个洞都配置了一组入口点和出口点：

- `entry_xs / entry_ys`
- `exit_xs / exit_ys`

程序会先根据这两个点计算：

- `tunnel_yaw = entry -> exit`

然后在“刚进入 trigger 准备区”或“刚进入 tunnel”时记录当前是从哪一侧进入。

这个“进入侧”是通过距离判断出来的：

- 比较机器人当前位置到 `entry` 的距离
- 再比较机器人当前位置到 `exit` 的距离
- 如果离 `exit` 更近，就认为这次是从 `exit` 侧接近
- 否则就认为是从 `entry` 侧接近

所以要区分两个概念：

- 选洞：靠全局路径遍历
- 判断从哪一侧进洞：靠当前机器人到 `entry / exit` 的距离比较

后续正常过洞和 recovery 都优先沿这个“进入侧”确定方向，不会在洞中间因为离另一侧更近而把朝向突然翻转。

- `tunnel_target_yaw`

具体规则是：

- 如果是从 `entry` 侧接近，就使用 `entry -> exit` 的方向
- 如果是从 `exit` 侧接近，就把方向反过来，等价于 `exit -> entry`

这个值可以给底盘或云台做方向参考。 

另外还会发布：

- `current_map_yaw`
  当前机器人在 `map` 系下的 yaw
- `tunnel_yaw_error`
  `tunnel_target_yaw - current_map_yaw`

## 7. 卡住是怎么判断的

现在的判断方法不是看速度指令，而是看机器人在 `map` 系下有没有真实位移。

### 7.1 两个阶段

程序会区分两个阶段：

- `APPROACHING`
  已经进入 `trigger` 区，但还没进入 `tunnel`
- `IN_TUNNEL`
  已经进入 `tunnel`

在进入这些状态前，整体运行顺序可以概括为：

1. 从 TF 获取当前全局位姿
2. 扫描全局路径，判断这次导航是否会经过某个洞
3. 如果会经过，则把该洞设为 `target_tunnel_id`
4. 再根据机器人当前位置判断是否已经进入 `trigger` 或 `tunnel`
5. 在刚进入接近阶段或刚进入洞体时，记录“这次是从 entry 侧还是 exit 侧进入”
6. 根据进入侧计算 `tunnel_target_yaw`
7. 进入 `APPROACHING / IN_TUNNEL` 后开始持续监测是否真的有位移进展
8. 如果长时间没有位移进展，则触发 recovery

### 7.2 进展监测逻辑

进入上述任一阶段后，会启动一个“进展监测”：

- 记录一个参考位置
- 如果机器人当前位置相对参考位置移动超过 `min_progress_distance`
  - 就认为“有进展”
  - 刷新 `last_progress_time`
- 如果很久都没有刷新
  - 就认为卡住

### 7.3 两类超时

- `approach_stuck_timeout`
  洞口准备阶段超时
- `tunnel_stuck_timeout`
  洞内阶段超时

超时后会触发 recovery。

## 8. recovery 是怎么做的

### 8.1 recovery 的目标

这套 recovery 的含义不是“强行从另一头钻出去”，而是：

- 过洞失败
- 先退回进入侧外
- 再重新尝试进洞

### 8.2 recovery 目标点怎么算

`buildRecoveryGoal()` 当前策略是：

- 在刚进入 `trigger` 或刚进入 `tunnel` 时，先锁定“本次是从哪一侧进入”
- 如果是从 `entry` 侧进入，就沿 `entry -> exit` 的反方向回退
- 如果是从 `exit` 侧进入，就沿 `exit -> entry` 的反方向回退
- 回退距离统一使用 `recovery_retreat_distance`
- 最终生成一个“进入侧外”的回退点

所以 recovery goal 的意义是：

- 退回到本次进入的那一侧外面
- 不尝试在卡住后继续往另一头硬穿

### 8.3 recovery 期间 BT 做了什么

在 `NavigateToPose` 这棵 BT 里，加了一个 `SelectPoseByTopic` 节点：

- 正常时，规划目标还是原始 `{goal}`
- 当 `tunnel_recovery_active = true` 时
  - 临时把规划目标换成 `tunnel_recovery_goal`
- recovery 结束后
  - 自动切回原始 `{goal}`

所以恢复逻辑不是在 `algo_master` 里重发 goal，而是在 Nav2 的 BT 内部临时改用 recovery goal。

## 9. 与云台 / 底盘的关系

这个节点本身不直接控制云台和底盘。

它只发布状态与方向参考：

- `disable_spin`
- `in_tunnel`
- `tunnel_target_yaw`
- `tunnel_yaw_error`
- `tunnel_recovery_active`

推荐理解为：

- 正常过洞时
  底盘可参考 `tunnel_target_yaw`
- recovery 时
  底盘根据 Nav2 的 `tunnel_recovery_goal` 退回进入侧外
- 云台如果要保持过洞方向，可以继续看 `tunnel_target_yaw`

也就是说：

- `tunnel_recovery_goal` 主要服务底盘导航恢复
- `tunnel_target_yaw` 更适合服务云台/底盘的过洞方向参考

## 10. 主要输出话题

### 10.1 状态类

- `will_pass_tunnel`
  当前全局路径是否会经过洞
- `target_tunnel_id`
  当前目标洞编号
- `in_tunnel`
  当前是否位于洞体区域内
- `disable_spin`
  当前是否处于洞口准备区附近，建议禁用原地旋转
- `tunnel_status`
  过洞状态机状态
- `tunnel_recovery_active`
  是否处于 recovery 中
- `low_clearance_mode`
  是否处于“低顶极限洞”模式（仅对配置的洞编号生效）
- `controller_selector`
  当前建议使用的控制器 ID；普通阶段发布 `default_controller_id`，接近洞、进洞和 recovery 阶段发布 `tunnel_controller_id`
- `tunnel_monitor`
  聚合消息，使用 `sentry_decision_msg/msg/TunnelMonitor`，把核心状态、姿态和 recovery 目标统一打包发出

### 10.2 方向类

- `tunnel_target_yaw`
  当前目标过洞方向
- `current_map_yaw`
  当前 map 系 yaw
- `tunnel_yaw_error`
  方向误差

### 10.3 recovery 类

- `tunnel_recovery_goal`
  recovery 期间临时使用的回退目标

### 10.4 位姿类

- `global_pose`
  当前全局位姿
- `global_point`
  当前全局坐标点

### 10.5 调试类

- `tunnel_markers`
  RViz marker，可查看 trigger/tunnel/方向/recovery 点

## 11. 主要参数说明

### 11.1 开关

- `enable_tunnel_mode`
  是否启用过洞逻辑

如果设为 `false`：

- 不再判断目标洞
- 不再发布有效的 `disable_spin`
- 不再进入 recovery
- `controller_selector` 也退回默认控制器

此时就相当于普通导航。

### 11.1.1 聚合消息 topic

- `tunnel_monitor_topic`
  聚合消息发布话题名，默认是 `tunnel_monitor`
- `low_clearance_mode_topic`
  低顶极限模式标志话题，默认是 `low_clearance_mode`

当前实现会保留原有分散 topic，同时额外发布一个聚合消息，便于电控或上层策略节点一次性取全量状态。

如果你的下游节点已经统一改成订阅 `TunnelMonitor`，建议：

- 发布端在 `tunnel_region_monitor` 里显式配置 `tunnel_monitor_topic: tunnel_monitor`
- 订阅端在 `sentry_bt` 里也显式配置 `tunnel_monitor_topic: tunnel_monitor`

这样 launch 和调试时更直观，不用依赖默认值对齐。

### 11.2 区域相关

- `trigger_x_mins/maxs`
- `trigger_y_mins/maxs`
- `tunnel_x_mins/maxs`
- `tunnel_y_mins/maxs`
- `entry_xs/ys`
- `exit_xs/ys`

一个下标表示同一个洞。

如果数组长度不一致，当前实现会在启动阶段直接报 `fatal` 并退出，防止配置错误被静默带入运行期。

### 11.2.1 低顶极限洞配置

- `low_clearance_tunnel_ids`
  需要启用“低顶极限模式”的洞编号列表。只有当前洞编号在该列表内，且状态处于 `APPROACHING / IN_TUNNEL / RECOVERY_ACTIVE` 时，`low_clearance_mode` 才会发布为 `true`。

示例（四个洞都启用）：

```yaml
tunnel_region_monitor:
  ros__parameters:
    low_clearance_tunnel_ids: [0, 1, 2, 3]
```

### 11.2.2 低顶模式自动切换 `vehicleHeight`

如果你希望在 `low_clearance_mode=true` 时自动把 `terrain_analysis` 的 `vehicleHeight` 调低，可以开启：

- `enable_low_clearance_param_switch`
  开关，默认 `false`
- `low_clearance_vehicle_height`
  低顶模式使用的 `vehicleHeight`
- `normal_vehicle_height`
  普通模式使用的 `vehicleHeight`
- `low_clearance_target_nodes`
  需要被动态改参数的目标节点名列表（默认 `terrain_analysis` 和 `terrain_analysis_ext`）

示例：

```yaml
tunnel_region_monitor:
  ros__parameters:
    low_clearance_tunnel_ids: [0, 1, 2, 3]
    enable_low_clearance_param_switch: true
    normal_vehicle_height: 0.23
    low_clearance_vehicle_height: 0.20
    low_clearance_target_nodes: ["terrain_analysis", "terrain_analysis_ext"]
```

### 11.3 判定阈值

- `activation_margin`
  已在洞内时的滞回边界
- `path_check_margin`
  扫描路径是否命中 trigger 的额外放宽量
- `disable_spin_margin`
  判断是否进入准备区的放宽量
- `min_path_points_in_region`
  连续多少个路径点落入 trigger 才算命中该洞

更具体地说：

- `activation_margin`
  用于判断机器人是否已经进入目标洞的 `trigger` 准备区。
  实现上是把 `trigger` 区域按这个值向四周放大后再判断位置。
  值越大，机器人越早进入“接近洞口”的状态。

- `path_check_margin`
  用于扫全局路径时判断路径点是否“命中某个洞的 trigger 区”。
  实现上也是在 `trigger` 边界基础上加一圈裕量。
  值越大，越容易把一条擦边经过洞口的路径识别成“会过洞”。

- `disable_spin_margin`
  用于发布 `disable_spin=true` 的范围判断。
  一般会比真正的洞口区域稍大一点，让机器人在接近洞口前就提前禁用原地旋转。

- `min_path_points_in_region`
  全局路径中至少有多少个路径点落进某个洞的 `trigger` 区，才认定这次导航会经过该洞。
  值太小容易误判，值太大又可能让短路径或稀疏路径漏判。

- `publish_rate`
  本节点内部状态刷新和输出发布频率，单位 Hz。
  例如 `2.0` 表示每秒更新 2 次。

- `transform_tolerance`
  查询 TF 时允许等待的超时时间，单位秒。
  如果 TF 更新偶尔有延迟，可以适当调大；但过大也会让状态刷新更迟钝。

### 11.4 卡住判定

- `approach_stuck_timeout`
- `tunnel_stuck_timeout`
- `min_progress_distance`

更具体地说：

- `approach_stuck_timeout`
  机器人已经进入 `trigger` 准备区、但还没真正进入 `tunnel` 时的卡住超时。
  在这段时间内，如果机器人一直没有达到“足够位移”，就会触发 recovery。

- `tunnel_stuck_timeout`
  机器人已经在洞体内部时的卡住超时。
  通常会比 `approach_stuck_timeout` 更严格，因为进洞后更希望尽快判断是否堵住。

- `min_progress_distance`
  多大的真实位移才算“有进展”。
  只有机器人相对参考点移动超过这个值，才会刷新 `last_progress_time`。
  这样可以避免把原地抖动、轻微打滑误判成正常推进。

### 11.5 recovery

- `recovery_retreat_distance`
  进入侧外回退距离
- `recovery_clear_margin`
  判断是否已经退出 trigger 区的边界裕量

更具体地说：

- `recovery_retreat_distance`
  触发 recovery 后，沿“本次进入侧的反方向”回退多远，单位米。
  当前逻辑不是强行继续穿洞，而是先退回进入侧外面，再重新尝试。

- `recovery_clear_margin`
  判断 recovery 是否完成的距离阈值。
  机器人当前位置距离 `tunnel_recovery_goal` 小于该值时，就认为已经退回到位。

### 11.5.1 低顶模式参数切换

- `enable_low_clearance_param_switch`
  是否在低顶极限洞模式下，自动修改目标节点的 `vehicleHeight` 参数。

- `normal_vehicle_height`
  普通模式下写给 `terrain_analysis / terrain_analysis_ext` 的 `vehicleHeight`。

- `low_clearance_vehicle_height`
  低顶模式下写给目标节点的 `vehicleHeight`。
  一般会比普通模式更低，用于让地形/障碍分析按更保守的车高进行处理。

如果 `low_clearance_tunnel_ids` 包含当前洞编号，并且节点状态处于：

- `APPROACHING`
- `IN_TUNNEL`
- `RECOVERY_ACTIVE`

则 `low_clearance_mode=true`，并在开启 `enable_low_clearance_param_switch` 时把目标节点切到 `low_clearance_vehicle_height`。
离开这些状态后，再恢复为 `normal_vehicle_height`。

## 12. RViz 可视化怎么看

`tunnel_markers` 里当前会画出：

- trigger 区域矩形
- tunnel 区域矩形
- 每个洞的 entry 点
- 每个洞的 exit 点
- 当前目标过洞朝向箭头
- recovery 目标点

其中当前目标洞会用更亮的 trigger / tunnel 区域高亮。

## 13. 当前限制

目前这套逻辑有几个需要知道的点：

- recovery 只接到了 `navigate_to_pose` 这棵 BT
- `navigate_through_poses` 还没有接同样的目标切换逻辑
- 路径选洞当前按洞编号顺序扫描，不是按路径上的真实先后顺序做几何排序
- 卡住判定目前基于平面位移，不是基于洞方向投影位移
- 如果节点重启时机器人已经在洞里，之前记录的“进入侧”会丢失；此时只能退化为按当前位置相对 `entry/exit` 的距离推断侧别

## 14. 建议的使用方式

如果你当前策略是“底盘过洞、云台保持过洞方向”，建议：

- `tunnel_region_monitor` 继续启用
- 电控订阅 `tunnel_target_yaw` 作为过洞方向参考
- recovery 时让底盘按 Nav2 的 recovery goal 后退
- 云台继续保持过洞方向，不跟 recovery 方向回头转

## 15. 一句话总结

`tunnel_region_monitor` 不是控制器，而是：

- 看路径会不会过洞
- 判断当前处于哪个过洞阶段
- 计算过洞方向
- 卡住时触发“退回入口外”的恢复
- 把这些状态统一发布给 Nav2、RViz 和电控使用

## 16. TunnelMonitor 消息字段

`TunnelMonitor.msg` 目前放在 `sentry_decision_msg` 包里，字段如下：

- `enable_tunnel_mode`
- `has_pose`
- `has_plan`
- `will_pass_tunnel`
- `in_tunnel`
- `disable_spin`
- `tunnel_recovery_active`
- `target_tunnel_id`
- `tracked_tunnel_id`
- `recovery_tunnel_id`
- `tunnel_status`
- `robot_x`
- `robot_y`
- `current_map_yaw`
- `tunnel_target_yaw`
- `tunnel_yaw_error`
- `controller_id`
- `global_pose`
- `tunnel_recovery_goal`

推荐用法：

- 如果你只是做调试或 RViz 联调，继续看原有分散 topic 也可以
- 如果你是电控、决策或串口桥接节点，优先订阅 `tunnel_monitor`，这样不用同时订阅十几个 topic 再自己拼状态
- `sentry_bt` 当前已经支持订阅 `tunnel_monitor`，并会把关键字段同步进黑板

## 17. tunnel_status 状态说明

`tunnel_status` 是 `int32`，对应代码里的过洞状态机枚举：

```text
0 NORMAL
1 WILL_PASS
2 APPROACHING
3 IN_TUNNEL
4 PASSED
5 RECOVERY_ACTIVE
```

各状态含义如下：

- `NORMAL = 0`
  当前没有检测到需要过洞，机器人也不在洞内。

- `WILL_PASS = 1`
  当前全局路径会经过某个洞，但机器人还没进入该洞的 trigger 准备区。

- `APPROACHING = 2`
  当前路径会过洞，并且机器人已经进入 trigger 准备区，但还没进入 tunnel 洞体区。

- `IN_TUNNEL = 3`
  机器人当前位置已经进入 tunnel 洞体区。

- `PASSED = 4`
  当前不再需要过洞，并且机器人不在洞内。这个状态表示过洞链路已经离开激活阶段。

- `RECOVERY_ACTIVE = 5`
  过洞准备阶段或洞内阶段长时间没有进展，已触发 recovery，当前会发布 `tunnel_recovery_goal`。

当前下位机“收云台/缩头”使用的是 `APPROACHING / IN_TUNNEL / RECOVERY_ACTIVE` 阶段：

```text
need_tunnel = tunnel_status == APPROACHING || tunnel_status == IN_TUNNEL || tunnel_status == RECOVERY_ACTIVE
```

也就是说：

- 只是路径未来会过洞，但还没靠近时，不收云台
- 靠近洞口准备过洞时，收云台
- 进洞过程中，继续保持收云台
- recovery 期间，继续保持收云台
- 已经出洞或不需要过洞时，打开云台
