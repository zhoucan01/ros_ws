# sentry_bt 说明

## 1. 这个包是做什么的

`sentry_bt` 是哨兵决策侧的行为树节点包，主要负责：

- 订阅上游决策、裁判、敌方目标、导航与过洞相关话题
- 把这些输入同步到 BehaviorTree 的黑板
- 在上位机计算补弹、血量、姿态等派生决策状态
- 运行行为树 XML，根据黑板状态输出导航目标或姿态切换命令

## 1.1 姿态逻辑现在怎么分层

当前姿态链路已经尽量拆开：

- `RefereeRaw.msg.real_sentry_attitude_switch`
  - 下位机/裁判系统当前真实姿态
- `topics2blackboard.cpp`
  - 负责真实姿态同步、累计时长、3 分钟削弱判断、姿态打分、产出 `desired_sentry_attitude`
- `sentry_bt.xml`
  - 不再自己做姿态条件判断，只负责执行 `desired_sentry_attitude`
- `SentryAttitudeSwitch`
  - 负责 5 秒切换冷却和实际姿态切换消息发布

也就是说，现在已经把“姿态选择”和“姿态执行”拆开了。

当前可执行节点是：

- `sentry_bt_node`

主要配置文件是：

- [config/sentry_bt.xml](/home/zc/ros_ws/src/sentry_bt/config/sentry_bt.xml)
- [config/sentry_bt_params.yaml](/home/zc/ros_ws/src/sentry_bt/config/sentry_bt_params.yaml)

## 1.2 点位坐标语义

当前 `sentry_bt` 里的固定点位参数 `points.*`，统一按“右下角全局坐标系下的真实点位”填写。

当前采用的处理方式是：

- `points.*`
  - YAML 里填写全局真实坐标
- `ManualPos`
  - 下位机发来的云台手坐标也按同一套全局坐标系处理
- `PublishNavGoal`
  - 在真正发布 `plc2target` 前，统一减去 `INIT_PACK_POINT` 的全局坐标
  - 把全局坐标转换成建图起点坐标系

也就是说，现在固定点和手操点已经统一到同一套全局坐标语义里，不再出现“有的减、有的不减”的情况。

当前 `INIT_PACK_POINT` 的全局坐标，也就是转换基准，是：

- `points.0`

发布日志里现在会同时打印：

- 全局目标坐标
- 相对 `INIT_PACK_POINT` 的发布坐标

方便你联调时核对两套坐标系。

## 2. 与过洞逻辑的关系

`sentry_bt` 现在已经接入 `tunnel_region_monitor` 发布的聚合消息：

- 消息类型：`sentry_decision_msg/msg/TunnelMonitor`
- 默认订阅话题：`tunnel_monitor`

接线位置在：

- [topics2blackboard.hpp](/home/zc/ros_ws/src/sentry_bt/include/sentry_bt/topics2blackboard.hpp)
- [topics2blackboard.cpp](/home/zc/ros_ws/src/sentry_bt/src/behaviors/topics2blackboard.cpp)

也就是说，`sentry_bt` 不需要再自己同时订阅一堆分散的过洞 topic，而是可以直接从一个聚合消息里拿状态。

## 3. 需要改的 YAML

这一轮和过洞相关，YAML 里需要关注的其实就 3 处：

### 3.1 `tunnel_region_monitor` 发布端

在 Nav2 参数里，建议显式写上：

```yaml
tunnel_region_monitor:
  ros__parameters:
    tunnel_monitor_topic: tunnel_monitor
```

当前我已经补到这两个文件里了：

- [reality/nav2_params.yaml](/home/zc/ros_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml)
- [simulation/nav2_params.yaml](/home/zc/ros_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/nav2_params.yaml)

### 3.2 `sentry_bt` 订阅端

在 `sentry_bt` 自己的参数里，建议保持一致：

```yaml
sentry_bt_node:
  ros__parameters:
    tunnel_monitor_topic: tunnel_monitor
```

当前我已经补到：

- [sentry_bt_params.yaml](/home/zc/ros_ws/src/sentry_bt/config/sentry_bt_params.yaml)

### 3.3 其余过洞分散 topic

目前不需要为了 `sentry_bt` 再额外配一堆：

- `in_tunnel`
- `will_pass_tunnel`
- `disable_spin`
- `tunnel_status`
- `tunnel_recovery_active`
- `tunnel_target_yaw`
- `tunnel_recovery_goal`

因为这些字段都已经被 `TunnelMonitor` 聚合打包了。

## 4. 黑板里现在有哪些过洞键

`BlackboardUpdater` 收到 `TunnelMonitor` 后，会把这些键同步进黑板：

- `tunnel_monitor`
- `enable_tunnel_mode`
- `tunnel_has_pose`
- `tunnel_has_plan`
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
- `tunnel_controller_id`
- `tunnel_global_pose`
- `tunnel_recovery_goal`

这样你后面如果想在 BT XML 里直接做条件判断，比如：

- 进洞后禁止某些动作
- recovery 时切换策略
- 根据 `disable_spin` 屏蔽会导致原地转向的行为

就不需要再去额外加订阅逻辑了。

## 5. 当前状态

现在的接线方式是：

- `tunnel_region_monitor` 发布 `TunnelMonitor`
- `sentry_bt` 订阅 `TunnelMonitor`
- `BlackboardUpdater` 把它同步到黑板

也就是说，数据链已经通了。

目前还没自动改动 `sentry_bt.xml` 去使用这些过洞黑板键，所以：

- 数据已经进黑板
- 但行为树是否真的利用这些数据，还要看你后面要不要把 XML 规则补上

## 5.1 当前 HostDecision / 黑板决策状态

`BlackboardUpdater` 现在除了同步原始消息外，还会在上位机产出一批派生状态：

- 比赛状态
  - `if_match_started`
- 补弹相关
  - `if_get_allow_17`
  - `if_allowance_less_50`
  - `if_allowance_less_100`
  - `allow_to_get_17mm`
  - `already_allowance_17`
  - `available_allowance_17`
  - `allowance_remain_time`
- 血量/受击相关
  - `if_hp_less_50`
  - `if_hp_less_100`
  - `if_need_hp_recover`
  - `if_recently_hurt`
  - `if_3s_not_hurted`
  - `if_5s_not_hurted`
- 目标点/决策相关
  - `if_enemy_outpost_alive`
  - `if_radar_outpost_target`
  - `if_force_enemy_outpost`
  - `if_can_rebuild_outpost`
  - `if_base_full_hp`
  - `if_base_low_hp`
  - `if_manual_target_valid`
  - `if_target_far`
- 姿态相关
  - `real_sentry_attitude_switch`
  - `desired_sentry_attitude`
  - `attack_attitude_time`
  - `defense_attitude_time`
  - `move_attitude_time`
  - `attack_attitude_weakened`
  - `defense_attitude_weakened`
  - `move_attitude_weakened`
- `attack_attitude_score`
- `defense_attitude_score`
- `move_attitude_score`

## 5.1.1 当前目标点决策逻辑

当前目标点决策是在行为树 XML 中按优先级从上到下执行的，整体是一个 `if / else if` 风格的 `Fallback` 链。

当前顺序如下：

1. 如果比赛没有开始
   - 条件：`if_match_started == false`
   - 目标点：`INIT_PACK_POINT`

2. 如果自己血量小于 100，或者需要回家补弹
   - 条件：`if_hp_less_100 == true || if_get_allow_17 == true || if_force_stay_home == true`
   - 目标点：`WE_DEPOT_POINT`

这里当前还加了一条“回家锁定”逻辑：

- 一旦因为低血或补弹需求进入回家流程，`if_force_stay_home` 会被置为 `true`
- 只要这个标志还在，就会持续把目标保持为 `WE_DEPOT_POINT`
- 如果是因为补弹需求回家，则必须“到家 + 血量恢复 + 补弹需求解除”后，才允许重新出家
- 如果只是因为回血回家，则只要“到家 + 回满血”就允许重新出家

### 5.1.1.1 当前补弹时机判断

补弹逻辑里现在不再把最后一条写成固定的 `remain_time < 5`，而是使用一个动态时间阈值。

当前思路不是“怕赶不上补弹，所以早点回家”，而是：

- 当当前弹量已经非常低时
- 结合当前位置到回家点的距离
- 估算以当前速度回家大概需要多久
- 让机器人尽量在一个“不早也不晚”的时机回家
- 目标是刚好赶上下一波补弹时间窗口

也就是说，这条动态阈值更像是“卡补弹点回家”的时机判断，而不是简单的保守提前回家。

当前相关参数在：

- `allowance.return_speed`
- `allowance.return_buffer_s`

3. 如果云台手数据有效
   - 条件：`if_get_manual_msg == true && if_manual_target_valid == true`
   - 目标：直接使用 `manual_target_pose`

4. 如果接收到雷达站数据，且敌方前哨站还有血量
   - 条件：`if_radar_outpost_target == true`
   - 目标点：`ENEMY_OUTPOST_POINT`

5. 如果比赛处于开局前一分半强制压前哨站时间窗口
   - 条件：`if_force_enemy_outpost == true`
   - 目标点：`ENEMY_OUTPOST_POINT`

这里当前的策略含义是：

- 因为如果没有雷达站数据，就无法确认敌方前哨站当前血量
- 所以在比赛开局前 90 秒，会默认优先压到敌方前哨站附近
- 目的是开局先守住/逼近前哨站相关区域，而不是等到后期再压前哨站

6. 如果当前可以重建前哨站，且己方前哨站血量为 0
   - 条件：`if_can_rebuild_outpost == true && we_outpost_hp <= 0`
   - 目标点：`WE_OUTPOST_POINT`

7. 如果己方基地当前不处于低血状态
   - 条件：`if_base_low_hp == false`
   - 目标点：`ENEMY_FORTRESS_POINT`

8. 如果比赛剩余时间小于等于两分钟
   - 条件：`game_remain_time <= 120`
   - 目标点：`WE_PROTECT_POINT`

这里当前把 `WE_PROTECT_POINT` 作为巡逻/回防点使用。

9. 如果己方基地血量低于 2000
   - 条件：`if_base_low_hp == true`
   - 目标点：`WE_PROTECT_POINT`

10. 否则
   - 当前默认回落到：`INIT_PACK_POINT`

也就是说，当前树的目标点逻辑不是做打分，而是严格优先级决策。

## 5.1.2 追击目标覆盖逻辑

在目标点决策完成后，还会进入“追击覆盖”阶段：

- 如果 `if_need_to_attack == 1`
- 并且 `if_need_hp_recover == true`

则会调用 `AntiAutoAim`，尝试生成一个攻击目标点 `final_attack_point`。

当前这块已经拆成三层：

1. 正常决策层
   - 行为树优先级链先给出一个 `normal_target_point`

2. 追击评估层
   - `AntiAutoAim` 输入：
     - `normal_target_point`
     - `normal_target_pose`（逻辑上依赖正常决策目标）
     - `enemy_pos_point`
     - `current_pos`
     - `costmap`
   - 输出：
     - `attack_target_pose`
     - `attack_target_valid`

3. 最终目标仲裁层
   - `SelectFinalTarget` 统一决定最终目标来源
   - 当前优先级是：
     - 手操目标优先
     - 其次攻击目标
     - 最后普通决策点

也就是说，现在最终发给导航的目标，不再由 `PublishNavGoal` 自己隐式判断，而是由 `SelectFinalTarget` 单独仲裁。

当前串口里发送的 `if_on_attack` 也是根据“最终仲裁结果是否真的用了攻击目标”来确定，而不是单纯看是否有追击意图。

### 5.1.2.1 当前“追击是否有输出”依赖链

当前追击输出不是由单一开关决定，而是三层共同决定：

1. 入口是否允许追击

- `if_need_to_attack == 1`
  - 目前来自 `enemy_msg.if_vision_on`
- `if_need_hp_recover == true`
  - 目前来自上位机自己的血量判断

只有入口允许，行为树才会真正调用 `AntiAutoAim`。

2. 当前正常目标点下是否允许追击

`AntiAutoAim` 会根据当前 `normal_target_point` 查 YAML 里的策略配置：

- `anti_autoaim.policy.<point_id>.enable`
- `anti_autoaim.policy.<point_id>.limit`

其中：

- `enable = false`
  - 该普通目标点下直接禁止追击
- `limit = true`
  - 追击候选点还必须落在该目标点对应的限制矩形内

对应矩形参数为：

- `anti_autoaim.rect.<point_id>.x_min`
- `anti_autoaim.rect.<point_id>.x_max`
- `anti_autoaim.rect.<point_id>.y_min`
- `anti_autoaim.rect.<point_id>.y_max`

3. 环境里是否真的能算出可行攻击点

即使入口允许、策略也允许，`AntiAutoAim` 还要继续检查：

- `enemy_pos_point`
- `current_pos`
- `costmap`

处理流程大致是：

- 先围绕敌人生成一圈候选点
- 用 `costmap` 过滤掉代价过高的点
- 如果开了 `limit`，再过滤掉矩形外的点
- 如果最后没有剩余候选点，则追击失败

所以当前追击是否有输出，可以概括为：

`if_need_to_attack && if_need_hp_recover && policy.enable && 环境可行`

最终输出结果是：

- `attack_target_valid`
- `attack_target_pose`

如果 `attack_target_valid == false`，那么最终仲裁层会回退到手操目标或普通决策目标。

### 5.1.2.2 追击相关 YAML 参数

和追击是否能输出直接相关的参数包括：

- `anti_autoaim.enable_attack`
- `anti_autoaim.limit_chase_range`
- `anti_autoaim.max_chase_distance`
- `anti_autoaim.cost_threshold`
- `anti_autoaim.distance_weight`
- `anti_autoaim.cost_weight`

以及每个普通目标点单独的策略参数：

- `anti_autoaim.policy.<point_id>.enable`
- `anti_autoaim.policy.<point_id>.limit`

以及每个普通目标点的追击限制区域：

- `anti_autoaim.rect.<point_id>.x_min/x_max/y_min/y_max`

## 5.2 姿态选择逻辑

当前姿态系统已经拆成了“选择”和“执行”两层：

- 选择层：`topics2blackboard.cpp`
  - 基于真实姿态、比赛剩余时间、当前位置、追击状态、受击状态等信息做姿态打分
  - 输出 `desired_sentry_attitude`
- 执行层：`sentry_bt.xml` + `SentryAttitudeSwitch`
  - 行为树只执行 `desired_sentry_attitude`
  - `SentryAttitudeSwitch` 负责姿态切换消息发布和 5 秒冷却

### 5.2.1 真实姿态和目标姿态的区别

- `real_sentry_attitude_switch`
  - 来源：`RefereeRaw.msg`
  - 含义：下位机/裁判系统当前真实生效的姿态
- `desired_sentry_attitude`
  - 来源：上位机姿态评分器
  - 含义：上位机当前希望切换到的姿态

姿态累计时间、3 分钟削弱判断，都是基于 `real_sentry_attitude_switch` 统计，而不是基于 `desired_sentry_attitude` 猜测。

### 5.2.2 姿态累计时间与削弱

`BlackboardUpdater` 会按 `game_remain_time` 的变化，给真实姿态累计使用时间：

- `attack_attitude_time`
- `defense_attitude_time`
- `move_attitude_time`

当某个姿态累计时间超过阈值 `attitude.weaken_threshold_s`（默认 180 秒）时，对应姿态会进入削弱状态：

- `attack_attitude_weakened`
- `defense_attitude_weakened`
- `move_attitude_weakened`

削弱不会直接禁止使用该姿态，而是通过打分惩罚降低它被选中的概率。

### 5.2.3 当前三类姿态的打分方式

#### 攻击姿态 `attack`

主要倾向于：

- 正在追击
- 已到点
- 目标不远
- 最近没有被打

当前默认加分/扣分来源：

- `attitude.attack.need_attack`
- `attitude.attack.arrived`
- `attitude.attack.recently_hurt_penalty`
- `attitude.attack.target_far_penalty`
- `attitude.attack.target_near_bonus`
- `attitude.attack.weakened_penalty`

#### 防御姿态 `defense`

主要倾向于：

- 最近被打
- 血量较低
- 离目标不算远

当前默认加分/扣分来源：

- `attitude.defense.recently_hurt`
- `attitude.defense.low_hp`
- `attitude.defense.target_far_penalty`
- `attitude.defense.target_near_bonus`
- `attitude.defense.weakened_penalty`

#### 移动姿态 `move`

主要倾向于：

- 目标较远
- 还没到点
- 当前目标是回家点 `WE_DEPOT_POINT`
- 最近没有持续受击

当前默认加分/扣分来源：

- `attitude.move.target_far`
- `attitude.move.arrived_penalty`
- `attitude.move.not_arrived_bonus`
- `attitude.move.recently_hurt_penalty`
- `attitude.move.go_home_bonus`
- `attitude.move.weakened_penalty`

当前代码里，如果行为树当前目标点是 `WE_DEPOT_POINT`，会额外给移动姿态加一笔较高分数，
目的是在“回家补血/补弹”的任务下，尽量优先维持移动姿态，不轻易切到攻击或防御姿态。

### 5.2.4 最终怎么选姿态

当前会分别计算：

- `attack_attitude_score`
- `defense_attitude_score`
- `move_attitude_score`

然后直接选分数最高的姿态，写入：

- `desired_sentry_attitude`

行为树 XML 不再自己做姿态判断，而是只负责把这个结果交给 `SentryAttitudeSwitch` 去执行。

### 5.2.5 可调参数

这些参数都在：

- [sentry_bt_params.yaml](/home/zc/ros_ws/src/sentry_bt/config/sentry_bt_params.yaml)

和姿态选择直接相关的参数有：

- `target_far_threshold`
- `attitude.weaken_threshold_s`

- `attitude.attack.need_attack`
- `attitude.attack.arrived`
- `attitude.attack.recently_hurt_penalty`
- `attitude.attack.target_far_penalty`
- `attitude.attack.target_near_bonus`
- `attitude.attack.weakened_penalty`

- `attitude.defense.recently_hurt`
- `attitude.defense.low_hp`
- `attitude.defense.target_far_penalty`
- `attitude.defense.target_near_bonus`
- `attitude.defense.weakened_penalty`

- `attitude.move.target_far`
- `attitude.move.arrived_penalty`
- `attitude.move.not_arrived_bonus`
- `attitude.move.recently_hurt_penalty`
- `attitude.move.weakened_penalty`

如果联调时发现姿态切换太激进或太保守，优先调整这一组参数，而不是先改 C++ 逻辑。

## 6. 建议的下一步

如果你准备让 BT 真正感知过洞状态，我建议优先加这几类规则：

- `in_tunnel == true` 时切到更保守的导航/姿态策略
- `disable_spin == true` 时避免触发容易原地转向的动作
- `tunnel_recovery_active == true` 时暂停攻击或追击逻辑
- `tunnel_status` 进入 `RECOVERY_ACTIVE` 时切换专门分支

## 7. 一句话总结

`sentry_bt` 现在已经完成了和过洞模块的消息接线。

YAML 里真正需要保证一致的，就是发布端和订阅端都使用同一个：

- `tunnel_monitor_topic: tunnel_monitor`

剩下的工作，主要就是看你要不要把这些黑板键进一步接进行为树 XML。 
