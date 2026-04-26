# sentry_bt 说明

## 1. 这个包是做什么的

`sentry_bt` 是哨兵决策侧的行为树节点包，主要负责：

- 订阅上游决策、裁判、敌方目标、导航与过洞相关话题
- 把这些输入同步到 BehaviorTree 的黑板
- 运行行为树 XML，根据黑板状态输出导航目标或姿态切换命令

当前可执行节点是：

- `sentry_bt_node`

主要配置文件是：

- [config/sentry_bt.xml](/home/zc/ros_ws/src/sentry_bt/config/sentry_bt.xml)
- [config/sentry_bt_params.yaml](/home/zc/ros_ws/src/sentry_bt/config/sentry_bt_params.yaml)

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
