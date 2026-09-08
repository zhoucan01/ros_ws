# 2026-06-22 哨兵姿态滚动预测与热量接线计划

## 1. 本次已完成

代码位置：
- `G:\slam\ros_ws\src\sentry_bt\src\behaviors\topics2blackboard.cpp`
- `G:\slam\ros_ws\src\sentry_bt\include\sentry_bt\topics2blackboard.hpp`
- `G:\slam\ros_ws\src\sentry_bt\config\sentry_bt_params.yaml`

已完成内容：
- 将姿态选择从“单步静态打分”升级为“轻量滚动预测决策”
- 保留 3 个姿态：
  - 1 攻击
  - 2 防御
  - 3 移动
- 保留执行链：
  - `BlackboardUpdater` 负责选姿态
  - `SentryAttitudeSwitch` 负责发布执行
- 已接入的预测因素：
  - `if_need_to_attack`
  - `if_arrived`
  - `if_target_far`
  - `if_recently_hurt`
  - `if_hp_less_100`
  - `if_get_allow_17`
  - `if_force_stay_home`
  - 当前真实姿态
  - 姿态切换冷却剩余
  - 各姿态累计时长

## 2. 当前热量链路状态

2026-06-22 本轮之后，以下字段已经接入消息主链路：

- `current_shoot_heat_17mm`
- `heat_limit_17mm`
- `heat_cool_rate_17mm`

已打通位置：

- `sentry_decision_msg/msg/RefereeRaw.msg`
- `sentry_decision_msg/msg/HostDecision.msg`
- `algo_master/include/algo_master/serialport.hpp`
- `algo_master/include/algo_master/constants.hpp`
- `sentry_bt/src/behaviors/topics2blackboard.cpp`

也就是说：

- 下位机串口包里只要开始真实填这 3 个字段
- `algo_master` 就会转发到 `referee_raw_msg`
- `BlackboardUpdater` 就会收到并写进黑板
- `HostDecision` 也会把它们回传出来，便于联调观察

## 3. 目前热量前瞻还处于“基础版”

当前已经做了基础接线和基础打分：

- 姿态预测器会读取实时热量、热量上限、冷却速率
- 在 stage score 中加入“接近热量上限 / 超过热量上限”的惩罚或散热收益
- 在 rollout 预测里会模拟未来若干秒热量变化

当前热量传播规则已按用户说明改为：

- 只要 `if_need_to_attack == true`，热量每秒增加 15
- 若 `if_need_to_attack == false`，热量每秒增加 0
- 然后再减去 `heat_cool_rate_17mm`

但它现在还只是第一版近似模型，尚未做到“精确按真实射频和真实冷却过程预测”。

## 4. 如果要把热量前瞻做得更准，最少还缺什么

建议优先从下位机 / 裁判系统再补这 1 类实时量：

1. `estimated_fire_rate_17mm` 或 `recent_shot_count`
- 未来 1 秒左右的估计发射强度
- 二选一即可：
  - 方式 A：上传“最近 0.2s / 0.5s 的发射计数”
  - 方式 B：上传“当前预计每秒发射几发”

原因：

- 现在增热规则已经按“瞄到目标则每秒 +15、没瞄到则 +0”接近真实语义
- 但 `if_need_to_attack` 仍是布尔量，不包含“接下来几秒会不会持续瞄住”的强弱信息
- 如果再补上真实射频估计，它就能更像你想要的那种“快打满前提前切，冷却后再切回”

## 5. 如果想把前瞻做得更像策略控制器，最好再补哪些量

可选增强量：

1. `fire_lock_state`
- 当前是否已经因超热锁发射机构

2. `target_track_confidence`
- 当前目标持续可打的可信度
- 让上位机区分“只是看见了”还是“接下来几秒大概率能持续输出”

3. `attitude_heat_limit_scale[3]`
- 三种姿态对有效热量上限的倍率

4. `attitude_cool_rate_scale[3]`
- 三种姿态对冷却速率的倍率

如果 3、4 是固定规则或固定标定，也可以不作为实时量上送，直接写进 YAML。

## 6. 推荐接线方式

推荐方案：
- 下位机直接上传“最终有效热量状态”
- 上位机只做滚动预测和切姿态决策

推荐上传字段：
- `current_shoot_heat_17mm`
- `effective_heat_limit_17mm`
- `effective_heat_cool_rate_17mm`
- `estimated_fire_rate_17mm`
- `fire_lock_state`

原因：
- 少做一层规则换算
- 避免上位机和下位机对姿态倍率理解不一致
- 更容易联调

注：
- 这次实际落地的消息字段名采用的是：
  - `current_shoot_heat_17mm`
  - `heat_limit_17mm`
  - `heat_cool_rate_17mm`

## 7. 下一步建议

建议实施顺序：

1. 下位机开始真实填充这 3 个字段
2. 再补 `estimated_fire_rate_17mm` 或最近发射计数
3. 把 rollout 里的“不同姿态未来增热”从固定常数改成基于实时射频估计
4. 再加“发射机构已锁 / 即将锁”的专门切换策略

## 8. 目标效果

当热量数据接上后，期望实现的行为是：

- 当前处于防御姿态
- 预测未来几秒继续输出会接近或超过热量上限
- 若攻击姿态或移动姿态在接下来几秒能带来更高有效输出 / 更快脱离热量危险区
- 则提前切姿态
- 冷却窗口过后，再根据目标、受击、低血、回家需求切回更合适的姿态

这时的感觉就会更接近你说的那种“有前瞻性”的切换，而不是只看当前一帧。
