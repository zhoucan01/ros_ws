# 2026-06-22 姿态与热量链路复审记录

## 本轮复审范围

- `G:\slam\ros_ws\src\sentry_bt\config\sentry_bt.xml`
- `G:\slam\ros_ws\src\sentry_bt\config\sentry_bt_params.yaml`
- `G:\slam\ros_ws\src\sentry_bt\include\sentry_bt\topics2blackboard.hpp`
- `G:\slam\ros_ws\src\sentry_bt\src\behaviors\topics2blackboard.cpp`
- `G:\slam\ros_ws\src\sentry_bt\src\sentry_bt_node.cpp`
- `G:\slam\ros_ws\src\sentry_decision_msg\msg\RefereeRaw.msg`
- `G:\slam\ros_ws\src\sentry_decision_msg\msg\HostDecision.msg`
- `G:\slam\ros_ws\src\algo_master\include\algo_master\serialport.hpp`
- `G:\slam\ros_ws\src\algo_master\include\algo_master\constants.hpp`

## 本轮确认的改动

### 1. 行为树追击入口修正

- 文件：`G:\slam\ros_ws\src\sentry_bt\config\sentry_bt.xml`
- 改动：`AntiAutoAim` 入口由 `if_need_hp_recover == true` 改为 `if_need_hp_recover == false`
- 目的：避免出现“越该回血越去追击”

### 2. 姿态选择升级为轻量滚动预测

- 文件：
  - `G:\slam\ros_ws\src\sentry_bt\include\sentry_bt\topics2blackboard.hpp`
  - `G:\slam\ros_ws\src\sentry_bt\src\behaviors\topics2blackboard.cpp`
  - `G:\slam\ros_ws\src\sentry_bt\config\sentry_bt_params.yaml`
- 改动：
  - 新增 `PredictiveState`
  - 新增姿态冷却、阶段打分、滚动打分、未来热量传播
  - 新增相关 YAML 参数
- 目的：让姿态切换从“只看当前帧”升级为“向前看几步再决策”

### 3. 热量三件套全链路打通

- 文件：
  - `G:\slam\ros_ws\src\sentry_decision_msg\msg\RefereeRaw.msg`
  - `G:\slam\ros_ws\src\sentry_decision_msg\msg\HostDecision.msg`
  - `G:\slam\ros_ws\src\algo_master\include\algo_master\serialport.hpp`
  - `G:\slam\ros_ws\src\algo_master\include\algo_master\constants.hpp`
  - `G:\slam\ros_ws\src\sentry_bt\src\behaviors\topics2blackboard.cpp`
  - `G:\slam\ros_ws\src\sentry_bt\src\sentry_bt_node.cpp`
- 新增字段：
  - `current_shoot_heat_17mm`
  - `heat_limit_17mm`
  - `heat_cool_rate_17mm`
- 目的：为姿态切换提供热量前瞻输入，并把状态回传到 `HostDecision`

### 4. 本轮顺手修正的一个小 bug

- 文件：`G:\slam\ros_ws\src\sentry_bt\src\behaviors\topics2blackboard.cpp`
- 改动：第一次看到真实姿态回传时，不再误判成“刚发生了一次姿态切换”
- 目的：避免系统刚启动或比赛刚开始时，被错误施加 5 秒切姿态冷却

## 复审结论

### 已确认没断的主链

- 下位机 `DecisionSerialMsg::RefereeRawData`
- `algo_master` 转 `RefereeRaw.msg`
- `BlackboardUpdater` 收到后写黑板
- `HostDecision.msg` 回传热量相关字段
- `desired_sentry_attitude` 仍由 `BlackboardUpdater` 选择，`SentryAttitudeSwitch` 只负责执行

### 本轮发现的剩余风险

#### P1. 串口协议尺寸已经变化，但代码里没有显式防呆

- 位置：
  - `G:\slam\ros_ws\src\algo_master\include\algo_master\serialport.hpp`
  - `G:\slam\ros_ws\src\algo_master\include\algo_master\constants.hpp`
- 说明：
  - `DecisionSerialMsg::RefereeRawData` 新增了 3 个 `uint16`
  - 这会直接改变 `sizeof(DecisionSerialMsg)`，而 `ProcRawBuf()` 解包长度绑定的是 `kDecisionRecvMsgSize`
  - 如果下位机没有同步升级同一版结构体，当前上位机会按错误长度切帧和解包
- 结论：
  - 这不是上位机单边能彻底解决的问题
  - 下位机必须同步更新结构体和发包长度

#### P2. 当前“前瞻”里，`need_attack` 仍被当作未来几秒恒定不变

- 位置：`G:\slam\ros_ws\src\sentry_bt\src\behaviors\topics2blackboard.cpp`
- 说明：
  - `simulate_attitude_step(...)` 里未来热量传播仍直接使用当前帧的 `need_attack`
  - 这意味着它能前瞻“热量积累”，但还不能前瞻“未来几秒会不会持续瞄住目标”
- 影响：
  - 方向基本对
  - 但切换时机仍可能偏早或偏晚
- 建议：
  - 后续最好补 `estimated_fire_rate_17mm` 或最近发射计数

#### P2. `current_hp == 0` 时直接返回防御姿态，语义比较硬编码

- 位置：`G:\slam\ros_ws\src\sentry_bt\src\behaviors\topics2blackboard.cpp`
- 说明：
  - `select_desired_sentry_attitude()` 当前在 `current_hp == 0` 时固定返回 `2`
  - 如果后续“死亡/失活”状态在下位机或行为树里有专门语义，这里可能会和真实期望不一致

## 当前源码级建议

1. 下位机同步更新 `DecisionSerialMsg`，并确认两端 `sizeof(DecisionSerialMsg)` 一致
2. 若后续真要做更强前瞻，优先增加 `estimated_fire_rate_17mm` 或最近发射计数
3. 若比赛里存在“死亡后姿态无意义/有专门失活状态”，再单独收口 `current_hp == 0` 的处理逻辑

## 本轮未完成的验证

- 当前电脑没有 ROS2 / `colcon` 环境
- 本轮只能做源码级一致性检查，未完成实际编译和运行验证

## 计划提交范围建议

若只推送“这轮姿态与热量链路”相关内容，建议只包含：

- `G:\slam\ros_ws\src\sentry_bt\config\sentry_bt.xml`
- `G:\slam\ros_ws\src\sentry_bt\config\sentry_bt_params.yaml`
- `G:\slam\ros_ws\src\sentry_bt\include\sentry_bt\topics2blackboard.hpp`
- `G:\slam\ros_ws\src\sentry_bt\src\behaviors\topics2blackboard.cpp`
- `G:\slam\ros_ws\src\sentry_bt\src\sentry_bt_node.cpp`
- `G:\slam\ros_ws\src\sentry_decision_msg\msg\RefereeRaw.msg`
- `G:\slam\ros_ws\src\sentry_decision_msg\msg\HostDecision.msg`
- `G:\slam\ros_ws\src\algo_master\include\algo_master\serialport.hpp`
- `G:\slam\ros_ws\src\algo_master\include\algo_master\constants.hpp`
- `G:\slam\ros_ws\src\pb2025_sentry_nav\mod_logs\README.md`
- `G:\slam\ros_ws\src\pb2025_sentry_nav\mod_logs\2026-06-22_sentry_attitude_rollout_and_heat_plan.md`
- `G:\slam\ros_ws\src\pb2025_sentry_nav\mod_logs\2026-06-22_sentry_attitude_review_and_changes.md`
