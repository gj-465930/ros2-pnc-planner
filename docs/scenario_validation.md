# Scenario Validation

本文档记录 Mini PNC Lab 第一版 YAML 场景验证结果。当前验证方式仍以人工、ROS2
接口证据和 RViz 观察为主，尚未实现自动 metrics runner 或批量 pass/fail 报告。

当前场景执行链路为：

```text
scenario YAML
      ↓
ScenarioLoader
      ↓
scenario_publisher
      ├── /routing_path
      └── /scenario/initial_state
                    ↓
             PncPlannerNode
                    ↓
       ReferenceLine / LatticePlanner / EgoVehicle
```

`expected` 中的 `collision_free`、`max_abs_l`、`max_acc` 等字段目前主要作为人工验收目标
和后续自动评测接口预留，文档不会把尚未采集的指标描述为自动通过。

## 验证范围

第一批基础场景均为无障碍路线场景：

```text
straight_cruise.yaml
curve_cruise.yaml
end_of_route.yaml
```

这些场景验证：

```text
ScenarioLoader 能解析 route 和 ego 初始状态
ScenarioPublisher 能发布 /routing_path 和 /scenario/initial_state
PncPlannerNode 能应用场景自车状态并更新 ReferenceLine
路线和初始状态均就绪后才开始规划
LatticePlanner 能否在对应边界条件下生成轨迹
RViz 和 TF 能否反映车辆实际运行状态
```

## 测试环境

本次验收环境：

```text
日期：2026-07-20
平台：macOS arm64
ROS2：Humble（Robostack / Pixi）
编译：C++17，ament_cmake，colcon
场景：src/pnc_planner/scenarios/end_of_route.yaml
```

## 构建与测试

从工作空间根目录构建并加载环境：

```bash
colcon build --packages-select pnc_planner
source install/setup.zsh
```

仅运行功能 gtest：

```bash
colcon test-result --delete-yes
colcon test \
  --packages-select pnc_planner \
  --event-handlers console_direct+ \
  --ctest-args -R '^test_' -V
colcon test-result --verbose
```

本次验收时，5 个 gtest target、18 个测试断言全部通过：

```text
QuinticPolynomial：2/2
CartesianFrenet：3/3
ReferenceLine：5/5
LatticePlanner：1/1
ScenarioLoader：7/7
```

完整 `colcon test` 仍会运行 flake8、CMake lint、uncrustify 和 xmllint。当前仓库存在
全局格式规则不一致和离线 XML schema 问题，因此完整 lint 报告尚未全绿；这些问题不等同于
算法或场景功能测试失败，后续应作为独立的格式与测试基础设施任务处理。

## 运行方式

终端 1：启动 planner、robot_state_publisher 和 RViz：

```bash
ros2 launch pnc_planner pnc_planner.launch.py
```

终端 2：发布场景：

```bash
ros2 run pnc_planner scenario_publisher --ros-args \
  -p scenario_file:=src/pnc_planner/scenarios/end_of_route.yaml
```

接口验证：

```bash
ros2 topic echo /scenario/initial_state --once
```

TF 验证需要在场景初始状态发布前启动监听，因为当前动态 TF 不是 transient-local：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

## 当前结果

| Scenario | Purpose | Expected Behavior | Status |
|---|---|---|---|
| `straight_cruise` | 直线巡航 | 参考线沿 x 轴生成，规划轨迹稳定跟随直线 | Pass |
| `curve_cruise` | 缓弯巡航 | 参考线随 YAML 路线形成缓弯，规划轨迹能够沿曲线路线生成 | Pass |
| `end_of_route` | 接近终点路线 | 从 x=16 m、v=3 m/s 启动并在 x=20 m 附近安全停车 | Partial |

## 基础场景既有记录

### `straight_cruise`

场景文件：

```text
src/pnc_planner/scenarios/straight_cruise.yaml
```

既有人工验收结果：

```text
ScenarioPublisher 成功发布直线路线
PncPlannerNode 成功初始化参考线
RViz 中可以观察到沿 x 轴的参考线和规划轨迹
```

### `curve_cruise`

场景文件：

```text
src/pnc_planner/scenarios/curve_cruise.yaml
```

既有人工验收结果：

```text
ScenarioPublisher 成功发布缓弯路线
PncPlannerNode 能根据外部路线更新 ReferenceLine
RViz 中可以观察到缓弯参考线和规划轨迹
```

## `end_of_route` 真实初始状态验收

场景文件：

```text
src/pnc_planner/scenarios/end_of_route.yaml
```

场景关键输入：

```text
路线：x = 0 m → 10 m → 20 m
ego.x = 16.0 m
ego.y = 0.0 m
ego.yaw = 0.0 rad
ego.v = 3.0 m/s
ego.a = 0.0 m/s²
ego.state = CRUISING
timeout_sec = 8.0 s
```

### Topic 证据

`ros2 topic echo /scenario/initial_state --once` 输出：

```yaml
header:
  frame_id: map
x: 16.0
y: 0.0
yaw: 0.0
velocity: 3.0
acceleration: 0.0
state: CRUISING
```

### 节点日志证据

`PncPlannerNode` 明确记录：

```text
初始化参考线成功，总长度为 20.00
Applied initial state: x=16.00, y=0.00, yaw=0.00, v=3.00, a=0.00, state=CRUISING
```

这证明运行时使用的是 YAML 中的真实初始状态，而不是旧的 `x=0、v=5` 写死值。

### TF 与运行结果

TF 记录显示车辆沿 x 轴继续运动：

```text
t=1784539676：x=18.7 m
t=1784539677：x=21.7 m，已经越过 x=20 m 路线终点
t=1784539684：x=42.7 m，约 8 秒时仍以接近 3 m/s 继续前进
```

整个过程中 y 和 yaw 约为 0，车辆没有明显横向跳变。系统未崩溃，也未出现容器越界，
但车辆没有在路线终点附近停车，因此 `reach_goal` 不能判为通过。

后续出现相同时间戳和 `x=43.6 m` 的重复 TF 输出，只表示 `tf2_echo` 重复最后一帧，
不能作为车辆已经主动停车的证据。

### 根因分析

接近终点后，巡航纵向候选的目标位置超过参考线长度，`LatticePlanner` 输出：

```text
[LatticePlanner] Error: 纵向轨迹生成失败
```

规划失败时，`PncPlannerNode` 中之前成功生成的 `planned_traj_` 仍然非空。节点随后继续调用
`trackTrajectory()`，导致车辆继续跟踪陈旧轨迹并越过终点。

该现象包含两个独立问题：

```text
1. 尚未实现接近路线终点时的停车纵向轨迹。
2. 规划失败后仍会继续跟踪上一周期的陈旧轨迹，缺少安全降级策略。
```

### 验收结论

| Check | Result | Evidence |
|---|---|---|
| 场景初始状态接口 | Pass | topic 值与 YAML 一致 |
| PncPlannerNode 应用初始状态 | Pass | 日志记录 x=16.0、v=3.0 |
| 参考线初始化 | Pass | 总长度 20.0 m |
| 运行稳定性 | Pass | 超过 8 秒未崩溃或越界 |
| collision_free | Not automatically evaluated | 当前场景无障碍物 |
| max_abs_l | Not automatically evaluated | TF 中 y≈0，仅有人工证据 |
| max_acc / max_decel | Not automatically evaluated | 尚无 metrics 采集 |
| reach_goal | Fail | 越过终点并继续前进 |
| 终点停车 | Fail | 未生成合理停车轨迹 |

总体结论：

```text
Partial：场景初始状态链路和系统稳定性通过，终点停车行为未通过。
```

## 当前局限与后续任务

当前限制包括：

```text
expected 指标尚未自动采集或自动判定
尚无批量 scenario runner
state 字段尚未驱动独立行为状态机
动态 TF 当前只在车辆状态更新时广播，晚启动的订阅者可能错过初始 TF
规划失败时缺少陈旧轨迹失效和安全降级策略
尚未实现终点停车纵向规划
```

建议后续工作：

```text
规划失败安全降级：禁止继续跟踪陈旧轨迹，并设计安全降级策略
终点停车规划：实现接近路线终点时的停车纵向轨迹
```

在这些任务完成前，不应将 `end_of_route` 描述为完整通过，也不应修改场景期望值来掩盖
当前规划行为。
