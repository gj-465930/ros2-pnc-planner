# 场景验证记录

这里记录第一版 YAML 场景链路的运行和排查结果。目前主要通过 ROS2 topic、节点日志、TF
和 RViz 检查运行状态，还没有接入自动 metrics 和批量测试。

场景执行链路：

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

YAML 中的 `collision_free`、`max_abs_l`、`max_acc` 等 `expected` 字段暂时只作为人工
检查目标。没有实际采集到的指标，不计为通过。

## 验证范围

第一批场景均为无障碍物场景：

```text
straight_cruise.yaml
curve_cruise.yaml
end_of_route.yaml
```

主要检查：

- route 和 ego 初始状态能否从 YAML 正确加载并发布。
- `PncPlannerNode` 是否使用场景中的初始状态。
- route 和 ego 都就绪后，规划循环能否正常启动。
- Lattice Planner 在直线、曲线和参考线终点附近的表现。
- topic、节点日志、TF 和 RViz 是否一致。

## 测试环境

```text
日期：2026-07-20
平台：macOS arm64
ROS2：Humble（Robostack / Pixi）
编译：C++17，ament_cmake，colcon
重点场景：src/pnc_planner/scenarios/end_of_route.yaml
```

## 构建与测试

```bash
colcon build --packages-select pnc_planner
source install/setup.zsh
```

功能测试：

```bash
colcon test-result --delete-yes
colcon test \
  --packages-select pnc_planner \
  --event-handlers console_direct+ \
  --ctest-args -R '^test_' -V
colcon test-result --verbose
```

最近一次保留的测试记录来自 2026-07-27，共 5 个 gtest target、19 个测试，全部通过：

```text
QuinticPolynomial：2/2
CartesianFrenet：3/3
ReferenceLine：5/5
LatticePlanner：2/2
ScenarioLoader：7/7
```

第二个 `LatticePlanner` 测试覆盖 `plan()` 失败时清空输出轨迹的接口契约。

完整 `colcon test` 还会运行 flake8、CMake lint、uncrustify 和 xmllint。目前仓库的格式
规则没有完全统一，离线环境下 XML schema 检查也有问题，因此功能 gtest 和完整 lint
暂时分开看待。

## 运行步骤

终端 1 启动 planner、robot_state_publisher 和 RViz：

```bash
ros2 launch pnc_planner pnc_planner.launch.py
```

终端 2 发布场景：

```bash
ros2 run pnc_planner scenario_publisher --ros-args \
  -p scenario_file:=src/pnc_planner/scenarios/end_of_route.yaml
```

检查初始状态：

```bash
ros2 topic echo /scenario/initial_state --once
```

监听 TF：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

当前动态 TF 不是 transient-local，最好在场景初始状态发布前启动监听。

## 场景结果

| Scenario | 预期行为 | 当前结果 |
|---|---|---|
| `straight_cruise` | 沿 x 轴生成参考线并稳定跟随 | Pass |
| `curve_cruise` | 根据 YAML 路线生成缓弯参考线和规划轨迹 | Pass |
| `end_of_route` | 从 x=16 m、v=3 m/s 启动，在 x=20 m 附近停车 | Partial |

### `straight_cruise`

`scenario_publisher` 可以正常发布直线路线，`PncPlannerNode` 能完成参考线初始化。RViz 中
可以看到沿 x 轴生成的参考线和规划轨迹。该场景作为最基础的无障碍回归场景。

### `curve_cruise`

节点可以使用 YAML 路线更新 `ReferenceLine`，RViz 中的参考线和规划轨迹都呈缓弯形态。
该场景用于组合检查参考线插值、Frenet 转换和轨迹跟踪。

## `end_of_route` 问题记录

场景输入：

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

### 初始状态确认

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

节点日志：

```text
初始化参考线成功，总长度为 20.00
Applied initial state: x=16.00, y=0.00, yaw=0.00, v=3.00, a=0.00, state=CRUISING
```

这说明运行时使用的是 YAML 中的 `x=16.0、v=3.0`，不是之前写死的 `x=0、v=5`。

### 修复前的现象

当时记录到的 TF：

```text
t=1784539676：x=18.7 m
t=1784539677：x=21.7 m，已经越过 x=20 m 路线终点
t=1784539684：x=42.7 m，约 8 秒时仍以接近 3 m/s 继续前进
```

y 和 yaw 基本保持为 0，没有明显横向跳变，程序也没有崩溃。但车辆没有在参考线终点
附近停车，因此 `reach_goal` 未通过。

`tf2_echo` 后续曾重复输出相同时间戳和 `x=43.6 m`。这是工具重复显示最后一帧，不能
作为车辆主动停车的证据。

### 原因定位

车辆接近终点后，巡航纵向候选的目标位置超过参考线长度，`LatticePlanner` 输出：

```text
[LatticePlanner] Error: 纵向轨迹生成失败
```

这里实际包含两个问题：

1. planner 还不会生成接近参考线终点的正常停车轨迹。
2. 本周期规划失败后，节点仍在跟踪上一周期的 `planned_traj_`。

第二个问题属于轨迹生命周期和失败处理，因此单独修复，没有和终点停车规划混在一起。

### 修复情况

目前已经完成以下修改：

- `LatticePlanner::plan()` 失败时保证输出轨迹为空。
- `PncPlannerNode` 使用本周期的局部候选轨迹接收规划结果。
- 规划失败后清空 `planned_traj_`，不再调用轨迹跟踪控制器。
- 节点使用 `planning_failure_fallback_decel` 执行直线受控减速。
- gtest 已覆盖 planner 失败时清空输出轨迹的契约。

这个修改解决了陈旧轨迹继续被执行的问题，但不等于实现了正常的终点停车。修复后的完整
TF 重跑数据还没有补录，所以 `end_of_route` 仍保持 `Partial`。

### 当前结论

| 检查项 | 结果 | 依据 |
|---|---|---|
| 场景初始状态接口 | Pass | topic 数值与 YAML 一致 |
| 节点应用初始状态 | Pass | 日志记录 x=16.0、v=3.0 |
| 参考线初始化 | Pass | 日志显示总长度 20.0 m |
| 运行稳定性 | Pass | 原始实验运行超过 8 秒，没有崩溃 |
| collision_free | 未自动评估 | 当前场景没有障碍物 |
| max_abs_l | 未自动评估 | TF 中 y≈0，目前只有人工观察 |
| max_acc / max_decel | 未自动评估 | 还没有 metrics 采集 |
| reach_goal | 未通过 | 修复前的实验中车辆越过终点 |
| 正常终点停车 | 未实现 | 当前只有规划失败后的 fallback deceleration |

总体状态：`Partial`。场景输入链路已经打通，陈旧轨迹问题已经修复，但正常终点停车尚未
实现，修复后的完整运行数据也还没有补录。

## 当前限制

- `expected` 指标还不能自动采集和判定。
- 尚未实现 batch scenario runner。
- `state` 字段还没有驱动独立的行为状态机。
- 动态 TF 只在车辆状态更新时广播，晚启动的订阅者可能错过初始 TF。
- planner 还不能生成正常的终点停车纵向轨迹。
- 修复后的 `end_of_route` TF 数据还没有补录。
- 一个 planner 进程只运行一个场景，切换场景需要重启节点。

下一阶段先打通静态障碍物的 YAML 解析、ROS2 发布、RViz 可视化和 planner 输入。终点停车
放到后续 Behavior Planner / PlanningTarget 阶段处理，不把 fallback deceleration 当作
正常停车规划。
