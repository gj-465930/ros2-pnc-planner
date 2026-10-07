# 场景验证记录

这里记录第一版 YAML 场景链路的运行和排查结果。目前主要通过 ROS2 topic、节点日志、TF
和 RViz 检查运行状态，还没有接入自动 metrics 和批量测试。

## 架构重构回归（2026-10-07）

将代码拆为 `pnc_planner_core`、`pnc_planner_runtime` 和节点入口，并整理规划、运行时、
仿真与可视化目录后，用户确认包构建及六个核心 gtest target 均通过。`ros2 run` 搭配
参数文件验证了节点入口和 `end_of_route` 正常 STOP；`ros2 launch` 搭配场景发布器验证了
RViz/TF 组合入口和 `static_obstacle_avoid` 绕行。重启节点分别运行
`static_obstacle_blocked`、`straight_cruise`、`curve_cruise`，人工检查结果均符合预期。

| 场景 | 本轮人工检查 |
|---|---|
| `end_of_route` | 正常 STOP，无 planning-failure fallback |
| `static_obstacle_avoid` | 绕过障碍物并回到中心线，终点正常停车 |
| `static_obstacle_blocked` | 无有效候选，进入 -3 m/s² fallback 并减速停车 |
| `straight_cruise` | 沿中心线巡航，终点正常停车 |
| `curve_cruise` | 沿曲线参考线巡航，终点正常停车 |

本轮是重构后的人工场景回归；下文 2026-10-03 的精确停车位置和日志文件名属于当时的
基线记录，不能当作本轮重新采集的数值。`expected` 字段仍未由自动 runner 或 metrics
判定；旧轨迹清空由代码和单元测试覆盖，首轮即失败的阻塞场景不能单独证明这一点。
阻塞场景的无碰撞结论仍受简化距离模型限制。

## 阶段 5 数值基线（2026-10-03）

正常路线终点停车已实现并完成闭环验证。BehaviorPlanner 给出 CRUISE/STOP 目标，
LatticePlanner 生成对应轨迹，控制器执行正常停车；规划失败后的 fallback 仍是独立路径。
本节保留当时的数值结果，后文历史记录保留此前的失败现象和排查依据。

本轮主要参数：规划时域 5 s，巡航目标速度 5 m/s，横向采样 `[3.5, 0.0, -3.5]`，
横向过渡距离 12 m，终点缓冲 2 m，fallback 减速度 -3 m/s²。
STOP 时长采样以估算时长为起点，至少向后搜索 1 s，步长为 0.025 s；
候选仍需通过速度、加速度、jerk、碰撞和终端安全检查。

### 核心自动测试

六个 gtest target、51 个用例全部通过，0 failures、0 errors：

| 模块 | 通过用例 |
|---|---:|
| QuinticPolynomial | 2/2 |
| CartesianFrenet | 4/4 |
| ReferenceLine | 5/5 |
| LatticePlanner | 19/19 |
| ScenarioLoader | 16/16 |
| BehaviorPlanner | 5/5 |

覆盖连续停车重规划、STOP 锁存、移动自车不能被过早替换为保持轨迹、停车点附近投影精度、
静态避障和规划失败清空输出。功能测试通过不等于全仓库 lint 已通过。

```zsh
colcon build --packages-select pnc_planner \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.zsh
colcon test --packages-select pnc_planner \
  --event-handlers console_direct+ \
  --ctest-args -R '^test_(behavior_planner|lattice_planner|reference_line|cartesian_frenet|quintic_polynomial|scenario_loader)$' \
  --output-on-failure
```

### 场景闭环结果

| 场景 | 结果 | 最终状态与依据 |
|---|---|---|
| end_of_route | Pass | x≈17.99921 m，s≈17.999 m，stop_s=18 m，v=0，无 fallback |
| straight_cruise | Pass | 中心线巡航后正常 STOP；s≈57.999 m，stop_s=58 m，v=0，无 fallback |
| curve_cruise | Pass | 曲线巡航后正常 STOP；s≈39.026 m，stop_s≈39.02694 m，v=0，无 fallback |
| static_obstacle_avoid | Pass | 持续选择 l=3.5 后恢复 l=0；s≈57.998 m，stop_s=58 m，v=0，无 fallback |
| static_obstacle_blocked | Pass | 无有效候选，执行 fallback；x≈4.415 m，v=0，停稳后位置不变 |

直线、曲线和绕行场景在扩展 STOP 时长采样后重新运行，分别在独立 ROS_DOMAIN_ID 下
执行约 35 s。日志验证了候选选择、状态更新和正常停车；绕行可视化沿用此前人工/RViz
证据，本轮没有新增自动轨迹净距或跟踪误差指标。

### 正常终点停车

`end_of_route` 的初始状态现为 x=14 m、v=3 m/s、a=0，参考线长 20 m，停车目标 s=18 m。
该起点即满足 STOP 触发条件，首轮直接进入 STOP；CRUISE → STOP 的切换由另外三个
巡航场景和 BehaviorPlanner 测试覆盖。停车全过程未出现 planning failed 或 fallback。
最终 s 误差约 -0.001 m，速度归零，后续保持轨迹规划成功。

最新日志证据：

```text
Behavior changed to STOP stop_s=18.00, remaining_distance=4.00
[cycle=161][STOP] x=17.99921, s=17.99900, stop_s=18.00000, error=-0.00100, v=0.000000
```

### 阻塞场景与 fallback

`static_obstacle_blocked` 的障碍物中心由 (5, 0) 调整为 (12, 0)，length=8 m、width=4 m，
初始自车为 x=0、v=5 m/s。旧位置的中心距离 5 m 已小于当前碰撞阈值 5.5 m，
旧记录只能证明失败处理，不能证明无碰撞停车。

新场景首轮 15 组候选全部被过滤，节点失败分支清空已保存轨迹并执行 -3 m/s² fallback。
完整运行未出现 plan ok，持续减速，到 cycle=164 停在 x≈4.415 m，随后位置保持稳定。
清空轨迹的契约由代码检查及已有 gtest 覆盖。

当前简化碰撞判定使用中心距离阈值 `(3.0 + obstacle.length) / 2.0 = 5.5 m`。
该场景初始 y=0、yaw=0，fallback 横摆角速度为零；结合最大前进位置，最小中心距离
约为 `12 - 4.415 = 7.585 m`，大于阈值，余量约 2.085 m。
这是基于本次直线执行和当前距离模型的推算，尚未接入自动碰撞指标，也不等于
矩形几何或连续碰撞检测验证。

该场景验证的是规划失败后的 fallback 停车，不代表已实现正常障碍物 STOP 决策。

### 证据来源与剩余限制

本地节点日志：

```text
end_of_route: pnc_planner_node_19061_1791023833119.log
straight_cruise: pnc_planner_node_24295_1791025011690.log
curve_cruise: pnc_planner_node_24293_1791025011691.log
static_obstacle_avoid: pnc_planner_node_24294_1791025011691.log
static_obstacle_blocked: pnc_planner_node_22415_1791024627997.log
```

这些日志位于本地 ROS2 日志目录，不作为仓库必需文件。`expected` 字段仍是人工检查目标，
没有自动 runner 或 metrics。停止后仍周期性规划和输出日志；低速死区、保持轨迹边界、
微小控制命令和 debug 标签统一留待后续整理。动态障碍物和 EM Planner 尚未实现。

场景执行链路：

```text
scenario YAML
      ↓
ScenarioLoader
      ↓
scenario_publisher
      ├── /routing_path
      ├── /scenario/initial_state
      └── /scenario/obstacles
                    ↓
             PncPlannerNode
                    ↓
       ReferenceLine / BehaviorPlanner / LatticePlanner / EgoVehicle
```

YAML 中的 `collision_free`、`max_abs_l`、`max_acc` 等 `expected` 字段暂时只作为人工
检查目标。没有实际采集到的指标，不计为通过。

## 验证范围

基础回归场景均为无障碍物场景：

```text
straight_cruise.yaml
curve_cruise.yaml
end_of_route.yaml
```

静态障碍物阻塞场景：

```text
static_obstacle_blocked.yaml
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

历史测试记录来自 2026-07-27，共 5 个 gtest target、19 个测试，全部通过；最新结果见本文开头：

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
| `end_of_route` | 从 x=14 m、v=3 m/s 启动，在 s=18 m 附近正常停车 | Pass |
| `static_obstacle_blocked` | 障碍物阻塞所有当前 Lattice 候选并触发受控减速 | Pass |
| `static_obstacle_avoid` | 提前横移绕过静态障碍物并返回中心线 | Pass |

### `straight_cruise`

`scenario_publisher` 可以正常发布直线路线，`PncPlannerNode` 能完成参考线初始化。RViz 中
可以看到沿 x 轴生成的参考线和规划轨迹。该场景作为最基础的无障碍回归场景。

### `curve_cruise`

节点可以使用 YAML 路线更新 `ReferenceLine`，RViz 中的参考线和规划轨迹都呈缓弯形态。
该场景用于组合检查参考线插值、Frenet 转换和轨迹跟踪。

## `end_of_route` 历史问题记录

以下记录对应旧版 x=16 m 场景和正常停车实现前的行为，不代表当前状态；
当前正常停车已通过，最新输入和结果见本文开头。

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

### 当时结论

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

## `static_obstacle_blocked` 历史验证记录

以下为障碍物中心 x=5 m 时的旧记录。该输入初始已进入当前碰撞模型的判定范围，
因此只能作为轨迹失效和减速链路的历史证据；当前 x=12 m 的复测结论见本文开头。

场景输入：

```text
路线：x = 0 m → 20 m → 40 m → 60 m
ego.x = 0.0 m
ego.y = 0.0 m
ego.yaw = 0.0 rad
ego.v = 5.0 m/s
ego.a = 0.0 m/s²
state = CRUISING
障碍物：id=1，中心 (5.0, 0.0)，length=8.0 m，width=4.0 m，heading=0.0 rad
planning_failure_fallback_decel = -3.0 m/s²
```

场景通过 `scenario_publisher` 发布后，`/scenario/obstacles` 中的障碍物数据与 YAML
一致，RViz 中可以看到位于 `(5.0, 0.0)` 的静态障碍物方块。

节点节流日志显示：

```text
Planning failed: ego=(0.00, 0.00), yaw=0.00, lat=3, lon=5,
evaluated=15, valid=0, kinematic=0, conversion=0, collision=15,
terminal=0; cleared stale trajectory and applying fallback decel -3.00
```

节点每 100 ms 重新规划一次，而障碍物持续阻塞当前候选集合。失败摘要使用节流日志，
最多每秒输出一次；重复进入失败分支不表示程序崩溃。

通过 `ros2 run tf2_ros tf2_echo map base_link` 观察到车辆位置最终稳定在：

```text
Translation: [4.173, 0.000, 0.000]
```

之后的位置保持不变，说明车辆没有继续跟踪陈旧轨迹。该结果也与简化运动学估算一致：

```text
d = v² / (2|a|) = 5² / (2×3) ≈ 4.17 m
```

### 结论

| 检查项 | 结果 | 依据 |
|---|---|---|
| 障碍物 YAML 解析与 topic 发布 | Pass | `/scenario/obstacles` 数据与 YAML 一致 |
| RViz 障碍物位置 | Pass | 障碍物显示在 `(5.0, 0.0)` |
| 所有候选轨迹被过滤 | Pass | 日志显示找不到安全轨迹 |
| 陈旧轨迹清除 | Pass | 失败后不再继续跟踪旧轨迹 |
| fallback 受控减速 | Pass | `tf2_echo` 位置稳定在 x≈4.173 m |
| 正常障碍物停车规划 | 未实现 | 当前行为是规划失败后的 fallback deceleration |

该场景证明了静态障碍物完全阻塞时的失败减速链路，不证明无碰撞停车，也不证明已经实现
`STOP_FOR_OBSTACLE` 或其他正常停车行为。

## 当前限制

- `expected` 指标还不能自动采集和判定。
- 尚未实现 batch scenario runner。
- VehicleState 描述仿真自车状态；独立的 BehaviorPlanner 当前只支持路线终点 CRUISE/STOP。
- 动态 TF 只在车辆状态更新时广播，晚启动的订阅者可能错过初始 TF。
- 正常停车已通过上述闭环日志验证，尚未接入自动停车误差和跟踪误差评估。
- 一个 planner 进程只运行一个场景，切换场景需要重启节点。
- 当前只处理静态障碍物，碰撞检查仍采用简化距离模型。

## `static_obstacle_avoid` 历史验证记录

以下为 2026-09-20 的绕行和末端 fallback 记录；本轮已复测正常末端停车，见本文开头。

场景输入：

```text
路线：x = 0 m → 20 m → 40 m → 60 m
ego.x = 0.0 m
ego.y = 0.0 m
ego.yaw = 0.0 rad
ego.v = 5.0 m/s
ego.a = 0.0 m/s²
state = CRUISING
障碍物：id=1，中心 (20.0, 0.0)，length=1.0 m，width=1.0 m，heading=0.0 rad
planning_time = 5.0 s
planning_failure_fallback_decel = -3.0 m/s²
```

通过 `ros2 launch pnc_planner pnc_planner.launch.py` 启动规划节点，并使用
`static_obstacle_avoid.yaml` 发布场景。节点日志确认：

```text
初始化参考线成功，总长度为 60.00
Applied initial state: x=0.00, y=0.00, yaw=0.00, v=5.00, a=0.00, state=CRUISING
Accepted 1 static obstacles
Scenario inputs are complete; starting planning.
```

复测日期：2026-09-20。

本次复测统一使用以下关键参数：

```text
lateral_samples = [3.5, 0.0, -3.5]
planning_time = 5.0 s
lateral_transition_distance = 12.0 m
max_lat_offset = 3.7 m
w_lateral_target_change = 100.0
```

RViz 可以同时观察有效候选轨迹和最终选中轨迹。车辆接近障碍物时，规划日志显示：

```text
evaluated=15, valid=10, selected_l=3.50, duration=5.00
```

车辆持续选择同一侧横向候选并完成绕行。通过障碍物后，选择结果恢复为：

```text
selected_l=0.00
```

车辆随后回到参考线中心附近。单轮规划耗时约为：

```text
planning_ms=0.13~0.47
```

该耗时明显低于 100 ms 的规划周期，没有观察到持续增长。

车辆运行到参考线末端附近时出现：

```text
ego=(38.69, -0.26), yaw=-0.03
evaluated=3, valid=0
collision=0, kinematic=0, terminal=3
```

此时车辆已经越过位于 x=20 m 的障碍物。规划失败来自参考线末端剩余距离不足，所有
候选被终端安全检查拒绝，并非绕行失败。随后节点清空旧轨迹并执行 `-3.0 m/s^2`
fallback deceleration。

### 结果解释

| 检查项 | 结果 | 依据 |
|---|---|---|
| 场景路线和自车初始状态 | Pass | 节点日志与 YAML 输入一致 |
| 静态障碍物接收与显示 | Pass | topic 数据和 RViz 位置与 YAML 一致 |
| 有效候选轨迹可视化 | Pass | RViz 可以观察横向候选及最终轨迹 |
| 碰撞候选过滤 | Pass | 障碍物进入前视范围后有效候选数量减少 |
| 横向选择稳定性 | Pass | 绕行期间持续选择 `l=3.5`，未发生左右跳变 |
| 闭环绕行 | Pass | 车辆纵向越过障碍物后才进入路线末端制动 |
| 回归中心线 | Pass | 通过障碍物后重新选择 `l=0.0` |
| 规划耗时 | Pass | 单轮约 `0.13~0.47 ms`，明显低于 100 ms 周期 |
| 路线末端处理 | Partial | 当前仍通过终端安全拒绝和 fallback 减速停车 |

### 结论

`static_obstacle_avoid` 已完成真实闭环验证。当前 Lattice baseline 能生成多个横向候选，
过滤与静态障碍物冲突的组合，并在连续重规划过程中保持绕行方向，车辆通过障碍物后
回到参考线中心。

该结果只证明当前静态场景和简化碰撞模型下的闭环绕行能力，不代表已经具备动态障碍物
预测、独立行为规划或工业级碰撞检测。路线终点仍未生成正常停车轨迹，fallback
deceleration 继续作为安全降级手段，而不是正常停车规划。
