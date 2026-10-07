# ROS2 PNC Planner

一个用于自动驾驶 PNC（Planning and Control）学习与作品集展示的 ROS2 C++ 局部规划与控制原型项目。

当前项目围绕一条较完整的基础闭环展开：参考线生成、Cartesian-Frenet 坐标转换、Lattice 局部轨迹规划、轨迹跟踪、自车仿真和 RViz 可视化。本项目定位为学习型和作品集型工程项目，不是生产级自动驾驶系统。

## 项目简介

当前系统可以基于 mock route 或订阅到的全局路径生成参考线，由 BehaviorPlanner 给出巡航或路线终点停车目标，再通过 Lattice Planner 采样并选择局部轨迹。Pure Pursuit 和 PID 控制器跟踪轨迹，并在简化自车模型中更新车辆状态。

当前版本以 Lattice Planner 作为 baseline，已具备核心测试、YAML 场景、静态障碍物绕行和最小 CRUISE/STOP 行为层。后续先补自动场景指标和更完整的 Lattice 验证，再扩展最小版本的 EM Planner。

## 当前功能

- 基于 `ament_cmake` 的 ROS2 C++17 package。
- 纯 C++ 规划与控制代码位于 `pnc_planner_core`；ROS2 节点、自车仿真与可视化位于 `pnc_planner_runtime`，核心算法可独立链接测试。
- 基于路径点和样条插值的参考线生成。
- Cartesian-Frenet 坐标转换工具，用于路径相对坐标系下的规划。
- 五次多项式轨迹生成基础模块。
- 可配置约束和代价权重的 Lattice Planner baseline。
- 独立的 BehaviorPlanner 和 PlanningTarget，支持 CRUISE/STOP，并在触发 STOP 后保持停车意图。
- 正常路线终点停车候选生成、连续重规划与控制闭环验证，正常 STOP 和规划失败 fallback 分开处理。
- Pure Pursuit 横向控制。
- PID 纵向控制。
- 简化自车状态仿真。
- RViz 中显示参考线、规划轨迹、停车目标和车辆模型。
- 支持从 YAML 解析并发布静态障碍物，在 RViz 中显示障碍物和有效 Lattice 候选轨迹。
- Lattice Planner 可以过滤与静态障碍物碰撞的候选，并在当前简化场景中完成闭环绕行。
- 提供候选数量、拒绝原因、选中横向目标、代价和单轮规划耗时等运行诊断。
- 支持外部 `/routing_path` 输入和 YAML 场景路线发布。
- 支持通过 `/scenario/initial_state` 发布并应用场景自车初始状态。
- 场景模式下路线、初始状态和障碍物列表均就绪后才启动规划循环。
- 规划失败时会使上一周期轨迹失效，并执行参数化的受控减速，不继续跟踪陈旧轨迹。
- 支持互斥的 mock 输入模式和 YAML 场景模式；一个 planner 进程只接受一个完整场景。
- 已添加核心模块单元测试：`QuinticPolynomial`、`CartesianFrenetConverter`、`ReferenceLine`、`LatticePlanner`、`ScenarioLoader` 和 `BehaviorPlanner`。
- 已建立第一版 scenario YAML schema 和 `scenario_publisher`，用于基础场景复现。

## 系统架构

```text
Routing / Mock Route
        |
        v
ReferenceLine + Spline Interpolation
        |
        v
Cartesian-Frenet Conversion
        |
        v
BehaviorPlanner → PlanningTarget (CRUISE / STOP)
        |
        v
LatticePlanner <--- Static Obstacles
        |
        v
Trajectory
        |
        v
Pure Pursuit + PID Control
        |
        v
EgoVehicle Simulation
        |
        v
RViz Visualization
```

## 目录结构

```text
src/pnc_planner/
  include/pnc_planner/      pnc_planner package 的头文件
  src/                      C++ 实现文件
    controller/             Pure Pursuit 和 PID 控制器
    math/                   样条、Frenet 转换、多项式等数学工具
    planning/               参考线、Lattice 与最小行为规划
    runtime/                ROS2 主节点
    simulation/             带 TF 广播的自车仿真
    visualization/          RViz 发布接口
    scenario/               场景加载与发布
  config/                   规划器运行参数
  launch/                   ROS2 launch 文件
  rviz/                     RViz 配置
  urdf/                     简化车辆模型
  scenarios/                YAML 场景文件
  test/                     核心模块单元测试

docs/architecture.md        系统架构说明
docs/scenario_schema.md     场景 YAML 格式说明
docs/scenario_validation.md 场景验证记录
docs/blog/                  模块开发记录和学习笔记
```

## 快速开始

在工作空间根目录编译 package：

```bash
colcon build --packages-select pnc_planner \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

加载工作空间环境：

```bash
source install/setup.zsh
```

启动 planner、robot state publisher 和 RViz：

```bash
ros2 launch pnc_planner pnc_planner.launch.py
```

仅观察节点日志时也可以直接启动：

```zsh
ros2 run pnc_planner pnc_planner_node --ros-args --log-level pnc_node:=debug
```

`ros2 run` 不会自动加载 `planner_config.yaml`，此时使用节点声明的默认参数；
需要使用 YAML 参数时追加 `--params-file src/pnc_planner/config/planner_config.yaml`。
当前节点默认值与 YAML 中的行为巡航速度均为 5 m/s。

默认配置中关闭了 mock routing，支持通过 `/routing_path` 输入外部路线：

```yaml
use_mock_routing: false
```

### 场景生命周期限制

当前第一版 `PncPlannerNode` 采用单场景生命周期：

```text
一个 planner 进程只运行一个完整场景
第一次成功接收的 route、ego 初始状态和障碍物列表会被锁定
后续重复的场景输入会被拒绝
切换场景前需要重启 planner
```

路线、初始状态和障碍物虽然来自同一个 YAML，但运行时通过独立 topic 发布。节点不会把后续路线和旧的自车状态静默组合，避免出现“新路线 + 旧 ego”的场景错配。

当 `use_mock_routing: true` 时，节点内部创建 mock route 和 mock ego，并忽略外部场景 topic。mock 模式与 YAML 场景模式不应同时使用。

规划器参数位于：

```text
src/pnc_planner/config/planner_config.yaml
```

## 运行测试

运行功能 gtest：

```bash
colcon test-result --delete-yes
colcon test \
  --packages-select pnc_planner \
  --event-handlers console_direct+ \
  --ctest-args -R '^test_' -V
colcon test-result --verbose
```

当前核心测试覆盖：

```text
QuinticPolynomial 边界条件
CartesianFrenet 直线参考线坐标转换
ReferenceLine 初始化、查询和边界处理
LatticePlanner 基础规划、失败输出契约、静态避障、终端安全和连续重规划
ScenarioLoader 场景 ego 与静态障碍物解析、校验和非法输入检查
BehaviorPlanner 巡航/停车决策、STOP 锁存和配置校验
```

最近一次功能 gtest 回归（2026-10-03）六个 target、51 个用例全部通过，覆盖基础数学、
参考线、场景解析、行为决策、静态障碍物路径和正常停车连续重规划。
后续修改后应重新运行测试；`colcon test-result --verbose` 读取的是已有结果文件。

完整质量检查可以不带 `-R '^test_'` 过滤重新运行。它还包含格式、Python 和 XML lint。
当前仓库存在全局格式规则不一致以及离线 XML schema 问题，因此应将功能 gtest 与完整质量
检查分开查看。

## 场景验证

项目包含第一版 YAML 场景格式和场景发布器。场景文件位于：

```text
src/pnc_planner/scenarios/
```

场景格式说明见：

```text
docs/scenario_schema.md
```

当前已人工/RViz 验证的基础场景：

| Scenario | Purpose | Status |
|---|---|---|
| `straight_cruise` | 直线巡航 | Pass |
| `curve_cruise` | 缓弯巡航 | Pass |
| `end_of_route` | 正常路线终点停车 | Pass：stop_s=18 m，最终 s≈17.999 m，v=0，无 fallback |
| `static_obstacle_avoid` | 静态障碍物闭环绕行 | Pass |
| `static_obstacle_blocked` | 完全阻塞后的安全降级 | Pass |

2026-10-03 复测中，直线、曲线和绕行场景均在路线末端正常停车，没有触发 fallback。
阻塞场景的障碍物中心现为 x=12 m，fallback 后自车停在 x≈4.415 m；
沿中心线执行的最小中心距离约 7.585 m，超过当前简化模型阈值 5.5 m。
场景通过状态来自人工/RViz 和节点日志验证，尚未自动采集全部 expected 指标。

详细验证记录见：

```text
docs/scenario_validation.md
```

## 当前局限

- 当前项目仍是早期局部规划与控制原型。
- 当前规划 baseline 是 Lattice Planner，尚未实现完整 EM Planner。
- 当前核心测试已覆盖基础数学、参考线、场景解析和静态障碍物 Lattice 规划，但尚未形成系统级自动指标评估。
- 当前已有 YAML 场景、ScenarioLoader 和 ScenarioPublisher，但 expected 指标仍未自动采集或自动判定。
- 行为规划只支持路线终点 CRUISE/STOP，尚未实现 FOLLOW、YIELD 或正常障碍物停车；VehicleState 继续描述仿真自车状态。
- 停稳后仍周期性规划和输出日志，停车完成状态、低速阈值边界和微小控制命令有待进一步整理。
- 当前场景生命周期要求切换 YAML 场景前重启 planner，尚未实现 reset 或 batch runner。
- 当前仅支持静态障碍物，碰撞检测仍采用简化距离模型，尚未支持动态障碍物预测。
- 当前横向采样可配置但仍是离散集合，尚未实现动态可行域生成。
- STOP 时长搜索跨度目前为工程固定值，不能保证任意初始状态都有可行候选。

## 后续计划

- 后续扩展场景 runner 和 metrics，使 `expected` 字段能够自动判定。
- 在已分离的核心算法和 ROS2 运行时边界上补充自动场景指标，并保持现有场景回归。
- 完善停车完成状态、日志和低速边界处理，再独立扩展正常障碍物停车行为。
- 继续完善 Lattice baseline 的代价分解和自动运行指标。
- 增加动态障碍物预测及相应的时空碰撞检查。
- 在当前 baseline 稳定后扩展最小版本 EM Planner。
- 增加控制器对比和轨迹跟踪误差指标。

## 学习记录

`docs/blog/` 目录中记录了一些模块开发过程和学习笔记，包括：

- Cartesian-Frenet 坐标转换。
- `PncPlannerNode` 模块开发记录。
- `EgoVehicle` 自车仿真模块开发记录。
- RViz 可视化模块开发记录。
- 样条实现学习过程中涉及的追赶法 C++ 实现笔记。
