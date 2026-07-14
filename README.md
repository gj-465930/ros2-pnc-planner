# ROS2 PNC Planner

一个用于自动驾驶 PNC（Planning and Control）学习与作品集展示的 ROS2 C++ 局部规划与控制原型项目。

当前项目围绕一条较完整的基础闭环展开：参考线生成、Cartesian-Frenet 坐标转换、Lattice 局部轨迹规划、轨迹跟踪、自车仿真和 RViz 可视化。本项目定位为学习型和作品集型工程项目，不是生产级自动驾驶系统。

## 项目简介

当前系统可以基于 mock route 或订阅到的全局路径生成参考线，将车辆状态转换到 Frenet 坐标系，在参考线附近通过 Lattice Planner 采样并选择局部轨迹，然后使用 Pure Pursuit 和 PID 控制器跟踪轨迹，并在一个简化自车模型中更新车辆状态。

当前版本以 Lattice Planner 作为 baseline。后续会在测试、场景验证、障碍物链路和行为规划层逐步完善之后，再扩展一个最小版本的 EM Planner。

## 当前功能

- 基于 `ament_cmake` 的 ROS2 C++17 package。
- 核心规划、控制、仿真与可视化代码已抽成 `pnc_planner_core`，便于单元测试复用。
- 基于路径点和样条插值的参考线生成。
- Cartesian-Frenet 坐标转换工具，用于路径相对坐标系下的规划。
- 五次多项式轨迹生成基础模块。
- 可配置约束和代价权重的 Lattice Planner baseline。
- Pure Pursuit 横向控制。
- PID 纵向控制。
- 简化自车状态仿真。
- RViz 中显示参考线、规划轨迹和车辆模型。
- 支持外部 `/routing_path` 输入和 YAML 场景路线发布。
- 已添加核心模块单元测试：`QuinticPolynomial`、`CartesianFrenetConverter`、`ReferenceLine` 和 `LatticePlanner` smoke test。
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
LatticePlanner
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
colcon build --packages-select pnc_planner
```

加载工作空间环境：

```bash
source install/setup.bash
```

启动 planner、robot state publisher 和 RViz：

```bash
ros2 launch pnc_planner pnc_planner.launch.py
```

默认配置中关闭了 mock routing，支持通过 `/routing_path` 输入外部路线：

```yaml
use_mock_routing: false
```

规划器参数位于：

```text
src/pnc_planner/config/planner_config.yaml
```

## 运行测试

运行当前 package 测试：

```bash
colcon test --packages-select pnc_planner --event-handlers console_direct+
colcon test-result --verbose
```

当前核心测试覆盖：

```text
QuinticPolynomial 边界条件
CartesianFrenet 直线参考线坐标转换
ReferenceLine 初始化、查询和边界处理
LatticePlanner 无障碍直线 smoke test
```

当前状态：

```text
12 tests, 0 errors, 0 failures, 0 skipped
```

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
| `end_of_route` | 接近路线终点 | Pass |

详细验证记录见：

```text
docs/scenario_validation.md
```

## 当前局限

- 当前项目仍是早期局部规划与控制原型。
- 当前规划 baseline 是 Lattice Planner，尚未实现完整 EM Planner。
- 当前已有核心模块基础单元测试，但覆盖范围仍以基础数学、参考线和无障碍 Lattice smoke test 为主。
- 当前已有 YAML 场景、ScenarioLoader 和 ScenarioPublisher，但 expected 指标仍未自动采集或自动判定。
- ego 初始状态已写入 YAML，但尚未注入 PncPlannerNode。
- 障碍物输入、碰撞检测和 RViz 障碍物可视化尚未形成完整闭环。
- 行为规划尚未独立成单独的 planning layer。

## 后续计划

- 扩展场景 runner 和 metrics，使 `expected` 字段能够自动判定。
- 将 ego 初始状态接入场景执行流程。
- 打通障碍物输入、碰撞检测和 RViz 可视化链路。
- 增加简单行为规划器和 planning target 抽象。
- 完善 Lattice baseline 的调试输出、代价分解和运行指标。
- 在当前 baseline 稳定后扩展最小版本 EM Planner。
- 增加控制器对比和轨迹跟踪误差指标。

## 学习记录

`docs/blog/` 目录中记录了一些模块开发过程和学习笔记，包括：

- Cartesian-Frenet 坐标转换。
- `PncPlannerNode` 模块开发记录。
- `EgoVehicle` 自车仿真模块开发记录。
- RViz 可视化模块开发记录。
- 样条实现学习过程中涉及的追赶法 C++ 实现笔记。
