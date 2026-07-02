# 项目架构

本文档描述当前 ROS2 PNC Planner 的系统分层、数据流和后续架构演进方向。它的目标不是罗列所有源码文件，而是帮助读者从 PNC 系统角度理解项目如何组织，以及后续为什么要这样扩展。

## 1. 项目定位

本项目是一个 ROS2 C++ 局部规划与控制原型项目，面向自动驾驶 PNC 学习和作品集展示。当前重点是把参考线、Frenet 坐标转换、局部轨迹规划、轨迹跟踪、自车仿真和 RViz 可视化串成一个可运行闭环。

当前规划模块以 Lattice Planner 作为 baseline。EM Planner 不是当前已完成能力，而是后续在测试、场景验证、障碍物链路和行为规划层稳定之后的扩展方向。

## 2. 当前系统闭环

当前版本的主链路如下：

```text
Routing / Mock Route
        |
        v
ReferenceLine
        |
        v
Frenet Conversion
        |
        v
LatticePlanner
        |
        v
Trajectory
        |
        v
Controller
        |
        v
EgoVehicle Simulation
        |
        v
RViz Visualization
```

这条链路对应一个典型的局部规划控制闭环：输入路线，生成参考线，在参考线坐标系附近规划轨迹，再由控制器跟踪轨迹并更新车辆状态。

## 3. 当前模块职责

### Routing / Mock Route

当前系统可以通过 mock route 快速生成一条测试路线，也可以通过 `/routing_path` 订阅外部发布的路径。这个层级的职责是提供离散路径点，不负责轨迹规划细节。

在当前阶段，mock route 主要用于快速验证规划、控制和可视化闭环是否能跑通。后续 scenario runner 建立后，路线输入应逐步由场景文件或场景发布器提供。

### ReferenceLine

`ReferenceLine` 将离散路径点转换为连续可查询的参考线。它提供按弧长 `s` 查询位置、朝向、曲率等信息的能力，也为 Frenet 坐标转换提供基础几何信息。

参考线是 PNC 中非常核心的一层。局部规划通常不直接在原始全局路径点上工作，而是在平滑参考线附近生成候选轨迹。

### Frenet Conversion

Frenet 坐标系把车辆和轨迹点表示为沿参考线方向的纵向坐标 `s` 和横向偏移 `l`。这样可以把二维轨迹规划问题拆成更容易处理的纵向运动和横向偏移问题。

当前系统中，Lattice Planner 会先将 ego 状态投影到参考线附近，再在 Frenet 空间中生成横向和纵向候选轨迹，最后再转换回 Cartesian 坐标用于控制和可视化。

### LatticePlanner

`LatticePlanner` 是当前局部规划 baseline。它的主要职责包括：

- 根据当前车辆状态生成横向候选轨迹。
- 根据车辆状态生成纵向候选轨迹。
- 对候选轨迹进行约束检查。
- 按代价函数选择最优横纵向轨迹组合。
- 将 Frenet 轨迹转换成 Cartesian 轨迹输出。

当前实现主要用于建立 baseline 能力。后续需要增加更清晰的 cost breakdown、候选轨迹调试输出、障碍物约束和场景验证。

### Trajectory

`Trajectory` 是规划器输出给控制器的轨迹结果，由一系列轨迹点组成。每个轨迹点包含位置、朝向、速度、加速度、曲率和时间信息。

在系统分层上，轨迹是规划模块和控制模块之间的接口。规划器只负责生成可行轨迹，控制器负责跟踪轨迹。

### Controller

当前控制层包括 Pure Pursuit 横向控制和 PID 纵向控制。

Pure Pursuit 根据规划轨迹和车辆状态计算横摆角速度命令，用于横向跟踪。PID 纵向控制根据目标轨迹速度和当前车辆速度计算加速度命令。

后续可以加入 Stanley、LQR 等控制器，并建立 tracking metrics，用同一批场景比较不同控制器的跟踪误差和稳定性。

### EgoVehicle Simulation

`EgoVehicle` 是一个简化自车仿真模块，用于根据控制命令更新车辆状态。它让当前项目可以在没有完整仿真器的情况下形成规划-控制-车辆状态更新闭环。

这个模块不是高保真车辆动力学模型，但足够支持早期局部规划和控制效果验证。

### RViz Visualization

`Visualizer` 负责把参考线、规划轨迹和车辆模型发布到 RViz。可视化对于 PNC 项目很重要，因为很多规划问题只看日志很难判断，例如参考线是否平滑、轨迹是否偏离、车辆是否能稳定跟踪。

后续障碍物、候选轨迹、最终轨迹、cost 信息和场景状态也应逐步可视化。

## 4. 当前运行时数据流

当前 `PncPlannerNode` 是主流程入口。一次周期性回调中的逻辑可以概括为：

```text
1. 发布参考线到 RViz
2. 检查参考线是否有效
3. 获取当前 ego vehicle 状态
4. 调用 LatticePlanner 生成局部轨迹
5. 发布规划轨迹到 RViz
6. 控制器根据规划轨迹计算控制命令
7. EgoVehicle 根据控制命令更新车辆状态
```

这说明当前架构已经具备 PNC 闭环雏形，但系统中的许多能力仍处于 baseline 阶段。

## 5. 当前架构局限

当前项目的主要局限包括：

- 核心算法仍直接链接到 node executable，不利于单元测试复用。
- 项目还缺少针对数学、参考线、Frenet 转换和规划模块的系统化单元测试。
- 场景 YAML 文件仍处于早期阶段，尚未形成完整 scenario loader 和 validation runner。
- 障碍物数据结构和 planner 接口已有雏形，但主流程中的障碍物输入、碰撞检查和可视化还没有形成完整链路。
- Behavior Planner 尚未独立，规划目标和行为状态还没有清晰分层。
- 当前 Lattice Planner 还缺少候选轨迹可视化、过滤原因统计和详细 cost breakdown。
- EM Planner 仍是未来方向，当前不能描述为已完成模块。

这些限制并不意味着当前项目没有价值。相反，它们定义了后续工程化和作品集打磨的路线。

## 6. 目标架构

后续希望逐步演进到如下结构：

```text
Scenario / Routing / Obstacles
        |
        v
ReferenceLine
        |
        v
BehaviorPlanner
        |
        v
Local Planner: Lattice Baseline / EM Planner v1
        |
        v
Trajectory
        |
        v
Controller Benchmark
        |
        v
Metrics + RViz Visualization
```

目标架构相比当前架构，多了三个关键能力：

- 场景验证：用 YAML 或其他配置方式复现不同路线、起点、终点和障碍物场景。
- 行为规划：在局部轨迹生成之前先给出巡航、跟车、停车、避障等高层目标。
- 指标评估：不仅能在 RViz 中看起来能跑，还能用 tracking error、collision-free、max acceleration、planning success rate 等指标验证。

## 7. 演进原则

后续开发应围绕四个问题推进：

- 能否测试：核心数学和规划逻辑是否可以通过单元测试验证。
- 能否复现：一个结果是否能由固定场景配置复现。
- 能否可视化：关键中间结果是否能在 RViz 或日志中观察。
- 能否解释：模块职责、算法选择和工程取舍是否能在面试中讲清楚。

因此，项目不应盲目堆叠高级算法名。更合理的路线是先把 Lattice baseline 做到可靠、可测、可复现、可解释，再扩展障碍物链路、行为规划和 EM Planner。

