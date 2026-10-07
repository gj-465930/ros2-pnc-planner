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
BehaviorPlanner → PlanningTarget (CRUISE / STOP)
        |
        v
LatticePlanner <--- Static Obstacles
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

### BehaviorPlanner / PlanningTarget

`BehaviorPlanner` 根据当前自车状态、参考线剩余距离、制动距离和规划前视给出路线终点
CRUISE/STOP 意图。`PlanningTarget` 是显式接口：CRUISE 携带目标速度，STOP 携带
停车位置 `stop_s`，要求终端速度为零。STOP 触发后在当前单场景生命周期内锁存。

VehicleState 描述仿真自车状态，PlanningTarget 描述规划意图。正常 STOP 通过有效轨迹
和控制器执行；规划失败由节点清空轨迹并执行 fallback 减速，两条路径独立。

### LatticePlanner

`LatticePlanner` 是当前局部规划 baseline。它的主要职责包括：

- 根据当前车辆状态生成横向候选轨迹。
- 根据车辆状态和 PlanningTarget 生成巡航或正常停车纵向候选。
- 对候选轨迹进行约束检查。
- 按代价函数选择最优横纵向轨迹组合。
- 将 Frenet 轨迹转换成 Cartesian 轨迹输出。

当前已有静态障碍物过滤、终端安全检查、候选可视化、拒绝诊断和正常停车连续重规划测试。
STOP 时长从估算值开始搜索，至少保留 1 s 的采样跨度；估算值不构成 jerk 可行性保证，
每个候选仍需经过约束检查。后续补充更清晰的 cost breakdown 和自动场景指标。

### Trajectory

`Trajectory` 是规划器输出给控制器的轨迹结果，由一系列轨迹点组成。每个轨迹点包含位置、朝向、速度、加速度、曲率和时间信息。

在系统分层上，轨迹是规划模块和控制模块之间的接口。规划器只负责生成可行轨迹，控制器负责跟踪轨迹。

### Controller

当前控制层包括 Pure Pursuit 横向控制和 PID 纵向控制。

Pure Pursuit 根据规划轨迹和车辆状态计算横摆角速度命令，用于横向跟踪。
PID 纵向控制结合速度误差反馈与轨迹加速度前馈计算加速度命令。

后续可以加入 Stanley、LQR 等控制器，并建立 tracking metrics，用同一批场景比较不同控制器的跟踪误差和稳定性。

### EgoVehicle Simulation

`EgoVehicle` 是一个简化自车仿真模块，用于根据控制命令更新车辆状态。它让当前项目可以在没有完整仿真器的情况下形成规划-控制-车辆状态更新闭环。

周期内按线性变化的加速度积分；速度过零时只积分到停止时刻，并用低速死区避免静止时
持续累计位移。该模块不是高保真车辆动力学模型。

### RViz Visualization

`Visualizer` 负责把参考线、规划轨迹、有效候选、静态障碍物、停车目标和车辆模型发布到 RViz。可视化对于 PNC 项目很重要，因为很多规划问题只看日志很难判断，例如参考线是否平滑、轨迹是否偏离、车辆是否能稳定跟踪。

更详细的 cost 分解和自动场景指标尚待扩展。

## 4. 当前运行时数据流

当前 `PncPlannerNode` 是主流程入口。一次周期性回调中的逻辑可以概括为：

```text
1. 等待 route、ego 初始状态和障碍物列表就绪，检查参考线
2. 发布参考线和静态障碍物到 RViz
3. 获取当前 ego vehicle 状态
4. 调用 BehaviorPlanner，获取 PlanningTarget 并显示停车目标
5. 调用 LatticePlanner，成功时发布候选和规划轨迹，再由控制器计算命令
6. 行为或轨迹规划失败时清空旧轨迹，使用独立 fallback 减速命令
7. EgoVehicle 更新车辆状态，记录行为、规划结果和执行状态
```

这说明当前架构已经具备 PNC 闭环雏形，但系统中的许多能力仍处于 baseline 阶段。

## 5. 当前架构局限

当前项目的主要局限包括：

- 已有可复用库和核心测试，但算法、仿真、可视化与 ROS2 依赖还需要单独整理边界和目录。
- 场景已有 YAML loader 和 publisher，尚无自动 validation runner 和 metrics。
- 碰撞检查使用简化中心距离模型，未实现完整矩形几何、连续碰撞检测或动态预测。
- BehaviorPlanner 只支持路线终点 CRUISE/STOP，尚未扩展跟车、让行和正常障碍物停车。
- 当前 Lattice Planner 仍需更详细的 cost breakdown 和更广的状态覆盖。
- 停车完成状态、低速阈值边界、静止命令和日志生命周期仍可完善。
- EM Planner 仍是未来方向，当前不能描述为已完成模块。

这些限制并不意味着当前项目没有价值。相反，它们定义了后续工程化和作品集打磨的路线。

## 6. 后续演进

当前已具备 YAML 场景输入、CRUISE/STOP 行为目标和 Lattice 规划闭环。后续希望逐步扩展到如下结构：

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

后续重点是：

- 自动场景评估：在已有 YAML 场景基础上采集并判定跟踪误差、碰撞、加速度和规划成功率等指标。
- 更丰富的行为目标：在当前路线终点 CRUISE/STOP 之外，逐步评估正常障碍物停车等决策。
- EM Planner v1：在已验证的 Lattice baseline 之外增加第二种局部规划方法。

## 7. 演进原则

后续开发应围绕四个问题推进：

- 能否测试：核心数学和规划逻辑是否可以通过单元测试验证。
- 能否复现：一个结果是否能由固定场景配置复现。
- 能否可视化：关键中间结果是否能在 RViz 或日志中观察。
- 能否解释：模块职责、算法选择和工程取舍是否能在面试中讲清楚。

因此，项目不应盲目堆叠高级算法名。更合理的路线是先把 Lattice baseline 做到可靠、可测、可复现、可解释，再扩展障碍物链路、行为规划和 EM Planner。

## 8. 核心算法与 ROS2 运行时边界

核心层包含数学计算、参考线、BehaviorPlanner、LatticePlanner、已实现的纯 C++ 控制器
和 VehicleInfo、Trajectory、PlanningTarget 等数据类型。核心层通过普通 C++ 数据接口
工作，不负责订阅消息、发布消息或管理 ROS2 定时器。

运行时层包含 PncPlannerNode、Visualizer 和当前带 TF 广播的 EgoVehicle。它负责接收
场景输入，将 ROS 消息转换为核心数据，调用核心算法，执行仿真更新并发布结果。
当前 EgoVehicle 同时承担运动积分和 TF 广播两个职责，先整体归入运行时层；后续若需
独立测试运动模型，再将数值更新与 TF 发布分离。

ScenarioLoader 保持独立场景加载库，依赖 yaml-cpp，不依赖 ROS2 节点。
ScenarioPublisher 负责把加载出的场景数据转换为 ROS 消息并发布。

当前构建依赖如下，箭头表示“依赖”：

```text
pnc_planner_node → pnc_planner_runtime → pnc_planner_core

scenario_publisher → pnc_scenario_loader
scenario_publisher → ROS2 消息与发布接口
```

核心库不反向依赖运行时库。LatticePlanner 只需要自车状态、参考线、障碍物和规划目标，
无需知道这些数据来自 ROS topic、YAML 还是单元测试。节点对象和发布接口留在运行时层。

`common.hpp` 已移除无必要的 ROS 消息 include；节点、Visualizer 和 EgoVehicle 已从
`pnc_planner_core` 移入 `pnc_planner_runtime`。参考线与 Lattice 源文件位于 `planning/`，
节点位于 `runtime/`，自车仿真位于 `simulation/`，可视化位于 `visualization/`。
`main.cpp` 只负责创建并运行节点。文件移动没有改变类名、命名空间、算法参数、topic、
launch 或单场景输入契约；构建、六个核心测试目标和五个 YAML 场景已完成回归。
