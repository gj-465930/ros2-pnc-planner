# Scenario Execution Design

本文档定义 Mini PNC Lab 第一版场景执行路径。目标是让 `scenarios/*.yaml` 不再只是静态数据，而是能够稳定、可重复地驱动现有规划节点运行，并在 RViz 中观察结果。

当前阶段优先建立最小可运行闭环，不在同一个任务中引入完整场景框架、障碍物链路或规划算法重构。

## 候选方案

### 方案一：C++ ScenarioLoader

C++ `ScenarioLoader` 负责解析 YAML，并转换为统一的场景数据结构：

```text
scenario.yaml
      ↓
ScenarioLoader
      ↓
Scenario
  ├── route
  ├── ego
  ├── obstacles
  └── expected
```

优点：

```text
与现有 C++ 核心模块结合紧密
数据类型和错误检查更明确
便于编写 C++ 单元测试
适合后续完整场景执行和自动指标验证
```

缺点：

```text
需要引入 yaml-cpp 及对应的 CMake/package 依赖
需要提前设计 Scenario、Route、Obstacle、Expected 等数据结构
当前 PncPlannerNode 只有路线 topic 输入，ego、obstacles 和 expected 尚无完整注入接口
现在实现容易把简单场景任务扩大为节点接口和规划流程重构
```

### 方案二：Lightweight ROS2 Publisher

轻量 publisher 读取 YAML 中的路线点，将其转换为 `nav_msgs/msg/Path`，并发布到现有 `/routing_path` topic：

```text
scenario.yaml
      ↓
scenario_publisher
      ↓ nav_msgs/msg/Path
/routing_path
      ↓
PncPlannerNode
      ↓
ReferenceLine → LatticePlanner → RViz
```

优点：

```text
可以复用 PncPlannerNode 已有的 /routing_path 订阅接口
实现范围小，能较快形成可复现场景闭环
不需要修改 ReferenceLine、LatticePlanner 或控制器
适合先验证 YAML 路线、参考线生成、规划和 RViz 显示
```

缺点：

```text
第一版只能真正注入 route
ego 初始状态仍由 PncPlannerNode 内部初始化
obstacles 暂时没有输入链路
expected 暂时不能自动计算和判定
Python publisher 与 C++ 核心测试体系的结合较弱
```

## 第一版选择

第一版选择 **lightweight Python/ROS2 publisher**。

选择原因是当前 `PncPlannerNode` 已经订阅 `/routing_path`，路线消息进入节点后可以直接初始化 `ReferenceLine`，无需修改规划算法。这样可以用最小改动完成第一条可复现链路：

```text
选择 YAML 场景
→ 发布路线
→ 生成参考线
→ 执行 Lattice 规划和轨迹跟踪
→ 在 RViz 中观察结果
```

该方案是 Phase 3 的阶段性实现，不代表放弃 C++ `ScenarioLoader`。第一版的重点是验证执行路径和场景数据是否合理，后续在输入接口成熟后再升级完整场景加载架构。

## 第一版支持范围

第一版 publisher 应当：

```text
接收 scenario YAML 文件路径
检查 schema_version 是否为 0.1
读取 name 和 route
检查 frame_id
检查 route.points 至少包含 3 个点
检查每个点包含两个有限数值 x、y
构造并发布 nav_msgs/msg/Path 到 /routing_path
输出清晰的加载成功或失败日志
```

各 YAML 字段在第一版中的状态：

```text
route       读取并实际发布
ego         可检查字段是否存在，但暂不注入 PncPlannerNode
obstacles   暂不执行，基础场景仅允许空列表
expected    保留为验收目标，暂不自动判定
```

publisher 不应默默忽略暂不支持的非空字段。例如遇到非空 `obstacles` 时，应明确提示第一版尚不支持该场景，而不是让用户误以为障碍物已经参与规划。

## 计划新增文件

后续实现任务预计涉及：

```text
src/pnc_planner/scripts/scenario_publisher.py
```

并按实际实现需要小范围更新：

```text
src/pnc_planner/CMakeLists.txt
src/pnc_planner/package.xml
src/pnc_planner/launch/pnc_planner.launch.py
```

是否新增独立 launch 文件，应在实现任务中根据启动方式决定。本设计任务不创建 publisher、不修改构建配置，也不修改现有节点。

## 依赖影响

Python 方案预计需要：

```text
rclpy
nav_msgs
geometry_msgs
Python YAML 解析库（PyYAML）
```

实现时需要在 `package.xml` 中声明运行依赖，并通过 CMake 将 Python 脚本安装为 ROS2 可执行程序。具体依赖名称和安装规则应在实现任务中结合当前 Robostack 环境确认。

## 验证方式

第一版至少验证三个基础场景：

```text
straight_cruise.yaml
curve_cruise.yaml
end_of_route.yaml
```

每个场景应完成以下检查：

```text
publisher 能成功读取 YAML
/routing_path 能收到 nav_msgs/msg/Path
PncPlannerNode 日志显示参考线初始化成功
RViz 能显示与 YAML points 对应的平滑参考线
规划轨迹能够被发布和显示
错误文件、少于 3 个路线点或非法坐标会得到明确报错
```

`expected` 的 `collision_free`、`reach_goal`、`max_abs_l`、`max_acc`、`max_decel` 和 `timeout_sec` 在第一版中只作为人工检查依据，不能描述为已经通过自动验证。

## 暂不实现

本阶段暂不实现：

```text
C++ ScenarioLoader
统一 C++ Scenario 数据结构
ego 初始状态外部注入
障碍物发布和碰撞检测链路
expected 自动指标收集与判定
场景批量运行和回归报告
PncPlannerNode 或 LatticePlanner 重构
```

## 升级到 C++ ScenarioLoader

C++ `ScenarioLoader` 是后续明确的演进方向。当满足下列任一关键需求，且相关输入接口已经具备时，应创建独立任务进行升级：

```text
需要从 YAML 设置 EgoVehicle 初始状态
障碍物输入链路已经实现，需要加载 obstacles
开始自动计算并判定 expected 指标
需要批量运行场景并生成回归测试结果
需要在 C++ 单元测试或集成测试中直接加载场景
```

建议的演进顺序：

```text
lightweight publisher
→ 三个基础 YAML 场景可重复运行
→ ego 初始状态可注入
→ obstacle pipeline
→ expected metrics
→ C++ ScenarioLoader 和统一 Scenario 数据结构
→ 自动场景回归测试
```

升级任务开始前，应重新评估 `Scenario` 数据结构、yaml-cpp 依赖、错误处理方式，以及 loader 与 ROS2 node 之间的职责边界。`ScenarioLoader` 应负责解析和校验数据，不应直接包含 LatticePlanner 算法逻辑。
