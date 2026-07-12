# Scenario Execution Design

本文档定义 Mini PNC Lab 第一版场景执行路径。目标是让 `scenarios/*.yaml` 能够稳定、可重复地驱动现有规划节点运行，并在 RViz 中观察结果。

当前阶段优先建立最小可运行闭环，不在同一个任务中引入完整评测框架、障碍物链路或规划算法重构。

## 候选方案

### 方案一：完整 C++ ScenarioLoader

完整 Loader 一次解析全部场景字段，并转换为统一的场景数据结构：

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
数据类型和错误检查明确
便于编写 C++ 单元测试和集成测试
适合后续完整场景执行和自动指标验证
```

缺点：

```text
需要引入 yaml-cpp 及对应的 CMake/package 依赖
需要提前设计完整 Scenario 数据结构
当前 PncPlannerNode 只有路线 topic 输入
ego、obstacles 和 expected 尚无完整注入或验证接口
现在实现全部字段会扩大 Task 015 的范围
```

### 方案二：单一 Lightweight Publisher

publisher 自己解析 YAML、构造 `nav_msgs/msg/Path`，然后发布到 `/routing_path`：

```text
scenario.yaml
      ↓
ScenarioPublisher（解析 + 发布）
      ↓
/routing_path
```

优点是文件少、接入快，可以直接复用 `PncPlannerNode` 已有的 `/routing_path` 订阅接口。缺点是 YAML 解析和 ROS2 通信耦合在一个类中，未来实现完整 Loader 时还要重新拆分，解析逻辑也不方便脱离 ROS2 单独测试。

### 方案三：最小 C++ Loader + Lightweight C++ Publisher

将数据解析与 ROS2 发布分开，但第一版只实现路线所需的最小数据范围：

```text
scenario.yaml
      ↓
ScenarioLoader
      ↓ ScenarioData
ScenarioPublisher
      ↓ nav_msgs/msg/Path
/routing_path
      ↓
PncPlannerNode
      ↓
ReferenceLine → LatticePlanner → RViz
```

这种方案需要引入 `yaml-cpp`，文件数量也比单一 publisher 多，但职责边界清楚。后续可以直接扩展 Loader 和 `ScenarioData`，不必从 ROS2 节点中重新抽取 YAML 解析代码。

## 第一版选择

第一版选择 **最小 C++ ScenarioLoader + lightweight C++ ScenarioPublisher**。

选择原因：

```text
项目主体使用 ROS2 C++，保持技术栈统一
用户希望通过 C++ 学习和维护 ROS2 节点
PncPlannerNode 已经订阅 /routing_path，可以直接复用现有输入接口
解析与发布分离，便于测试并为后续完整 Scenario 系统保留扩展路径
只实现 route，仍然可以控制第一版范围
```

第一版执行链路为：

```text
选择 YAML 场景
→ ScenarioLoader 读取并校验路线
→ ScenarioPublisher 发布 /routing_path
→ PncPlannerNode 初始化 ReferenceLine
→ 执行 Lattice 规划和轨迹跟踪
→ 在 RViz 中观察结果
```

`ScenarioLoader` 负责回答“文件中包含什么有效数据”，`ScenarioPublisher` 负责回答“如何把这些数据送入 ROS2 系统”。Loader 不发布 topic，Publisher 不直接解析 YAML 细节。

## 第一版数据结构

第一版只需要保存路线发布所需的数据。可以使用最小的 `ScenarioData`，至少包含：

```text
schema_version
name
route.frame_id
route.points
```

第一版不必为所有未来字段建立复杂类层次。`ego` 和 `expected` 可以保留在 YAML 中并做必要的存在性检查，但暂时不进入规划流程；非空 `obstacles` 应被明确拒绝，因为当前系统尚未实现障碍物输入链路。

## 第一版职责

### ScenarioLoader

```text
接收 scenario YAML 文件路径
使用 yaml-cpp 加载文件
检查 schema_version 是否为 0.1
读取 name 和 route
检查 frame_id 第一版是否为 map
检查 route.points 至少包含 3 个点
检查每个点包含两个有限数值 x、y
遇到非空 obstacles 时明确报错
返回 ScenarioData 或清晰的失败信息
```

### ScenarioPublisher

```text
声明 scenario_file ROS2 参数
调用 ScenarioLoader 加载场景
将 route.points 转换为 nav_msgs/msg/Path
等待 /routing_path 存在订阅者后发布路线
输出场景名称、文件路径和路线点数
不负责生成参考线或调用 LatticePlanner
```

各 YAML 字段在第一版中的状态：

```text
route       由 Loader 解析，并由 Publisher 实际发布
ego         有明确的 YAML 来源，但暂不注入 PncPlannerNode
obstacles   只允许空列表，非空时拒绝执行
expected    保留为人工验收目标，暂不自动判定
```

## 计划新增文件

后续实现任务预计涉及：

```text
src/pnc_planner/include/pnc_planner/scenario/scenario_data.hpp
src/pnc_planner/include/pnc_planner/scenario/scenario_loader.hpp
src/pnc_planner/src/scenario/scenario_loader.cpp
src/pnc_planner/src/scenario_publisher.cpp
```

并按实际实现需要小范围更新：

```text
src/pnc_planner/CMakeLists.txt
src/pnc_planner/package.xml
```

第一版可以先通过两个终端分别启动 planner 和 publisher，不强制修改现有 launch 文件。是否新增独立场景 launch 文件，留到基础执行链路验证通过后决定。

## 依赖影响

C++ 方案需要：

```text
rclcpp
nav_msgs
geometry_msgs
yaml-cpp
```

现有 package 已经依赖前三项。实现时需要确认当前 Robostack 环境中 `yaml-cpp` 的 CMake package 名称，在 `package.xml` 中声明依赖，并只把 YAML 解析依赖链接到需要它的 target。

## 验证方式

第一版必须先验证：

```text
straight_cruise.yaml
```

执行时应确认：

```text
ScenarioLoader 能成功读取并校验 YAML
ScenarioPublisher 能发布 nav_msgs/msg/Path
/routing_path 能收到路线消息
PncPlannerNode 日志显示参考线初始化成功
RViz 能显示与 YAML points 对应的平滑参考线
规划轨迹能够被发布和显示
错误文件、少于 3 个路线点或非法坐标会得到明确报错
现有单元测试仍然通过
```

基础链路稳定后，再验证：

```text
curve_cruise.yaml
end_of_route.yaml
```

`expected` 中的 `collision_free`、`reach_goal`、`max_abs_l`、`max_acc`、`max_decel` 和 `timeout_sec` 在第一版中只作为人工检查依据，不能描述为已经通过自动验证。

## 暂不实现

本阶段暂不实现：

```text
完整 Scenario 数据结构
ego 初始状态外部注入
障碍物发布和碰撞检测链路
expected 自动指标收集与判定
场景批量运行和回归报告
PncPlannerNode 或 LatticePlanner 重构
Behavior Planner 或 EM Planner
```

## 后续演进

第一版不是未来重新创建 `ScenarioLoader`，而是逐步扩展现有 Loader 和 `ScenarioData`。当相关系统接口成熟后，按以下顺序演进：

```text
最小 Loader + route publisher
→ 三个基础 YAML 场景可重复运行
→ ego 初始状态可注入
→ Loader 解析并应用 ego
→ obstacle pipeline
→ Loader 解析并应用 obstacles
→ expected metrics
→ Loader 提供完整 expected 数据
→ 自动场景回归测试
```

扩展时仍需保持职责边界：

```text
ScenarioLoader    只负责解析和校验场景数据
ScenarioPublisher 只负责 ROS2 输入消息发布
场景 Runner       负责生命周期、超时和执行流程
Metrics           负责收集实际结果并与 expected 比较
Planner           只负责规划，不解析 YAML
```
