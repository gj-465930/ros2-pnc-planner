# Scenario YAML Schema

本文档定义 Mini PNC Lab 第一版场景 YAML 格式。目标是让路线、初始自车状态、障碍物和期望结果可以被固定文件复现，避免项目只依赖手写 mock route。

当前 schema 是 **v0.1**，面向 Phase 3 的基础场景验证。它不追求覆盖真实自动驾驶场景库的全部复杂度，而是优先满足：

```text
可读
可手写
可被 C++ 或 Python 简单解析
可用于后续场景 runner 和 metrics
```

## 顶层结构

```yaml
schema_version: "0.1"
name: straight_cruise
description: Straight road cruise without obstacles.
tags: [basic, straight, cruise]

route:
  frame_id: map
  points:
    - [0.0, 0.0]
    - [20.0, 0.0]
    - [40.0, 0.0]

ego:
  x: 0.0
  y: 0.0
  yaw: 0.0
  v: 5.0
  a: 0.0
  state: CRUISING

obstacles: []

expected:
  success: true
  collision_free: true
  reach_goal: false
  max_abs_l: 0.5
  max_acc: 3.0
  max_decel: 5.0
  timeout_sec: 10.0
```

## 字段说明

### `schema_version`

场景格式版本。第一版固定为：

```yaml
schema_version: "0.1"
```

后续如果新增动态障碍物、交通规则、红绿灯或复杂地图字段，再升级版本。

### `name`

场景名称，应和文件名基本一致。例如：

```yaml
name: straight_cruise
```

命名建议使用小写加下划线。

### `description`

一句话说明场景目的。它不是展示文案，而是帮助开发者快速理解这个场景验证什么。

### `tags`

标签列表，用于后续批量筛选场景。例如：

```yaml
tags: [basic, curve, cruise]
```

### `route`

路线输入。Phase 3 第一版只支持离散路径点：

```yaml
route:
  frame_id: map
  points:
    - [0.0, 0.0]
    - [20.0, 0.0]
    - [40.0, 0.0]
```

要求：

```text
至少 3 个点
点按行驶方向排序
单位为 meter
frame_id 第一版固定使用 map
```

### `ego`

自车初始状态：

```yaml
ego:
  x: 0.0
  y: 0.0
  yaw: 0.0
  v: 5.0
  a: 0.0
  state: CRUISING
```

字段含义：

```text
x, y     初始位置，单位 meter
yaw      初始航向角，单位 rad
v        初始速度，单位 m/s
a        初始加速度，单位 m/s^2
state    自车状态，第一版可用 CRUISING / EMERGENCY / INIT / STANDBY
```

### `obstacles`

障碍物列表。无障碍场景使用空列表：

```yaml
obstacles: []
```

静态障碍物示例：

```yaml
obstacles:
  - id: 1
    type: static
    x: 18.0
    y: 0.0
    yaw: 0.0
    length: 4.5
    width: 2.0
    vx: 0.0
    vy: 0.0
```

字段含义：

```text
id        障碍物编号
type      static 或 dynamic，Phase 3 先只使用 static
x, y      障碍物中心点位置
yaw       障碍物朝向角，单位 rad
length    障碍物长度，单位 meter
width     障碍物宽度，单位 meter
vx, vy    障碍物速度，Phase 3 静态障碍物使用 0.0
```

### `expected`

期望结果和基础指标阈值：

```yaml
expected:
  success: true
  collision_free: true
  reach_goal: false
  max_abs_l: 0.5
  max_acc: 3.0
  max_decel: 5.0
  timeout_sec: 10.0
```

字段含义：

```text
success          期望场景能够正常运行或规划成功
collision_free   期望无碰撞
reach_goal       是否要求到达终点，早期巡航场景可为 false
max_abs_l        最大允许横向偏差，单位 meter
max_acc          最大允许加速度，单位 m/s^2
max_decel        最大允许减速度绝对值，单位 m/s^2
timeout_sec      场景最大运行时间
```

## 第一批场景

Phase 3 第一批只做基础无障碍场景：

```text
straight_cruise.yaml
curve_cruise.yaml
end_of_route.yaml
```

后续障碍物链路进入 Phase 4 后，再扩展：

```text
static_obstacle_stop.yaml
static_obstacle_avoid.yaml
```

## 设计原则

场景文件必须回答四个问题：

```text
这个场景验证什么？
输入路线和自车状态是什么？
期望结果是什么？
后续是否能被 runner 自动复现？
```

如果一个 YAML 只存了几个点，但没有期望结果，它就只是 mock data，不是 scenario。
