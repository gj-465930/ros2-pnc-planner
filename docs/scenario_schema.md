# Scenario YAML Schema

场景 YAML 用来固定路线、自车初始状态、障碍物和预期结果，使同一个场景可以重复运行和
检查，而不依赖节点中的临时 mock 数据。

## 版本状态

| Version | Status | Obstacles |
|---|---|---|
| `0.1` | 已实现 | 只允许 `obstacles: []` |
| `0.2` | 格式已确定，运行链路尚未完全接入 | 支持静态矩形障碍物 |

现有的 `straight_cruise`、`curve_cruise` 和 `end_of_route` 继续使用 v0.1。新增静态
障碍物场景使用 v0.2。

## v0.1 示例

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

## v0.2 静态障碍物示例

```yaml
schema_version: "0.2"
name: static_obstacle_avoid
description: Avoid a static obstacle on a straight route.
tags: [obstacle, static, avoid]

route:
  frame_id: map
  points:
    - [0.0, 0.0]
    - [20.0, 0.0]
    - [40.0, 0.0]
    - [60.0, 0.0]

ego:
  x: 0.0
  y: 0.0
  yaw: 0.0
  v: 5.0
  a: 0.0
  state: CRUISING

obstacles:
  - id: 1
    x: 20.0
    y: 0.0
    heading: 0.0
    length: 4.5
    width: 2.0

expected:
  success: true
  collision_free: true
  reach_goal: false
  max_abs_l: 3.5
  max_acc: 3.0
  max_decel: 5.0
  timeout_sec: 10.0
```

## 通用字段

### `schema_version`

格式版本必须是字符串：

```yaml
schema_version: "0.1"
```

兼容规则：

- v0.1 的 `obstacles` 必须为空。
- v0.2 的 `obstacles` 可以为空，也可以包含静态障碍物。
- 动态障碍物、交通规则和信号灯需要新的 schema 版本。

### `name`

场景名称应与文件名一致，使用小写字母和下划线：

```yaml
name: static_obstacle_avoid
```

### `description`

一句话说明场景内容和验证目的。

### `tags`

用于后续筛选或批量运行场景：

```yaml
tags: [obstacle, static, avoid]
```

### `route`

```yaml
route:
  frame_id: map
  points:
    - [0.0, 0.0]
    - [20.0, 0.0]
    - [40.0, 0.0]
```

约束：

- 至少包含 3 个点。
- 路径点按行驶方向排列。
- 坐标单位为 m。
- `frame_id` 当前固定为 `map`。
- 每个坐标都必须是有限数值。

### `ego`

```yaml
ego:
  x: 0.0
  y: 0.0
  yaw: 0.0
  v: 5.0
  a: 0.0
  state: CRUISING
```

| Field | Meaning | Unit |
|---|---|---|
| `x`, `y` | 初始位置 | m |
| `yaw` | 初始航向角，逆时针为正 | rad |
| `v` | 初始速度 | m/s |
| `a` | 初始加速度 | m/s² |
| `state` | 初始运行状态，当前只接收 `CRUISING` | - |

## `obstacles`

无障碍场景必须保留显式空列表：

```yaml
obstacles: []
```

静态障碍物使用矩形包围盒表示：

```yaml
obstacles:
  - id: 1
    x: 20.0
    y: 0.0
    heading: 0.0
    length: 4.5
    width: 2.0
```

| Field | Meaning | Constraint |
|---|---|---|
| `id` | 场景内的障碍物编号 | 非负且唯一 |
| `x`, `y` | 矩形中心位置 | 有限数值，单位 m |
| `heading` | 局部 x 轴相对全局 x 轴的朝向 | 有限数值，单位 rad，逆时针为正 |
| `length` | 矩形局部 x 方向长度 | 有限且大于 0，单位 m |
| `width` | 矩形局部 y 方向宽度 | 有限且大于 0，单位 m |

障碍物使用 `route.frame_id`，不为每个障碍物重复保存坐标系。v0.2 全部视为静态障碍物，
因此不保存 `type`、`vx` 或 `vy`。

## `expected`

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

| Field | Meaning |
|---|---|
| `success` | 是否期望场景正常运行或规划成功 |
| `collision_free` | 是否要求无碰撞 |
| `reach_goal` | 是否要求到达路线终点 |
| `max_abs_l` | 最大允许横向偏移，单位 m |
| `max_acc` | 最大允许加速度，单位 m/s² |
| `max_decel` | 最大允许减速度绝对值，单位 m/s² |
| `timeout_sec` | 最大运行时间，单位 s |

这些字段目前仍是人工检查目标，还没有接入自动指标判定。

## 设计边界

当前 schema 不包含：

- 动态障碍物速度和预测轨迹。
- 车道、道路边界和交通规则。
- 信号灯和停止线。
- 场景重置或批量执行配置。

一个有效场景至少需要明确输入、预期行为和检查条件。只保存若干路径点而没有预期结果的
文件属于 mock 数据，不作为场景验证用例。
