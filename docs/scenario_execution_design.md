# Scenario Execution Design

场景系统负责把 YAML 中的测试输入稳定地送入规划节点。解析、ROS2 通信、规划和指标检查
保持分离，避免 planner 直接读取 YAML 或依赖 RViz 消息。

## 组件职责

```text
ScenarioLoader
  读取和校验 YAML，返回纯 C++ ScenarioData

scenario_publisher
  将 ScenarioData 转换为 ROS2 场景输入消息

PncPlannerNode
  接收并锁定一组场景输入，输入完整后开始规划

Visualizer
  显示参考线、轨迹、自车和障碍物，不作为算法输入

LatticePlanner
  使用参考线、自车状态和障碍物生成局部轨迹
```

## 当前执行链路

当前已经接入 route 和 ego 初始状态：

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

`ScenarioLoader` 不发布 topic，`scenario_publisher` 不生成参考线，也不直接调用 planner。

## 静态障碍物扩展

静态障碍物链路使用第三个场景输入 topic：

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
              ├── Visualizer
              └── LatticePlanner
```

### 消息定义

规划输入使用独立的 obstacle message，不使用 RViz Marker 代替：

```text
# Obstacle.msg
int32 id
float64 x
float64 y
float64 heading
float64 length
float64 width
```

```text
# ObstacleArray.msg
std_msgs/Header header
pnc_planner/Obstacle[] obstacles
```

`ObstacleArray.header.frame_id` 当前固定为 `map`，时间戳与同一次场景发布的 route 和 ego
消息一致。

### Topic 与 QoS

三个场景输入使用相同的 QoS：

```text
KeepLast(1)
reliable
transient_local
```

对应 topic：

```text
/routing_path
/scenario/initial_state
/scenario/obstacles
```

transient-local 使后启动的 planner 也能收到场景发布器保留的最后一条输入。

### 空障碍物列表

无障碍场景也必须发布一次空的 `ObstacleArray`：

```text
收到空数组    障碍物输入已经就绪，当前场景明确没有障碍物
没有收到消息  障碍物输入尚未就绪
```

不能用“没有收到消息”表示“没有障碍物”，否则节点无法判断应该开始规划还是继续等待输入。

## 场景就绪条件

静态障碍物链路接入后，场景模式的规划门控为：

```text
route_ready_
&& initial_state_ready_
&& obstacles_ready_
```

三类输入全部成功接收后才开始规划。这样可以避免节点先按无障碍环境规划，随后才收到障碍
物列表。

当前源码已经实现 route 和 ego 的门控；`obstacles_ready_` 及其回调仍需在后续实现中
接入。

## 输入校验

### Route

- 至少包含 3 个路径点。
- `frame_id` 为 `map`。
- 每个坐标都是有限数值。
- `ReferenceLine` 初始化成功后才能进入 ready。

### Ego initial state

- 位置、航向角、速度和加速度都是有限数值。
- 速度非负。
- 当前只接收 `CRUISING`。
- 校验成功后才能进入 ready。

### Obstacles

- 数组可以为空。
- `header.frame_id` 为 `map`。
- `id` 非负且在同一列表中唯一。
- 位置、朝向和尺寸都是有限数值。
- `length` 和 `width` 大于 0。
- 整个列表校验成功后才能进入 ready。

非法输入不会锁定对应状态，后续合法输入仍可重试。

## 单场景生命周期

一个 `PncPlannerNode` 进程只运行一个完整场景：

```text
等待输入
    ↓
分别校验 route、ego 和 obstacles
    ↓
三类输入全部就绪
    ↓
开始规划并锁定场景输入
    ↓
拒绝后续重复输入
```

具体规则：

- 第一份合法 route 初始化并锁定参考线。
- 第一份合法 ego 设置并锁定自车初始状态。
- 第一份合法 obstacle list 保存并锁定，空列表也是合法输入。
- 重复输入不会替换运行中的场景数据。
- 切换场景需要重启 planner。

route、ego 和 obstacles 通过独立 topic 发布，当前没有提供跨 topic 的原子事务。运行时要求
每次在新的 planner 进程中只启动一个 `scenario_publisher`。无重启切换需要完整的 reset 和
场景关联协议，暂不在当前范围内实现。

## Mock 模式

`use_mock_routing=true` 时，节点内部完成全部输入初始化：

```text
mock route ready
mock ego ready
empty obstacle list ready
```

mock 模式不等待场景 topic，并拒绝外部 route、ego 和 obstacle 输入。mock 模式与 YAML
场景模式互斥。

## Schema 兼容

```text
v0.1  只允许空 obstacles，继续支持现有三个基础场景
v0.2  允许空列表或静态矩形障碍物
```

`scenario_publisher` 对两个版本都发布 `/scenario/obstacles`。v0.1 场景发布显式空数组，
因此 planner 不需要根据 schema 版本区分 readiness 行为。

## 暂不实现

- 动态障碍物速度和轨迹预测。
- ST 图和完整速度规划。
- Behavior Planner。
- 自动 expected metrics。
- 无重启场景切换和 batch runner。
- 生产级 OBB/SAT 碰撞检测。

静态障碍物接入按解析、发布接收、RViz 可视化、planner 碰撞过滤和场景验证逐步完成，避免
在同一次修改中同时改变数据协议和规划算法。
