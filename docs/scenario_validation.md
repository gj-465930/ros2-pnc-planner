# Scenario Validation

本文档记录 Mini PNC Lab 第一版 YAML 场景验证结果。当前验证方式是 **人工/RViz 复现验证**：通过 `scenario_publisher` 发布场景路线到 `/routing_path`，由 `PncPlannerNode` 生成参考线、规划轨迹并在 RViz 中观察结果。

当前阶段还没有自动 metrics runner，因此 `expected` 字段中的 `collision_free`、`max_abs_l`、`max_acc` 等指标主要作为人工验收目标和后续自动评测的接口预留。

## 验证范围

第一批基础场景均为无障碍路线场景：

```text
straight_cruise.yaml
curve_cruise.yaml
end_of_route.yaml
```

这些场景验证的是：

```text
ScenarioLoader 能解析 YAML
ScenarioPublisher 能发布 /routing_path
PncPlannerNode 能接收外部 route 并更新 ReferenceLine
LatticePlanner 能在基础路线场景中输出轨迹
RViz 能显示参考线、规划轨迹和车辆模型
```

## 运行方式

从工作空间根目录构建并加载环境：

```bash
colcon build --packages-select pnc_planner
source install/setup.bash
```

终端 1：启动 planner、robot_state_publisher 和 RViz：

```bash
ros2 launch pnc_planner pnc_planner.launch.py
```

终端 2：发布一个场景路线：

```bash
ros2 run pnc_planner scenario_publisher --ros-args \
  -p scenario_file:=src/pnc_planner/scenarios/straight_cruise.yaml
```

替换 `scenario_file` 参数即可运行其他场景：

```bash
ros2 run pnc_planner scenario_publisher --ros-args \
  -p scenario_file:=src/pnc_planner/scenarios/curve_cruise.yaml

ros2 run pnc_planner scenario_publisher --ros-args \
  -p scenario_file:=src/pnc_planner/scenarios/end_of_route.yaml
```

## 当前结果

| Scenario | Purpose | Expected Behavior | Status |
|---|---|---|---|
| `straight_cruise` | 直线巡航 | 参考线沿 x 轴生成，规划轨迹稳定跟随直线 | Pass |
| `curve_cruise` | 缓弯巡航 | 参考线随 YAML 路线形成缓弯，规划轨迹能够沿曲线路线生成 | Pass |
| `end_of_route` | 接近终点路线 | 发布较短路线后，系统不崩溃，规划轨迹在路线末端附近保持合理 | Pass |

## 场景说明

### `straight_cruise`

文件：

```text
src/pnc_planner/scenarios/straight_cruise.yaml
```

验证目标：

```text
最基础的 route → ReferenceLine → LatticePlanner → RViz 闭环
```

当前结果：

```text
ScenarioPublisher 成功发布 /routing_path
PncPlannerNode 成功初始化参考线
RViz 中可以观察到直线参考线和规划轨迹
```

### `curve_cruise`

文件：

```text
src/pnc_planner/scenarios/curve_cruise.yaml
```

验证目标：

```text
验证非直线路线下参考线生成和局部规划输出是否正常
```

当前结果：

```text
ScenarioPublisher 成功发布曲线路线
RViz 中可以观察到缓弯参考线和规划轨迹
```

### `end_of_route`

文件：

```text
src/pnc_planner/scenarios/end_of_route.yaml
```

验证目标：

```text
验证较短路线和路线终点附近边界条件下，系统不崩溃，并能保持基础规划输出
```

当前结果：

```text
ScenarioPublisher 成功发布短路线
PncPlannerNode 能处理该路线并输出可视化结果
```

## 当前局限

当前场景验证仍是第一版，主要局限包括：

```text
ego 初始状态已写入 YAML，但尚未注入 PncPlannerNode
obstacles 当前只支持空列表
expected 指标尚未自动采集或自动判定
还没有批量 scenario runner
还没有保存 RViz 截图/GIF 作为正式结果附件
```

这些限制是后续 Phase 4 及评测框架工作的入口，而不是当前场景链路失败。

## 后续计划

下一步可以按以下顺序扩展：

```text
1. 将 ego 初始状态接入 PncPlannerNode 或场景执行流程
2. 增加基础 metrics 记录，例如 lateral error、planning success、planning time
3. 增加批量运行脚本或 runner
4. 打通静态障碍物输入和 RViz 障碍物显示
5. 将 expected 字段升级为自动判定依据
```
