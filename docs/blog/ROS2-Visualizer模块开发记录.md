# ROS2 + EM Planner 开发记录（一）：Visualizer 模块

> 项目仓库：[github.com/gj-465930/ros2-pnc-planner](https://github.com/gj-465930/ros2-pnc-planner)

---

## 1. 模块流程

整个 Visualizer 模块的工作流程很简单，三行就能概括：

```
main.cpp 生成路径点 → Visualizer 封装 Marker → 发布到 /visualization_marker → RViz2 渲染
```

具体来看文件结构：

```
pnc_planner/
├── include/pnc_planner/visualizer.hpp   # 声明接口
├── src/visualizer.cpp                   # 封装 Marker，发布话题
└── src/main.cpp                         # 入口，构造路径点，调用 Visualizer
```

**为什么把 Visualizer 单独抽出来，而不是全写在 main.cpp 里？**

因为后面 EM Planner 的每个模块（参考线生成、DP 规划、QP 平滑）都需要可视化调试。如果每个模块都在自己的代码里硬写 Marker 发布逻辑，会有一堆重复代码。抽成独立的 Visualizer 类，谁想画东西就调一下 `publishReferenceLine`，干净。

**Visualizer 不继承 rclcpp::Node**

一个程序只需要一个 ROS 节点。Visualizer 拿着主节点的指针干活，不自己建新节点。后面 FrenetConverter、DpPlanner 也都是这个模式——一个节点，多个模块挂上面。

**当前阶段 main.cpp 做的事：**

```cpp
// 构造一条正弦曲线作为参考线，交给 Visualizer 发布
for (double x = 0.0; x <= 20.0; x += 0.2) {
    p.x = x;
    p.y = std::sin(x);
    reference_points.push_back(p);
}
vis.publishReferenceLine(reference_points);
```

后面把这段替换成 EM Planner 的真实输出，就是规划结果的可视化了。

---

## 2. 为什么不用定时器周期性发布

刚开始我的头文件里加了 `timer_`，打算每 500ms 刷一次 Marker。后来发现完全没必要，删掉了。

原因是 Marker 有一个特性：只要把 `action` 设为 `ADD`，且不发送 `DELETE` 指令，RViz2 就会一直画着这条线。背后的机制是 RViz2 内部维护了一个"黑板"——你把 Marker 写上去，它就存在那里。每刷新一帧屏幕，RViz2 自己去黑板读数据重新渲染，不需要你反复发。

关键代码就这两行：

```cpp
marker.action = visualization_msgs::msg::Marker::ADD;
marker.lifetime = rclcpp::Duration(0, 0);  // 0 表示永不过期，默认值
```

不需要定时器不等于不能用定时器。如果后面路径需要动态更新（比如障碍物移动导致路径变化），就重新调一次 `publishReferenceLine` 覆盖掉旧的。调一次，新线就替换旧线。不需要一个定时器在旁边空转。

省掉的代码：

- 头文件里删了 `rclcpp::TimerBase::SharedPtr timer_` 成员变量
- 构造函数里删了 `create_wall_timer` 调用
- 删了整个 `on_timer()` 回调函数

少了一个定时器，少了一个线程切换，代码更精简。

---

## 3. 解决时序 Bug：RViz2 收不到消息

**症状：** 先启动节点，后启动 RViz2，RViz2 里看不到参考线。必须关了节点重开才行。

**原因：** ROS2 默认的 QoS 策略是 `volatile`——消息发了就发了。如果发布的时候没有订阅者，消息直接丢弃，后来连上来的订阅者什么也收不到。

我的场景就是典型的"先发后收"——main 函数里在初始化完成后立刻发布，但这时候我还没打开 RViz2。等 RViz2 连上来，那条 Marker 早就消失在空气里了。

**解决方案一（最简单的，但不够优雅）：sleep**

```cpp
rclcpp::sleep_for(std::chrono::seconds(2));   // 等 RViz 连上来
vis.publishReferenceLine(reference_points);    // 再发布
```

一行解决问题，但不体面。你等 2 秒可能不够，等 5 秒又浪费时间。

**解决方案二（正确的）：用 transient_local QoS**

```cpp
rclcpp::QoS qos(10);
qos.transient_local();
pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
    "visualization_marker", qos);
```

最终采用这个方案。`transient_local` 的含义是：发布者把最后一条消息保存起来，之后有新的订阅者连上来，自动补发一份。

对比表：

|          | volatile（默认） | transient_local    |
| -------- | ---------------- | ------------------ |
| 先发后收 | 收不到           | 能收到             |
| 性能开销 | 无               | 极小（存一条消息） |
| 适用场景 | 实时流数据       | 配置、参考线、地图 |

改这一行，从根源解决时序问题，不需要 sleep 这种补丁。

---$$

## 本次提交

```bash
git commit -m "feat: 初始化PNC规划器，完成Visualizer模块（RViz参考线绘制，transient_local QoS）"
```

GitHub 链接：[github.com/gj-465930/ros2-pnc-planner](https://github.com/gj-465930/ros2-pnc-planner)
