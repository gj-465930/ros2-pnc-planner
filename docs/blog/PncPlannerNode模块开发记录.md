# PncPlannerNode 模块开发记录：把散装的零件装进一个壳

## 一、为什么加这个模块

一开始我的 main.cpp 长这样：

```cpp
int main() {
    auto node = make_shared<Node>("pnc_node");
    Visualizer vis(node);
    EgoVehicle ego(node);
    ReferenceLine ref;

    // 在 main 里手写测试数据...
    // 在 main 里手写定时器...
    // 在 main 里手写所有逻辑...

    rclcpp::spin(node);
}
```

所有东西散在 main 函数里——创建组件、造测试数据、写定时回调、发布消息——全搅在一起。模块还没几个就已经很乱了，后面再加规划器、加订阅、加参数，main 会变成灾难。

于是我把这些散装零件装进了一个类 `PncPlannerNode`：

```cpp
int main() {
    rclcpp::init(argc, argv);
    auto node = make_shared<PncPlannerNode>("pnc_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
}
```

main 只剩三行。所有 ROS 相关的装配——组件初始化、定时器、回调——都封装在 `PncPlannerNode` 内部。我自己总结的好处有三条：

1. **main 只负责启动**，不关心内部怎么接线
2. **测试的时候不需要跑完整系统**，直接构造 `PncPlannerNode` 就行
3. **以后加模块**（规划器、订阅方）只在类里加成员和方法，不用碰 main

---

## 二、架构调整：shared_ptr 换成裸指针

做完封装之后，我还改了一个起初没意识到的问题。

### 2.1 原来的写法

`EgoVehicle` 和 `Visualizer` 的构造函数接收 `rclcpp::Node::SharedPtr`：

```cpp
EgoVehicle(rclcpp::Node::SharedPtr node);
Visualizer(rclcpp::Node::SharedPtr node);
```

然后在 `PncPlannerNode` 构造里传 `this`：

```cpp
ego_vehicle_ = std::make_shared<EgoVehicle>(this);
visualizer_  = std::make_shared<Visualizer>(this);
```

看起来能编译，但实际上这里有个所有权的问题。

### 2.2 问题在哪

`PncPlannerNode` 的生命周期由 `main` 里的 `make_shared<PncPlannerNode>` 管理。如果把 `this` 裸指针传给 `shared_ptr<Node>` 构造函数，就会在 `EgoVehicle` 和 `Visualizer` 内部分别创建**新的 shared_ptr 控制块**。

同一块内存被三个不同的 shared_ptr 控制块各自管理——每个都认为自己持有所有权，析构的时候会各自 delete 一次。这就是 double free。

技术上说，在构造函数里不能安全地调用 `shared_from_this()`，因为外层的 shared_ptr 还没构造完。所以真要传 shared_ptr，就得把组件的创建推迟到工厂函数或者 init 方法里——但这就把简单问题复杂化了。

### 2.3 改法

我的思路是：既然 `EgoVehicle` 和 `Visualizer` 本身就不应该持有 Node 的所有权，为什么要让它俩拿 shared_ptr？

```cpp
// 改前
explicit EgoVehicle(rclcpp::Node::SharedPtr node);   // 构造：接 shared_ptr
rclcpp::Node::SharedPtr node_;                        // 成员：存 shared_ptr

// 改后
explicit EgoVehicle(rclcpp::Node *node);              // 构造：接裸指针
rclcpp::Node *node_;                                  // 成员：存裸指针
```

`EgoVehicle` 和 `Visualizer` 是 `PncPlannerNode` 的成员变量，生命期严格小于 Node 本身——Node 析构前，这两个子模块一定先析构。裸指针不会悬空。

这样一来主次关系就非常清晰了：

```
PncPlannerNode (持有所有权)
  ├── EgoVehicle   (引用，不持有)
  ├── Visualizer   (引用，不持有)
  └── ReferenceLine(独立对象)
```

只有 Node 持有所有权，子模块只管用，不管生命周期。这是典型的 **依赖注入 + 非拥有引用** 模式。

---

## 三、总结

这次做的事情其实就两件：

1. **加壳** — 把散在 main 里的组件装配逻辑收到 `PncPlannerNode` 里，main 只负责启动
2. **理权** — `shared_ptr` 改裸指针，所有权归 Node，子模块只是借用

我的感受是：刚开始写 ROS2 的时候，所有教程都教你用 shared_ptr，导致我下意识觉得任何地方都该用。但实际上，只有在真正需要共享所有权的时候才应该用 shared_ptr——内部子模块引用父模块，裸指针就够了。
