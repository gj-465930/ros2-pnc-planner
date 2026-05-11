#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node {
public:
  MyNode() : Node("node_name") {
    RCLCPP_INFO(this->get_logger(), "节点 %s 已启动！", "node_name");
  }
};
