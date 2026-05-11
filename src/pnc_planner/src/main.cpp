#include "pnc_planner/visualizer.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pnc_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
}