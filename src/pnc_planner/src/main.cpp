#include "pnc_planner/visualizer.hpp"
#include <cmath>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pnc_node");

  // 画图
  pnc_planner::Visualizer vis(node);
  std::vector<geometry_msgs::msg::Point> reference_points;

  for (double x = 0.0; x <= 20.0; x += 0.2) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = std::sin(x);
    p.z = 0.0;
    reference_points.push_back(p);
  }
  vis.publishReferenceLine(reference_points);

  rclcpp::spin(node);
  rclcpp::shutdown();
}