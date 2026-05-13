#include "pnc_planner/ego_vehicle.hpp"
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

  // 实例化车辆
  pnc_planner::EgoVehicle ego(node);
  double dt = 0.02;
  ego.setPose(0.0, 0.0, 0.0);
  ego.setCommand(0.7, 0.0);
  auto timer = node->create_wall_timer(
      std::chrono::milliseconds(20), [&ego, dt, node]() {
        ego.updateState(dt);
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                             "车辆状态更新中...");
      });

  rclcpp::spin(node);
  rclcpp::shutdown();
}