#pragma once

#include <chrono>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pnc_planner/common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace pnc_planner {

class Visualizer {
public:
  explicit Visualizer(rclcpp::Node *node);

  // 用mark消息画参考线
  void publishReferenceLineMarker(
      const std::vector<geometry_msgs::msg::Point> &points);

  // 用path消息画参考线
  void publishReferenceLine(const nav_msgs::msg::Path &path);

  void publishTrajectory(const nav_msgs::msg::Path &path);

private:
  rclcpp::Node *node_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
};

} // namespace pnc_planner