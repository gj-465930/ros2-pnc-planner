#pragma once

#include <chrono>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace pnc_planner {

class Visualizer {
public:
  explicit Visualizer(rclcpp::Node::SharedPtr node);

  void
  publishReferenceLine(const std::vector<geometry_msgs::msg::Point> &points);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
};

} // namespace pnc_planner