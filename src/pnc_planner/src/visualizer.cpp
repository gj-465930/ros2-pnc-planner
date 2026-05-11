#include "pnc_planner/visualizer.hpp"

namespace pnc_planner {
Visualizer::Visualizer(rclcpp::Node::SharedPtr node) : node_(node) {
  rclcpp::QoS qos(10);
  qos.transient_local();
  pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "visualization_marker", qos);
}

void Visualizer::publishReferenceLine(
    const std::vector<geometry_msgs::msg::Point> &points) {
  if (points.empty()) {
    RCLCPP_WARN(node_->get_logger(), "there is no points!");
    return;
  }

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = node_->now();

  marker.ns = "reference_line";
  marker.id = 0;

  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.orientation.w = 1.0;

  // 线宽0.1m 绿线
  marker.scale.x = 0.1;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.points = points;

  pub_->publish(marker);
  RCLCPP_INFO(node_->get_logger(), "reference line published");
}

} // namespace pnc_planner
