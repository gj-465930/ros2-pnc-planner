#include "pnc_planner/visualizer.hpp"

#include "tf2/LinearMath/Quaternion.hpp"

#include <utility>

namespace pnc_planner
{
Visualizer::Visualizer(rclcpp::Node & node) : node_(node)
{
  rclcpp::QoS qos(10);
  qos.transient_local();
  marker_pub_ =
    node_.create_publisher<visualization_msgs::msg::Marker>("reference_line_marker", qos);
  path_pub_ = node_.create_publisher<nav_msgs::msg::Path>("reference_line_path", qos);
  traj_pub_ = node_.create_publisher<nav_msgs::msg::Path>("trajectory_path", 10);
  candidate_marker_pub_ =
    node_.create_publisher<visualization_msgs::msg::MarkerArray>("lattice_candidate_markers", 10);
  obstacle_marker_pub_ =
    node_.create_publisher<visualization_msgs::msg::MarkerArray>("static_obstacle_markers", qos);
}

// 传入point消息然后用mark画参考线
void Visualizer::publishReferenceLineMarker(
  const std::vector<geometry_msgs::msg::Point> & points) const
{
  if (points.empty()) {
    RCLCPP_WARN(node_.get_logger(), "there is no points!");
    return;
  }

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = node_.now();

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

  marker_pub_->publish(marker);
  RCLCPP_INFO(node_.get_logger(), "reference line published");
}

// 传入Path消息画参考线
void Visualizer::publishReferenceLine(const nav_msgs::msg::Path & path) const
{
  if (path_pub_ != nullptr) {
    path_pub_->publish(path);
  }
}

void Visualizer::publishTrajectory(const nav_msgs::msg::Path & path) const
{
  if (traj_pub_ != nullptr) {
    traj_pub_->publish(path);
  }
}

void Visualizer::publishCandidateTrajectories(const std::vector<Trajectory> & trajectories) const
{
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.reserve(trajectories.size() + 1);

  visualization_msgs::msg::Marker delete_all_marker;
  delete_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_all_marker);

  const auto stamp = node_.now();
  for (std::size_t index = 0; index < trajectories.size(); ++index) {
    const auto & trajectory = trajectories[index];
    if (trajectory.empty()) {
      continue;
    }

    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = stamp;

    marker.ns = "candidate_trajectories";
    marker.id = static_cast<int>(index);

    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.04;

    marker.color.r = 0.25F;
    marker.color.g = 0.65F;
    marker.color.b = 1.0F;
    marker.color.a = 0.35F;

    marker.points.reserve(trajectory.size());

    for (const auto & trajectory_point : trajectory) {
      geometry_msgs::msg::Point point;
      point.x = trajectory_point.x;
      point.y = trajectory_point.y;
      point.z = 0.05;

      marker.points.push_back(point);
    }

    marker_array.markers.push_back(std::move(marker));
  }

  candidate_marker_pub_->publish(marker_array);
}

void Visualizer::publishStaticObstacles(
  const pnc_planner::msg::ObstacleArray & obstacles_array) const
{
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.reserve(obstacles_array.obstacles.size() + 1);

  visualization_msgs::msg::Marker delete_all_marker;
  delete_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_all_marker);

  const auto stamp = node_.now();
  constexpr double marker_height = 1.5;

  for (const auto & obstacle : obstacles_array.obstacles) {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = obstacles_array.header.frame_id;
    marker.header.stamp = stamp;

    marker.ns = "static_obstacles";
    marker.id = obstacle.id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = obstacle.x;
    marker.pose.position.y = obstacle.y;
    marker.pose.position.z = marker_height * 0.5;

    tf2::Quaternion qtn;
    qtn.setRPY(0.0, 0.0, obstacle.heading);

    marker.pose.orientation.x = qtn.x();
    marker.pose.orientation.y = qtn.y();
    marker.pose.orientation.z = qtn.z();
    marker.pose.orientation.w = qtn.w();

    marker.scale.x = obstacle.length;
    marker.scale.y = obstacle.width;
    marker.scale.z = marker_height;

    marker.color.r = 0.9F;
    marker.color.g = 0.2F;
    marker.color.b = 0.1F;
    marker.color.a = 0.85F;

    marker_array.markers.push_back(marker);
  }

  obstacle_marker_pub_->publish(marker_array);
}

}  // namespace pnc_planner
