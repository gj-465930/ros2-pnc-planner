#include "pnc_planner/scenario/scenario_loader.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include <chrono>
#include <exception>
#include <memory>
#include <string>

namespace pnc_planner::scenario
{
namespace
{
class ScenarioPublisher : public rclcpp::Node
{
public:
  ScenarioPublisher() : Node("scenario_publisher")
  {
    const std::string scenario_file_path =
      this->declare_parameter<std::string>("scenario_file", "");

    scenario_data_ = ScenarioLoader::LoadFromFile(scenario_file_path);

    route_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/routing_path", 10);

    publisher_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200), [this]() { this->TryPublishRoute(); });

    RCLCPP_INFO(
      this->get_logger(), "Loaded scenario '%s' from '%s' with %zu route points",
      scenario_data_.name.c_str(), scenario_file_path.c_str(), scenario_data_.route.points.size());
  }

private:
  void TryPublishRoute();

  ScenarioData scenario_data_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr route_publisher_;
  rclcpp::TimerBase::SharedPtr publisher_timer_;
  bool route_published_{false};
};
}  // namespace
void ScenarioPublisher::TryPublishRoute()
{
  if (route_published_) {
    return;
  }

  if (route_publisher_->get_subscription_count() == 0U) {
    return;
  }

  nav_msgs::msg::Path route_path;
  route_path.header.stamp = this->now();
  route_path.header.frame_id = scenario_data_.route.frame_id;

  route_path.poses.reserve(scenario_data_.route.points.size());

  for (const auto & route_point : scenario_data_.route.points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = route_path.header;

    pose.pose.position.x = route_point[0];
    pose.pose.position.y = route_point[1];
    pose.pose.position.z = 0.0;

    pose.pose.orientation.w = 1.0;

    route_path.poses.push_back(pose);
  }

  route_publisher_->publish(route_path);
  route_published_ = true;
  publisher_timer_->cancel();

  RCLCPP_INFO(
    this->get_logger(), "Published scenario '%s' to /routing_path", scenario_data_.name.c_str());
}

}  // namespace pnc_planner::scenario

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    const auto node = std::make_shared<pnc_planner::scenario::ScenarioPublisher>();

    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("scenario_publisher"), "%s", e.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
