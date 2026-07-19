#include "pnc_planner/msg/scenario_initial_state.hpp"
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
    rclcpp::QoS scenario_qos(rclcpp::KeepLast(1));
    scenario_qos.reliable();
    scenario_qos.transient_local();

    const std::string scenario_file_path =
      this->declare_parameter<std::string>("scenario_file", "");

    scenario_data_ = ScenarioLoader::LoadFromFile(scenario_file_path);

    route_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/routing_path", scenario_qos);

    initial_state_publisher_ = this->create_publisher<pnc_planner::msg::ScenarioInitialState>(
      "/scenario/initial_state", scenario_qos);

    publisher_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200), [this]() { this->TryPublishScenario(); });

    RCLCPP_INFO(
      this->get_logger(), "Loaded scenario '%s' from '%s' with %zu route points",
      scenario_data_.name.c_str(), scenario_file_path.c_str(), scenario_data_.route.points.size());
  }

private:
  void TryPublishScenario();

  ScenarioData scenario_data_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr route_publisher_;
  rclcpp::Publisher<pnc_planner::msg::ScenarioInitialState>::SharedPtr initial_state_publisher_;
  rclcpp::TimerBase::SharedPtr publisher_timer_;
  bool scenario_published_{false};
};
}  // namespace
void ScenarioPublisher::TryPublishScenario()
{
  if (scenario_published_) {
    return;
  }

  const auto stamp = this->now();

  nav_msgs::msg::Path route_path;
  route_path.header.stamp = stamp;
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

  pnc_planner::msg::ScenarioInitialState initial_state;
  initial_state.header.stamp = stamp;
  initial_state.header.frame_id = scenario_data_.route.frame_id;
  initial_state.x = scenario_data_.ego.x;
  initial_state.y = scenario_data_.ego.y;
  initial_state.velocity = scenario_data_.ego.v;
  initial_state.acceleration = scenario_data_.ego.a;
  initial_state.yaw = scenario_data_.ego.yaw;
  initial_state.state = scenario_data_.ego.state;

  route_publisher_->publish(route_path);
  initial_state_publisher_->publish(initial_state);

  scenario_published_ = true;
  publisher_timer_->cancel();

  RCLCPP_INFO(
    this->get_logger(), "Published scenario '%s' to /routing_path and /scenario/initial_state",
    scenario_data_.name.c_str());
}

}  // namespace pnc_planner::scenario

int main(const int argc, char ** argv)
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
