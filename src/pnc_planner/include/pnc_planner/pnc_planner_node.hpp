#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "pnc_planner/ego_vehicle.hpp"
#include "pnc_planner/reference_line.hpp"
#include "pnc_planner/visualizer.hpp"

namespace pnc_planner {

class PncPlannerNode : public rclcpp::Node {
public:
  explicit PncPlannerNode(const std::string &node_name = "pnc_planner_node");

  ~PncPlannerNode() override = default;

private:
  void timerCallback();
  void testSinPathVisual();
  void testEgoVehicle();
  void
  updateReferenceLine(const std::vector<geometry_msgs::msg::Point> &points);
  void publishReferenceLine();

  std::shared_ptr<EgoVehicle> ego_vehicle_;
  std::shared_ptr<Visualizer> visualizer_;
  std::shared_ptr<ReferenceLine> ref_line_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace pnc_planner