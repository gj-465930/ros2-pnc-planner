#pragma once

#include "pnc_planner/controller/longitudinal_controller_base.hpp"
#include "pnc_planner/controller/pure_pursuit_controller.hpp"
#include "pnc_planner/ego_vehicle.hpp"
#include "pnc_planner/lattice_planner.hpp"
#include "pnc_planner/msg/scenario_initial_state.hpp"
#include "pnc_planner/reference_line.hpp"
#include "pnc_planner/visualizer.hpp"
#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/path.hpp"

#include <memory>
#include <string>

namespace pnc_planner
{

class PncPlannerNode : public rclcpp::Node
{
public:
  explicit PncPlannerNode(const std::string & node_name = "pnc_planner_node");

  ~PncPlannerNode() override = default;

private:
  void timerCallback();
  bool updateReferenceLine(const std::vector<geometry_msgs::msg::Point> & points);
  void publishReferenceLine();
  void globalRouteCallback(const nav_msgs::msg::Path::ConstSharedPtr & msg);
  void publishTrajectory(const Trajectory & traj);
  void trackTrajectory(const double dt);
  void initialStateCallback(const pnc_planner::msg::ScenarioInitialState::ConstSharedPtr & msg);

  Trajectory planned_traj_;
  std::shared_ptr<EgoVehicle> ego_vehicle_;
  std::shared_ptr<Visualizer> visualizer_;
  std::shared_ptr<ReferenceLine> ref_line_;
  std::shared_ptr<LatticePlanner> lattice_planner_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_route_sub_;
  rclcpp::Subscription<pnc_planner::msg::ScenarioInitialState>::SharedPtr initial_state_sub_;
  std::unique_ptr<controller::LateralControllerBase> lateral_ctrl_;
  std::unique_ptr<controller::LongitudinalControllerBase> longitudinal_controller_;

  bool route_ready_{false};
  bool initial_state_ready_{false};
};

}  // namespace pnc_planner
