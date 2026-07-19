#include "pnc_planner/pnc_planner_node.hpp"

#include <cmath>
#include <vector>

int main(const int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<pnc_planner::PncPlannerNode>("pnc_node");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
