#pragma once

#include <array>
#include <string>
#include <vector>

namespace pnc_planner::scenario
{

struct Route
{
  std::string frame_id;
  std::vector<std::array<double, 2>> points;
};

struct EgoInitialState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v{0.0};
  double a{0.0};
  std::string state;
};

struct ScenarioData
{
  std::string schema_version;
  std::string name;
  Route route;
  EgoInitialState ego;
};

}  // namespace pnc_planner::scenario
