#pragma once

#include <array>
#include <string>
#include <vector>

namespace pnc_planner::scenario {

  struct ScenarioData {
    std::string schema_version;
    std::string name;
    std::string frame_id;
    std::vector<std::array<double, 2>> route_points;
  };

}  // namespace pnc_planner::scenario
