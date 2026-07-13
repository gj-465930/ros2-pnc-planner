#pragma once

#include <string>

#include "pnc_planner/scenario/scenario_data.hpp"

namespace pnc_planner::scenario {

  class ScenarioLoader {
    public:
      static ScenarioData LoadFromFile(const std::string& scenario_file_path);
  };

}  // namespace pnc_planner::scenario
