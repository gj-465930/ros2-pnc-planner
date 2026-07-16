#include "pnc_planner/scenario/scenario_loader.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace pnc_planner::scenario
{

static std::string CreateTemporaryScenario(
  const std::string & file_name, const std::string & ego_yaml)
{
  const std::filesystem::path file_path = std::filesystem::temp_directory_path() / file_name;

  std::ofstream output(file_path);

  if (!output.is_open()) {
    throw std::runtime_error("Failed to create temporary scenario file");
  }

  output << R"(schema_version: "0.1"
name: invalid_ego
route:
  frame_id: "map"
  points:
    - [0.0, 0.0]
    - [10.0, 0.0]
    - [20.0, 0.0]
ego:
)" << ego_yaml
         << R"(
obstacles: []
expected:
  success: true
)";
  output.close();
  return file_path.string();
}

static std::string GetLoadErrorMessage(const std::string & scenario_file)
{
  try {
    ScenarioLoader::LoadFromFile(scenario_file);
  } catch (const std::runtime_error & exception) {
    return exception.what();
  }

  return "";
}

namespace
{
TEST(ScenarioLoaderTest, ParsesEndOfRouteEgoInitialState)
{
  const std::string scenario_file =
    std::string(PNC_PLANNER_SOURCE_DIR) + "/scenarios/end_of_route.yaml";

  const auto scenario = ScenarioLoader::LoadFromFile(scenario_file);

  EXPECT_EQ(scenario.schema_version, "0.1");
  EXPECT_EQ(scenario.name, "end_of_route");

  EXPECT_EQ(scenario.route.frame_id, "map");
  EXPECT_EQ(scenario.route.points.size(), 3U);

  EXPECT_DOUBLE_EQ(scenario.ego.x, 16.0);
  EXPECT_DOUBLE_EQ(scenario.ego.y, 0.0);
  EXPECT_DOUBLE_EQ(scenario.ego.yaw, 0.0);
  EXPECT_DOUBLE_EQ(scenario.ego.v, 3.0);
  EXPECT_DOUBLE_EQ(scenario.ego.a, 0.0);
  EXPECT_EQ(scenario.ego.state, "CRUISING");
}

TEST(ScenarioLoaderTest, RejectsNegativeEgoVelocity)
{
  const std::string scenario_file = CreateTemporaryScenario(
    "pnc_negative_ego_velocity.yaml",
    R"(  x: 0.0
  y: 0.0
  yaw: 0.0
  v: -1.0
  a: 0.0
  state: CRUISING
)");

  std::string error_message = GetLoadErrorMessage(scenario_file);
  std::filesystem::remove(scenario_file);
  EXPECT_NE(error_message.find("ego.v must be non-negative"), std::string::npos);
}

TEST(ScenarioLoaderTest, RejectsMissingEgoField)
{
  const std::string scenario_file = CreateTemporaryScenario(
    "pnc_missing_ego_field.yaml",
    R"(  x: 0.0
  y: 0.0
  yaw: 0.0
  v: 1.0
  state: CRUISING
)");

  std::string error_message = GetLoadErrorMessage(scenario_file);
  std::filesystem::remove(scenario_file);
  EXPECT_NE(error_message.find("Missing or invalid 'ego.a'"), std::string::npos);
}

TEST(ScenarioLoaderTest, RejectsNonNumericEgoValue)
{
  const std::string scenario_file = CreateTemporaryScenario(
    "pnc_non_numeric_ego.yaml",
    R"(  x: 0.0
  y: 0.0
  yaw: 0.0
  v: fast
  a: 0.0
  state: CRUISING
)");

  std::string error_message = GetLoadErrorMessage(scenario_file);
  std::filesystem::remove(scenario_file);
  EXPECT_NE(error_message.find("Failed to parse scenario file"), std::string::npos);
}

TEST(ScenarioLoaderTest, RejectsNonFiniteEgoValue)
{
  const std::string scenario_file = CreateTemporaryScenario(
    "pnc_non_finite_ego_value.yaml",
    R"(  x: .nan
  y: 0.0
  yaw: 0.0
  v: 1.0
  a: 0.0
  state: CRUISING
)");

  const std::string error_message = GetLoadErrorMessage(scenario_file);
  std::filesystem::remove(scenario_file);
  EXPECT_NE(error_message.find("ego.x must be finite"), std::string::npos);
}

TEST(ScenarioLoaderTest, RejectsInfiniteEgoValue)
{
  const std::string scenario_file = CreateTemporaryScenario(
    "pnc_infinite_ego_value.yaml",
    R"(  x: .inf
  y: 0.0
  yaw: 0.0
  v: 1.0
  a: 0.0
  state: CRUISING
)");

  const std::string error_message = GetLoadErrorMessage(scenario_file);
  std::filesystem::remove(scenario_file);
  EXPECT_NE(error_message.find("ego.x must be finite"), std::string::npos);
}

TEST(ScenarioLoaderTest, RejectsUnsupportedEgoState)
{
  const std::string scenario_file = CreateTemporaryScenario(
    "pnc_unsupported_ego_state.yaml",
    R"(  x: 0.0
  y: 0.0
  yaw: 0.0
  v: 1.0
  a: 0.0
  state: EMERGENCY
)");

  const std::string error_message = GetLoadErrorMessage(scenario_file);
  std::filesystem::remove(scenario_file);
  EXPECT_NE(error_message.find("Unsupported ego.state: EMERGENCY"), std::string::npos);
}

}  // namespace
}  // namespace pnc_planner::scenario
