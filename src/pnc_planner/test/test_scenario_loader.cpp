#include "pnc_planner/scenario/scenario_loader.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace pnc_planner::scenario
{

static std::string CreateTemporaryScenario(
  const std::string & file_name, const std::string & ego_yaml,
  const std::string & schema_version = "0.1", const std::string & obstacles_yaml = "[]")
{
  const std::filesystem::path file_path = std::filesystem::temp_directory_path() / file_name;

  std::ofstream output(file_path);

  if (!output.is_open()) {
    throw std::runtime_error("Failed to create temporary scenario file");
  }

  output << "schema_version: \"" << schema_version << "\"\n"
         << R"(name: temporary_scenario
route:
  frame_id: "map"
  points:
    - [0.0, 0.0]
    - [10.0, 0.0]
    - [20.0, 0.0]
ego:
)" << ego_yaml
         << "\nobstacles: " << obstacles_yaml << "\n"
         << R"(
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

TEST(ScenarioLoaderTest, ParsesEmptyObstacleListInSchemaV02)
{
  const std::string scenario_file = CreateTemporaryScenario(
    "pnc_empty_obstacles.yaml",
    R"(  x: 0.0
  y: 0.0
  yaw: 0.0
  v: 1.0
  a: 0.0
  state: CRUISING
)",
    "0.2", "[]");

  const auto scenario = ScenarioLoader::LoadFromFile(scenario_file);
  std::filesystem::remove(scenario_file);

  EXPECT_EQ(scenario.schema_version, "0.2");
  EXPECT_TRUE(scenario.obstacles.empty());
}

TEST(ScenarioLoaderTest, ParsesSingleStaticObstacle)
{
  const std::string scenario_file = CreateTemporaryScenario(
    "pnc_single_static_obstacle.yaml",
    R"(  x: 0.0
  y: 0.0
  yaw: 0.0
  v: 1.0
  a: 0.0
  state: CRUISING
)",
    "0.2", R"([{id: 7, x: 12.5, y: -1.25, heading: 0.3, length: 4.8, width: 2.0}])");

  const auto scenario = ScenarioLoader::LoadFromFile(scenario_file);
  std::filesystem::remove(scenario_file);

  ASSERT_EQ(scenario.obstacles.size(), 1U);

  const auto & obstacle = scenario.obstacles[0];
  EXPECT_EQ(obstacle.id, 7);
  EXPECT_DOUBLE_EQ(obstacle.x, 12.5);
  EXPECT_DOUBLE_EQ(obstacle.y, -1.25);
  EXPECT_DOUBLE_EQ(obstacle.heading, 0.3);
  EXPECT_DOUBLE_EQ(obstacle.length, 4.8);
  EXPECT_DOUBLE_EQ(obstacle.width, 2.0);
}

TEST(ScenarioLoaderTest, PreservesMultipleObstacleOrderAndValues)
{
  const std::string scenario_file = CreateTemporaryScenario(
    "pnc_multiple_static_obstacles.yaml",
    R"(  x: 0.0
  y: 0.0
  yaw: 0.0
  v: 1.0
  a: 0.0
  state: CRUISING
)",
    "0.2",
    R"([{id: 11, x: 8.0, y: 1.5, heading: 0.1, length: 4.0, width: 1.8}, {id: 3, x: 16.0, y: -2.0, heading: -0.2, length: 5.0, width: 2.2}]
)");

  const auto scenario = ScenarioLoader::LoadFromFile(scenario_file);
  std::filesystem::remove(scenario_file);

  ASSERT_EQ(scenario.obstacles.size(), 2U);

  const auto & first = scenario.obstacles[0];
  EXPECT_EQ(first.id, 11);
  EXPECT_DOUBLE_EQ(first.x, 8.0);
  EXPECT_DOUBLE_EQ(first.y, 1.5);
  EXPECT_DOUBLE_EQ(first.heading, 0.1);
  EXPECT_DOUBLE_EQ(first.length, 4.0);
  EXPECT_DOUBLE_EQ(first.width, 1.8);

  const auto & second = scenario.obstacles[1];
  EXPECT_EQ(second.id, 3);
  EXPECT_DOUBLE_EQ(second.x, 16.0);
  EXPECT_DOUBLE_EQ(second.y, -2.0);
  EXPECT_DOUBLE_EQ(second.heading, -0.2);
  EXPECT_DOUBLE_EQ(second.length, 5.0);
  EXPECT_DOUBLE_EQ(second.width, 2.2);
}

TEST(ScenarioLoaderTest, RejectsNonPositiveObstacleLength)
{
  const std::string scenario_file = CreateTemporaryScenario(
    "pnc_non_positive_obstacle_length.yaml",
    R"(  x: 0.0
  y: 0.0
  yaw: 0.0
  v: 1.0
  a: 0.0
  state: CRUISING
)",
    "0.2", R"([{id: 1, x: 10.0, y: 0.0, heading: 0.0, length: 0.0, width: 2.0}])");

  const std::string error_message = GetLoadErrorMessage(scenario_file);
  std::filesystem::remove(scenario_file);

  EXPECT_NE(
    error_message.find("obstacles[0].length must be finite and greater than 0"), std::string::npos);
}

TEST(ScenarioLoaderTest, RejectsNonPositiveObstacleWidth)
{
  const std::string scenario_file = CreateTemporaryScenario(
    "pnc_non_positive_obstacle_width.yaml",
    R"(  x: 0.0
  y: 0.0
  yaw: 0.0
  v: 1.0
  a: 0.0
  state: CRUISING
)",
    "0.2", R"([{id: 1, x: 10.0, y: 0.0, heading: 0.0, length: 4.0, width: -1.0}])");

  const std::string error_message = GetLoadErrorMessage(scenario_file);
  std::filesystem::remove(scenario_file);

  EXPECT_NE(
    error_message.find("obstacles[0].width must be finite and greater than 0"), std::string::npos);
}

TEST(ScenarioLoaderTest, RejectsDuplicateObstacleId)
{
  const std::string scenario_file = CreateTemporaryScenario(
    "pnc_duplicate_obstacle_id.yaml",
    R"(  x: 0.0
  y: 0.0
  yaw: 0.0
  v: 1.0
  a: 0.0
  state: CRUISING
)",
    "0.2",
    R"([{id: 1, x: 8.0, y: 1.5, heading: 0.1, length: 4.0, width: 1.8}, {id: 1, x: 16.0, y: -2.0, heading: -0.2, length: 5.0, width: 2.2}])");

  const std::string error_message = GetLoadErrorMessage(scenario_file);
  std::filesystem::remove(scenario_file);

  EXPECT_NE(error_message.find("Duplicate obstacle id: 1"), std::string::npos);
}

TEST(ScenarioLoaderTest, RejectsMissingObstacleField)
{
  const std::string scenario_file = CreateTemporaryScenario(
    "pnc_missing_obstacle_heading.yaml",
    R"(  x: 0.0
  y: 0.0
  yaw: 0.0
  v: 1.0
  a: 0.0
  state: CRUISING
)",
    "0.2", R"([{id: 1, x: 10.0, y: 0.0, length: 4.0, width: 2.0}])");

  const std::string error_message = GetLoadErrorMessage(scenario_file);
  std::filesystem::remove(scenario_file);

  EXPECT_NE(error_message.find("Missing or invalid 'obstacles[0].heading'"), std::string::npos);
}

TEST(ScenarioLoaderTest, RejectsNonFiniteObstacleValue)
{
  const std::string scenario_file = CreateTemporaryScenario(
    "pnc_non_finite_obstacle_value.yaml",
    R"(  x: 0.0
  y: 0.0
  yaw: 0.0
  v: 1.0
  a: 0.0
  state: CRUISING
)",
    "0.2", R"([{id: 1, x: .nan, y: 0.0, heading: 0.0, length: 4.0, width: 2.0}])");

  const std::string error_message = GetLoadErrorMessage(scenario_file);
  std::filesystem::remove(scenario_file);

  EXPECT_NE(error_message.find("obstacles[0].x must be finite"), std::string::npos);
}

TEST(ScenarioLoaderTest, RejectsNegativeObstacleId)
{
  const std::string scenario_file = CreateTemporaryScenario(
    "pnc_negative_obstacle_id.yaml",
    R"(  x: 0.0
  y: 0.0
  yaw: 0.0
  v: 1.0
  a: 0.0
  state: CRUISING
)",
    "0.2", R"([{id: -1, x: 10.0, y: 0.0, heading: 0.0, length: 4.0, width: 2.0}])");

  const std::string error_message = GetLoadErrorMessage(scenario_file);
  std::filesystem::remove(scenario_file);

  EXPECT_NE(error_message.find("obstacles[0].id must be non-negative"), std::string::npos);
}

}  // namespace
}  // namespace pnc_planner::scenario
