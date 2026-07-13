#include "pnc_planner/scenario/scenario_loader.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace pnc_planner::scenario {
  ScenarioData ScenarioLoader::LoadFromFile(
    const std::string &scenario_file_path) {
    if (scenario_file_path.empty()) {
      throw std::runtime_error("Scenario file path is empty");
    }

    YAML::Node root;

    try {
      root = YAML::LoadFile(scenario_file_path);
    } catch (const YAML::Exception &exception) {
      throw std::runtime_error(
        "Failed to load scenario file: '" + scenario_file_path +
        "': " + exception.what()
      );
    }

    try {
      if (!root.IsMap()) {
        throw std::runtime_error(
          "Scenario root must be a YAML map");
      }

      const YAML::Node schema_version_node = root["schema_version"];
      if (!schema_version_node || !schema_version_node.IsScalar()) {
        throw std::runtime_error(
          "Missing or invalid 'schema_version'");
      }

      ScenarioData scenario_data;

      const std::string schema_version =
          schema_version_node.as<std::string>();

      if (schema_version != "0.1") {
        throw std::runtime_error(
          "Unsupported schema version: " +
          schema_version);
      }
      scenario_data.schema_version = schema_version;

      const YAML::Node name_node = root["name"];
      if (!name_node || !name_node.IsScalar()) {
        throw std::runtime_error(
          "Missing or invalid 'name'");
      }

      const std::string name = name_node.as<std::string>();

      if (name.empty()) {
        throw std::runtime_error(
          "Scenario name must not be empty");
      }
      scenario_data.name = name;

      const YAML::Node route_node = root["route"];
      if (!route_node || !route_node.IsMap()) {
        throw std::runtime_error(
          "Missing or invalid 'route'");
      }

      const YAML::Node frame_id_node = route_node["frame_id"];
      if (!frame_id_node || !frame_id_node.IsScalar()) {
        throw std::runtime_error(
          "Missing or invalid 'route.frame_id'");
      }
      const std::string frame_id = frame_id_node.as<std::string>();

      // Schema v0.1 only accepts routes in the map frame.
      if (frame_id != "map") {
        throw std::runtime_error(
          "route.frame_id must be 'map'");
      }
      scenario_data.frame_id = frame_id;

      const YAML::Node points_node = route_node["points"];
      if (!points_node || !points_node.IsSequence()) {
        throw std::runtime_error(
          "Missing or invalid 'route.points'");
      }

      if (points_node.size() < 3U) {
        throw std::runtime_error(
          "route.points must contain at least 3 points");
      }
      scenario_data.route_points.reserve(points_node.size());

      // Each route point must be a finite 2D coordinate [x, y].
      for (std::size_t index = 0; index < points_node.size(); ++index) {
        const YAML::Node point_node = points_node[index];

        if (!point_node.IsSequence() || point_node.size() != 2U) {
          throw std::runtime_error(
            "route.points[" + std::to_string(index) +
            "] must have exactly two values");
        }

        const double x = point_node[0].as<double>();
        const double y = point_node[1].as<double>();

        if (!std::isfinite(x) || !std::isfinite(y)) {
          throw std::runtime_error(
            "route.points[" + std::to_string(index) +
            "] must contain finite x and y values");
        }
        scenario_data.route_points.push_back({x, y});
      }

      const YAML::Node ego_node = root["ego"];
      if (!ego_node || !ego_node.IsMap()) {
        throw std::runtime_error(
          "Missing or invalid 'ego'");
      }

      const YAML::Node obstacles_node = root["obstacles"];
      if (!obstacles_node || !obstacles_node.IsSequence()) {
        throw std::runtime_error(
          "Missing or invalid 'obstacles'");
      }

      if (obstacles_node.size() != 0U) {
        throw std::runtime_error(
          "Non-empty obstacles are not supported in scenario schema v0.1");
      }

      const YAML::Node expected_node = root["expected"];
      if (!expected_node || !expected_node.IsMap()) {
        throw std::runtime_error(
          "Missing or invalid 'expected'");
      }

      return scenario_data;
    } catch (const YAML::Exception &exception) {
      throw std::runtime_error(
        "Failed to parse scenario file '" + scenario_file_path +
        "': " + exception.what());
    }
  }
} // namespace pnc_planner::scenario
