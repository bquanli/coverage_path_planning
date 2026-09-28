#pragma once

#include "coverage_path_planning/common/robot_model/robot_model.hh"
#include <yaml-cpp/yaml.h>


namespace YAML
{

template <>
struct convert<Eigen::Vector2d>
{
  static bool
  decode(Node const& node, Eigen::Vector2d& rhs)
  {
    if(!node.IsSequence() || node.size() != 2)
    {
      return false;
    }

    rhs.x() = node[0].as<double>();
    rhs.y() = node[1].as<double>();

    return true;
  }
};

template <>
struct convert<coverage_path_planning::Footprint>
{
  static bool
  decode(Node const& node, coverage_path_planning::Footprint& rhs)
  {
    if(!node.IsMap())
    {
      return false;
    }

    rhs.type = node["type"].as<std::string>();
    rhs.points = node["points"].as<std::vector<Eigen::Vector2d>>();

    return true;
  }
};

template <>
struct convert<coverage_path_planning::RobotConfig>
{
  static bool
  decode(Node const& node, coverage_path_planning::RobotConfig& rhs)
  {
    if(!node.IsMap())
    {
      return false;
    }

    rhs.name = node["name"].as<std::string>();
    rhs.footprint = node["footprint"].as<coverage_path_planning::Footprint>();

    return true;
  }
};

} // namespace YAML