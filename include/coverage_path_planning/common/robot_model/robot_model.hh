#pragma once

#include <Eigen/Core>

#include <string>
#include <vector>
namespace coverage_path_planning
{

struct Footprint
{
  std::string type;
  std::vector<Eigen::Vector2d> points;
};

struct RobotConfig
{
  std::string name;
  Footprint footprint;
};

}