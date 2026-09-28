#pragma once

#include "coverage_path_planning/common/robot_model/robot_model.hh"

namespace coverage_path_planning
{

class RobotModelHelper
{
public:
  static RobotModelHelper const&
  instance();

private:
  RobotModelHelper();
  RobotConfig robot_config_;
};
}