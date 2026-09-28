#pragma once

#include "coverage_path_planning/common/robot_model/robot_model.hh"

namespace coverage_path_planning
{

class RobotModelHelper
{
public:
  static RobotModelHelper const&
  instance();

  [[nodiscard]] RobotConfig const&
  config() const noexcept
  {
    return robot_config_;
  }

private:
  RobotModelHelper();
  RobotConfig robot_config_;
};
} // namespace coverage_path_planning
