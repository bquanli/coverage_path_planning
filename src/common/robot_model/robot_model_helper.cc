#include "coverage_path_planning/common/log/log.hh"
#include "coverage_path_planning/common/robot_model/robot_model_helper.hh"

#include "coverage_path_planning/common/robot_model/robot_config_yaml.hh"

namespace coverage_path_planning
{

RobotModelHelper const&
RobotModelHelper::instance()
{
  static RobotModelHelper helper;
  return helper;
}


RobotModelHelper::RobotModelHelper()
{
  auto config =
      YAML::LoadFile("/workspace/coverage_path_planning/data/robot.yaml");
  log::info("RobotConfig:{}", YAML::Dump(config));
  robot_config_ = config.as<RobotConfig>();
}
} // namespace coverage_path_planning
