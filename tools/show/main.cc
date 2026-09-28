#include "options.hh"
#include "replay.hh"
#include "coverage_path_planning/common/log/log.hh"
#include "coverage_path_planning/common/robot_model/robot_model_helper.hh"

#include <exception>

int
main()
{
  try
  {
    auto const options = show::load_options();
    auto const& footprint =
        coverage_path_planning::RobotModelHelper::instance().config().footprint;
    return show::run(options, footprint);
  }
  catch(std::exception const& ex)
  {
    coverage_path_planning::log::error("show: {}", ex.what());
    return 1;
  }
}
