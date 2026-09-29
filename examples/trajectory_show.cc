#include "../tools/show/options.hh"
#include "../tools/show/replay.hh"
#include "coverage_path_planning/common/log/log.hh"
#include "coverage_path_planning/common/robot_model/robot_model_helper.hh"

#include <exception>

int
main()
{
  try
  {
    // Share real-log replay, water simulation and signal handling with show.
    // configs/show.yaml controls input, endpoint, frame rate and looping.
    auto const options = show::load_options();
    auto const& footprint =
        coverage_path_planning::RobotModelHelper::instance().config().footprint;
    return show::run(options, footprint);
  }
  catch(std::exception const& ex)
  {
    coverage_path_planning::log::error("trajectory_show: {}", ex.what());
    return 1;
  }
}
