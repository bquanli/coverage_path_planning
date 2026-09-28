#include "coverage_path_planning/viz/viz.hh"
#include <chrono>
#include <thread>

int
main()
{
  planning_viz::Config config;
  config.host = "127.0.0.1";
  config.port = 8765;
  config.default_frame = "map";
  config.websocket_enabled = true;

  std::string error;
  if(!planning_viz::init(config, error))
  {
    return 1;
  }
  double angle_step = 0.01;
  double radius = 15.0;
  std::vector<Eigen::Vector3d> global_path;
  global_path.reserve(121);
  for(int i = 0; i <= 120; ++i)
  {
    auto const angle = static_cast<double>(i) * angle_step;
    double const yaw = angle + std::numbers::pi / 2.0;

    global_path.emplace_back(radius * std::cos(angle),
                             radius * std::sin(angle),
                             yaw);
  }
  planning_viz::DrawContext dc{
      .stamp_ns = duration_cast<std::chrono::nanoseconds>(
                      std::chrono::system_clock::now().time_since_epoch())
                      .count(),
      .frame_id = "map"};
  for(int i = 0; i < 6000; ++i)
  {
    planning_viz::history_path(global_path, dc);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  // All publishing has finished. Explicit shutdown reports recording errors.
  if(!planning_viz::shutdown(error))
  {
    return 1;
  }
  return 0;
}