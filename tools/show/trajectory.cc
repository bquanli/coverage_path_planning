#include "trajectory.hh"

#include <cmath>
#include <numbers>

namespace show
{
Polygon
make_footprint(RobotState const& state,
               coverage_path_planning::Footprint const& footprint)
{
  Polygon vertices;
  vertices.reserve(footprint.points.size());
  auto const c = std::cos(state.yaw);
  auto const s = std::sin(state.yaw);
  for(auto const& point : footprint.points)
  {
    vertices.emplace_back(state.position.x() + c * point.x() - s * point.y(),
                          state.position.y() + s * point.x() + c * point.y(),
                          state.position.z());
  }
  return vertices;
}

// Select real log poses, using distance and accumulated heading change together.
// This also samples in-place turns and handles the -pi/pi yaw boundary.
std::vector<Polygon>
build_trajectory(std::span<Frame const> frames,
                 coverage_path_planning::Footprint const& footprint,
                 double distance_step_m,
                 double yaw_step_deg)
{
  std::vector<Polygon> polygons;
  if(frames.empty())
  {
    return polygons;
  }
  auto const yaw_step = yaw_step_deg * std::numbers::pi / 180.0;
  
  polygons.push_back(make_footprint(frames.front().state, footprint));
  double progress = 0.0;
  std::size_t last_selected = 0;
  for(std::size_t i = 1; i < frames.size(); ++i)
  {
    auto const& previous = frames[i - 1].state;
    auto const& current = frames[i].state;
    auto const distance = (current.position - previous.position).norm();
    auto const angle = std::abs(
        std::remainder(current.yaw - previous.yaw, 2.0 * std::numbers::pi));
    progress += distance / distance_step_m + angle / yaw_step;
    if(progress >= 1.0)
    {
      polygons.push_back(make_footprint(current, footprint));
      last_selected = i;
      progress = 0.0;
    }
  }
  auto const& last = frames.back().state;
  auto const& selected = frames[last_selected].state;
  if((last.position - selected.position).norm() > 1e-9 ||
     std::abs(std::remainder(last.yaw - selected.yaw, 2.0 * std::numbers::pi)) >
         1e-9)
  {
    polygons.push_back(make_footprint(last, footprint));
  }
  return polygons;
}

} // namespace show
