#pragma once

#include "types.hh"
#include "coverage_path_planning/common/robot_model/robot_model.hh"

#include <span>
#include <memory>
#include <vector>

namespace show
{
using Polygon = std::vector<Eigen::Vector3d>;


Polygon
make_footprint(RobotState const& state,
               coverage_path_planning::Footprint const& footprint);

// Retain endpoints; heading changes increase the sampling density.
std::vector<Polygon>
build_trajectory(std::span<Frame const> frames,
                 coverage_path_planning::Footprint const& footprint,
                 double distance_step_m,
                 double yaw_step_deg);

struct CleaningFootprint
{
  Polygon cloth;    // Closed 300 x 80 mm contact rectangle.
  Polygon squeegee; // Open arc, with an 8 mm suction band.
};

CleaningFootprint
make_cleaning_footprint(RobotState const& state);

struct WaterLeakSnapshot
{
  Polygon
      wet_triangles; // Triples of vertices; preserves holes in the wet mask.
  Polygon wet_boundary_lines; // Pairs of vertices on exposed wet-cell edges.
  double wet_area_m2 = 0.0;
  double covered_area_m2 = 0.0;
  std::size_t clusters = 0; // Four-connected regions >= 0.0025 m².

  double
  leak_rate() const
  {
    return covered_area_m2 > 0.0 ? wet_area_m2 / covered_area_m2 * 100.0 : 0.0;
  }
};

// Incremental port of examples/water_leak_model.py using its default geometry.
// Bounds use the entire log, but only advance() deposits/removes water.
// Pose origin is the wheel axle; yaw=0 points along world +X.
class WaterLeakSimulation
{
public:
  explicit WaterLeakSimulation(std::span<Frame const> frames,
                               bool suction = true);
  ~WaterLeakSimulation();
  void
  reset();
  void
  advance(Frame const& frame);
  WaterLeakSnapshot
  snapshot() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace show
