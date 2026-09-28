#pragma once

#include "types.hh"
#include "coverage_path_planning/common/robot_model/robot_model.hh"

#include <span>
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

} // namespace show
