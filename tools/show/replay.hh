#pragma once

#include "options.hh"
#include "coverage_path_planning/common/robot_model/robot_model.hh"

namespace show
{
// Owns visualization startup, replay and shutdown. Returns a process exit code.
int
run(Options const& options, coverage_path_planning::Footprint const& footprint);

} // namespace show
