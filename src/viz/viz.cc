#include "coverage_path_planning/viz/viz.hh"

#include "coverage_path_planning/viz/backend.hh"

namespace planning_viz
{

namespace
{

detail::Backend&
backend()
{
  static detail::Backend instance;
  return instance;
}

} // namespace

bool
init(Config const& config, std::string& error)
{
  return backend().init(config, error);
}

bool
shutdown(std::string& error)
{
  return backend().shutdown(error);
}

void
global_path(Points3 points, DrawContext ctx)
{
  backend().globalPath(points, ctx);
}

void
local_path(Points3 points, DrawContext ctx)
{
  backend().localPath(points, ctx);
}

void
footprint(Points3 vertices, DrawContext ctx)
{
  backend().footprint(vertices, ctx);
}

void
clear_local_path(DrawContext ctx)
{
  backend().clearLocalPath(ctx);
}

} // namespace planning_viz
