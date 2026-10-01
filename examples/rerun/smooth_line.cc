// 演示 coverage_path_planning::planning::smooth_path：构造一条 L 形路径，
// 平滑后用 rerun 对比前后两条线。示例只负责准备数据和展示，
// 算法本体住在 src/planning/path_smoother.cc，由 tests/path_smoother_test.cc 守护。
#include "coverage_path_planning/planning/path_smoother.hh"

#include <rerun.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <cstdio>
#include <exception>
#include <vector>

namespace
{

using coverage_path_planning::planning::RegionShape;
using coverage_path_planning::planning::SmoothOptions;
using coverage_path_planning::planning::smooth_path;
using coverage_path_planning::planning::smoothing_cost;

std::vector<Eigen::Vector2d>
make_l_shaped_path()
{
  std::vector<Eigen::Vector2d> path;
  path.reserve(56);

  // 向右走 60 厘米。
  for(int i = 0; i <= 30; ++i)
  {
    path.emplace_back(0.02 * i, 0.0);
  }

  // 再向上走 50 厘米，不重复添加拐点。
  for(int i = 1; i <= 25; ++i)
  {
    path.emplace_back(0.6, 0.02 * i);
  }

  return path;
}

std::vector<rerun::Vec3D>
to_rerun(std::vector<Eigen::Vector2d> const& path)
{
  std::vector<rerun::Vec3D> points;
  points.reserve(path.size());
  for(auto const& point : path)
  {
    points.emplace_back(static_cast<float>(point.x()),
                        static_cast<float>(point.y()),
                        0.0F);
  }
  return points;
}

double
max_offset(std::vector<Eigen::Vector2d> const& a,
           std::vector<Eigen::Vector2d> const& b)
{
  double largest = 0.0;
  for(std::size_t i = 0; i < a.size(); ++i)
  {
    largest = std::max(largest, (a[i] - b[i]).norm());
  }
  return largest;
}

} // namespace

int
main()
{
  try
  {
    auto const origin = make_l_shaped_path();

    SmoothOptions options;
    options.radius = 0.05;
    options.shape = RegionShape::Circle;
    options.w_ref = 1.0;
    options.w_smooth = 20.0;

    auto const result = smooth_path(origin, options);

    // 这几个数字比图更能说明优化到底有没有生效。
    std::printf("converged   : %s\n", result.converged ? "yes" : "no");
    std::printf("iterations  : %zu / %zu\n",
                result.iterations,
                options.max_iter);
    std::printf("max step    : %.3e (tolerance %.3e)\n",
                result.max_step,
                options.tolerance);
    std::printf("cost        : %.6f -> %.6f\n",
                smoothing_cost(origin, origin, options),
                smoothing_cost(result.points, origin, options));
    std::printf("max offset  : %.6f (radius %.3f)\n",
                max_offset(result.points, origin),
                options.radius);

    rerun::RecordingStream const rec("smooth_line");
    rec.spawn().exit_on_failure();
    rec.log_static("/", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);
    rec.log("/origin",
            rerun::LineStrips3D{rerun::LineStrip3D{to_rerun(origin)}});
    rec.log("/smooth",
            rerun::LineStrips3D{rerun::LineStrip3D{to_rerun(result.points)}});
    rec.log("/smooth/points", rerun::Points3D{to_rerun(result.points)});

    return 0;
  }
  catch(std::exception const& error)
  {
    std::fprintf(stderr, "smooth_line failed: %s\n", error.what());
    return 1;
  }
}
