#include "coverage_path_planning/planning/path_smoother.hh"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace coverage_path_planning::planning
{
namespace
{

bool
is_finite(Eigen::Vector2d const& point)
{
  return std::isfinite(point.x()) && std::isfinite(point.y());
}

void
ensure_same_size(std::vector<Eigen::Vector2d> const& a,
                 std::vector<Eigen::Vector2d> const& b)
{
  if(a.size() != b.size())
  {
    throw std::invalid_argument(
        "path_smoother: points and origin_points must have equal size");
  }
}

void
ensure_valid_weight(double value, char const* name)
{
  if(!std::isfinite(value) || value < 0.0)
  {
    throw std::invalid_argument(
        std::string("path_smoother: ") + name +
        " must be finite and non-negative");
  }
}

void
ensure_valid_options(SmoothOptions const& options)
{
  ensure_valid_weight(options.radius, "radius");
  ensure_valid_weight(options.w_ref, "w_ref");
  ensure_valid_weight(options.w_smooth, "w_smooth");

  if(!std::isfinite(options.tolerance) || options.tolerance < 0.0)
  {
    throw std::invalid_argument(
        "path_smoother: tolerance must be finite and non-negative");
  }
}

void
ensure_valid_input(std::vector<Eigen::Vector2d> const& origin_points)
{
  if(origin_points.size() < k_min_smooth_points)
  {
    throw std::invalid_argument(
        "path_smoother: at least 3 points are required");
  }

  if(!std::ranges::all_of(origin_points, is_finite))
  {
    throw std::invalid_argument(
        "path_smoother: all coordinates must be finite");
  }
}

/// 把偏移量投影回允许区域。offset 是相对原始点的位移，而不是绝对坐标。
Eigen::Vector2d
project_into_region(Eigen::Vector2d offset, SmoothOptions const& options)
{
  if(options.shape == RegionShape::Circle)
  {
    double const length = offset.norm();
    if(length > options.radius)
    {
      offset *= options.radius / length;
    }
    return offset;
  }

  return {std::clamp(offset.x(), -options.radius, options.radius),
          std::clamp(offset.y(), -options.radius, options.radius)};
}

} // namespace

double
smoothing_cost(std::vector<Eigen::Vector2d> const& points,
               std::vector<Eigen::Vector2d> const& origin_points,
               SmoothOptions const& options)
{
  ensure_same_size(points, origin_points);

  double cost = 0.0;
  for(std::size_t i = 0; i < points.size(); ++i)
  {
    cost += options.w_ref * (points[i] - origin_points[i]).squaredNorm();
  }

  for(std::size_t i = 1; i + 1 < points.size(); ++i)
  {
    Eigen::Vector2d const second_difference =
        points[i - 1] - 2.0 * points[i] + points[i + 1];
    cost += options.w_smooth * second_difference.squaredNorm();
  }

  return cost;
}

std::vector<Eigen::Vector2d>
smoothing_gradient(std::vector<Eigen::Vector2d> const& points,
                   std::vector<Eigen::Vector2d> const& origin_points,
                   SmoothOptions const& options)
{
  ensure_same_size(points, origin_points);

  std::size_t const n = points.size();
  std::vector<Eigen::Vector2d> gradient(n);

  for(std::size_t i = 0; i < n; ++i)
  {
    gradient[i] = 2.0 * options.w_ref * (points[i] - origin_points[i]);
  }

  // 一个二阶差分项同时影响三个点，模板为 [+1, -2, +1]。
  for(std::size_t i = 1; i + 1 < n; ++i)
  {
    Eigen::Vector2d const second_difference =
        points[i - 1] - 2.0 * points[i] + points[i + 1];
    Eigen::Vector2d const term = 2.0 * options.w_smooth * second_difference;

    gradient[i - 1] += term;
    gradient[i] -= 2.0 * term;
    gradient[i + 1] += term;
  }

  return gradient;
}

SmoothResult
smooth_path(std::vector<Eigen::Vector2d> const& origin_points,
            SmoothOptions const& options)
{
  ensure_valid_options(options);
  ensure_valid_input(origin_points);

  // J 的 Hessian 是 2*w_ref*I + 2*w_smooth*D^T D，D 为二阶差分算子。
  // ||D^T D|| <= ||D||_1 * ||D||_inf = 4 * 4 = 16，故 L <= 2*w_ref + 32*w_smooth。
  // 步长取 1/L 是 J 单调不增的充分条件。
  double const lipschitz = 2.0 * options.w_ref + 32.0 * options.w_smooth;
  if(!std::isfinite(lipschitz) || lipschitz <= 0.0)
  {
    throw std::invalid_argument(
        "path_smoother: weights are too large or both zero");
  }
  double const step = 1.0 / lipschitz;

  std::size_t const n = origin_points.size();
  std::vector<Eigen::Vector2d> current = origin_points;
  std::vector<Eigen::Vector2d> next(n);

  for(std::size_t iter = 0; iter < options.max_iter; ++iter)
  {
    std::vector<Eigen::Vector2d> const gradient =
        smoothing_gradient(current, origin_points, options);

    double max_step = 0.0;
    for(std::size_t i = 0; i < n; ++i)
    {
      if(options.fix_ends && (i == 0 || i + 1 == n))
      {
        next[i] = origin_points[i];
      }
      else
      {
        // 梯度下降后的候选点，换算成相对原始点的偏移再做投影。
        Eigen::Vector2d const offset =
            current[i] - step * gradient[i] - origin_points[i];
        next[i] = origin_points[i] + project_into_region(offset, options);
      }

      max_step = std::max(max_step, (next[i] - current[i]).norm());
    }

    // 所有点计算完毕后统一切换到新一轮。
    current.swap(next);

    if(max_step < options.tolerance)
    {
      return {std::move(current), iter + 1, max_step, true};
    }

    if(iter + 1 == options.max_iter)
    {
      return {std::move(current), options.max_iter, max_step, false};
    }
  }

  // max_iter 为 0：未做任何迭代，原样返回。
  return {std::move(current), 0, 0.0, false};
}

} // namespace coverage_path_planning::planning
