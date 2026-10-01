#include "coverage_path_planning/planning/path_smoother.hh"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

namespace
{

using coverage_path_planning::planning::RegionShape;
using coverage_path_planning::planning::SmoothOptions;
using coverage_path_planning::planning::smooth_path;
using coverage_path_planning::planning::smoothing_cost;
using coverage_path_planning::planning::smoothing_gradient;
using Path = std::vector<Eigen::Vector2d>;

/// 向右 60 厘米再向上 50 厘米的 L 形路径，拐角处有一个尖点。
Path
make_l_shaped_path()
{
  Path path;
  for(int i = 0; i <= 30; ++i)
  {
    path.emplace_back(0.02 * i, 0.0);
  }
  for(int i = 1; i <= 25; ++i)
  {
    path.emplace_back(0.6, 0.02 * i);
  }
  return path;
}

SmoothOptions
make_options()
{
  SmoothOptions options;
  options.radius = 0.05;
  options.shape = RegionShape::Circle;
  options.w_ref = 1.0;
  options.w_smooth = 20.0;
  return options;
}

/// 只统计二阶差分能量，用来衡量“有多平滑”而不混入贴近项。
double
smoothness_energy(Path const& path)
{
  double energy = 0.0;
  for(std::size_t i = 1; i + 1 < path.size(); ++i)
  {
    energy += (path[i - 1] - 2.0 * path[i] + path[i + 1]).squaredNorm();
  }
  return energy;
}

} // namespace

// 最有价值的一个测试：解析梯度必须等于目标函数的数值微分。
// 梯度装配写错（漏项、重复累加、分量弄反）会在这里被直接捕获。
TEST(PathSmoother, GradientMatchesCentralDifference)
{
  Path const origin = make_l_shaped_path();
  SmoothOptions const options = make_options();

  // 必须在偏离原始位置的点上校验：原始位置处贴近项梯度恰好为零，测不出问题。
  Path probe = origin;
  probe[20] += Eigen::Vector2d(0.01, -0.02);
  probe[30] += Eigen::Vector2d(-0.005, 0.015);
  probe[35] += Eigen::Vector2d(-0.015, 0.01);

  Path const analytic = smoothing_gradient(probe, origin, options);
  ASSERT_EQ(analytic.size(), probe.size());

  constexpr double eps = 1e-6;
  for(std::size_t i = 0; i < probe.size(); ++i)
  {
    for(int axis = 0; axis < 2; ++axis)
    {
      Path forward = probe;
      Path backward = probe;
      forward[i][axis] += eps;
      backward[i][axis] -= eps;

      double const numeric = (smoothing_cost(forward, origin, options) -
                              smoothing_cost(backward, origin, options)) /
                             (2.0 * eps);

      EXPECT_NEAR(numeric, analytic[i][axis], 1e-6)
          << "point " << i << " axis " << axis;
    }
  }
}

// 步长取 1/L 时梯度下降保证 J 单调不增，这是不需要人工判断的硬指标。
// J 出现反弹即意味着迭代存在 bug；奇偶振荡通常指向双缓冲区有字段未写全。
TEST(PathSmoother, CostDecreasesMonotonically)
{
  Path const origin = make_l_shaped_path();
  SmoothOptions options = make_options();

  double previous = std::numeric_limits<double>::infinity();
  for(std::size_t iterations = 0; iterations <= 200; ++iterations)
  {
    options.max_iter = iterations;
    auto const result = smooth_path(origin, options);
    double const cost = smoothing_cost(result.points, origin, options);

    EXPECT_LE(cost, previous + 1e-12) << "cost increased at iteration "
                                      << iterations;
    previous = cost;
  }

  EXPECT_LT(previous, smoothing_cost(origin, origin, options));
}

TEST(PathSmoother, ConvergesBeforeIterationLimit)
{
  Path const origin = make_l_shaped_path();
  SmoothOptions const options = make_options();

  auto const result = smooth_path(origin, options);

  EXPECT_TRUE(result.converged);
  EXPECT_LT(result.iterations, options.max_iter);
  EXPECT_LT(result.max_step, options.tolerance);
  EXPECT_EQ(result.points.size(), origin.size());
}

TEST(PathSmoother, ReportsFailureWhenIterationLimitIsHit)
{
  Path const origin = make_l_shaped_path();
  SmoothOptions options = make_options();
  options.max_iter = 5;

  auto const result = smooth_path(origin, options);

  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.iterations, 5U);
  EXPECT_GT(result.max_step, options.tolerance);
}

TEST(PathSmoother, CornerBecomesSmoother)
{
  Path const origin = make_l_shaped_path();
  SmoothOptions const options = make_options();

  auto const result = smooth_path(origin, options);

  EXPECT_LT(smoothness_energy(result.points), smoothness_energy(origin) * 0.5);
}

TEST(PathSmoother, OffsetStaysInsideCircleRegion)
{
  Path const origin = make_l_shaped_path();
  SmoothOptions options = make_options();
  options.shape = RegionShape::Circle;
  options.radius = 0.01;

  auto const result = smooth_path(origin, options);

  for(std::size_t i = 0; i < origin.size(); ++i)
  {
    EXPECT_LE((result.points[i] - origin[i]).norm(), options.radius + 1e-12)
        << "point " << i;
  }
}

TEST(PathSmoother, OffsetStaysInsideSquareRegion)
{
  Path const origin = make_l_shaped_path();
  SmoothOptions options = make_options();
  options.shape = RegionShape::Square;
  options.radius = 0.01;

  auto const result = smooth_path(origin, options);

  for(std::size_t i = 0; i < origin.size(); ++i)
  {
    Eigen::Vector2d const offset = result.points[i] - origin[i];
    EXPECT_LE(std::abs(offset.x()), options.radius + 1e-12) << "point " << i;
    EXPECT_LE(std::abs(offset.y()), options.radius + 1e-12) << "point " << i;
  }
}

TEST(PathSmoother, ZeroRadiusLeavesPathUnchanged)
{
  Path const origin = make_l_shaped_path();
  SmoothOptions options = make_options();
  options.radius = 0.0;

  auto const result = smooth_path(origin, options);

  for(std::size_t i = 0; i < origin.size(); ++i)
  {
    EXPECT_NEAR(result.points[i].x(), origin[i].x(), 1e-15);
    EXPECT_NEAR(result.points[i].y(), origin[i].y(), 1e-15);
  }
}

TEST(PathSmoother, FixedEndsStayExactlyInPlace)
{
  Path const origin = make_l_shaped_path();
  SmoothOptions options = make_options();
  options.fix_ends = true;

  auto const result = smooth_path(origin, options);

  EXPECT_EQ(result.points.front(), origin.front());
  EXPECT_EQ(result.points.back(), origin.back());
}

TEST(PathSmoother, FreeEndsMayMove)
{
  Path const origin = make_l_shaped_path();
  SmoothOptions options = make_options();
  options.fix_ends = false;

  auto const result = smooth_path(origin, options);

  EXPECT_EQ(result.points.size(), origin.size());
  for(auto const& point : result.points)
  {
    EXPECT_TRUE(std::isfinite(point.x()));
    EXPECT_TRUE(std::isfinite(point.y()));
  }
}

// 直线已经是最优解，平滑器应该原地不动。这类不动点测试能捕获漏写分量
// 或符号弄反这类 bug，因为任何负作用都会把点推走。
TEST(PathSmoother, StraightLineIsFixedPoint)
{
  Path origin;
  for(int i = 0; i < 20; ++i)
  {
    origin.emplace_back(0.05 * i, 0.0);
  }
  SmoothOptions const options = make_options();

  auto const result = smooth_path(origin, options);

  EXPECT_TRUE(result.converged);
  for(std::size_t i = 0; i < origin.size(); ++i)
  {
    EXPECT_NEAR(result.points[i].x(), origin[i].x(), 1e-12) << "point " << i;
    EXPECT_NEAR(result.points[i].y(), origin[i].y(), 1e-12) << "point " << i;
  }
}

TEST(PathSmoother, RejectsTooFewPoints)
{
  EXPECT_THROW(smooth_path({}), std::invalid_argument);
  EXPECT_THROW(smooth_path({Eigen::Vector2d(0.0, 0.0)}),
               std::invalid_argument);
  EXPECT_THROW(
      smooth_path({Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.1, 0.0)}),
      std::invalid_argument);
}

TEST(PathSmoother, RejectsNonFiniteCoordinates)
{
  Path origin = make_l_shaped_path();
  origin[10].y() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(smooth_path(origin), std::invalid_argument);

  origin[10].y() = std::numeric_limits<double>::infinity();
  EXPECT_THROW(smooth_path(origin), std::invalid_argument);
}

TEST(PathSmoother, RejectsInvalidOptions)
{
  Path const origin = make_l_shaped_path();

  SmoothOptions negative_radius = make_options();
  negative_radius.radius = -0.01;
  EXPECT_THROW(smooth_path(origin, negative_radius), std::invalid_argument);

  SmoothOptions negative_weight = make_options();
  negative_weight.w_smooth = -1.0;
  EXPECT_THROW(smooth_path(origin, negative_weight), std::invalid_argument);

  SmoothOptions nan_tolerance = make_options();
  nan_tolerance.tolerance = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(smooth_path(origin, nan_tolerance), std::invalid_argument);
}

TEST(PathSmoother, RejectsMismatchedSizesInDiagnostics)
{
  Path const origin = make_l_shaped_path();
  Path const shorter(origin.begin(), origin.begin() + 10);
  SmoothOptions const options = make_options();

  EXPECT_THROW(static_cast<void>(smoothing_cost(shorter, origin, options)),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(smoothing_gradient(shorter, origin, options)),
               std::invalid_argument);
}
