#include <rerun.hpp>

#include <Eigen/Eigen>

#include <algorithm>

enum class RegionShape : int8_t
{
  Circle,
  Square,
};

struct SmoothOptions
{
  double radius{0.1};
  RegionShape shape{RegionShape::Circle};

  double w_ref{1.0};
  double w_smooth{20.0};

  std::size_t max_iter{200};
  double tolerance{1e-8};
  bool fixed_end{false};
};

struct SmoothResult
{
  std::vector<Eigen::Vector2d> points;
  std::size_t max_iter{0};
  std::vector<double> costs;
  double max_step{0.0};
  double converged{0.0};
};

double
cost(std::vector<Eigen::Vector2d> const& p,
     std::vector<Eigen::Vector2d> const& o,
     SmoothOptions const& g)
{
  double j = 0.0;
  for(std::size_t i = 0; i < p.size(); ++i)
    j += g.w_ref * (p[i] - o[i]).squaredNorm();
  for(std::size_t i = 1; i + 1 < p.size(); ++i)
    j += g.w_smooth * (p[i - 1] - 2.0 * p[i] + p[i + 1]).squaredNorm();
  return j;
};

SmoothResult
smooth(std::vector<Eigen::Vector2d> const& origin_points,
       SmoothOptions options = {})
{
  if(std::ranges::any_of(
         origin_points,
         [](Eigen::Vector2d const& point)
         { return !std::isfinite(point.x()) || !std::isfinite(point.y()); }))
  {
    throw "origin points is unused!";
  }



  std::vector<Eigen::Vector2d> current = origin_points;
  // 当前目标函数的梯度 Lipschitz 常数上界。
  double const lipschitz = 2.0 * options.w_ref + 32.0 * options.w_smooth;
  if(!std::isfinite(lipschitz))
  {
    throw std::invalid_argument("Weights are too large");
  }

  double const step = 1.0 / lipschitz;
  std::size_t const n = origin_points.size();
  std::vector<Eigen::Vector2d> gridden(n);
  std::vector<Eigen::Vector2d> next(n);
  double max_move = 0.0;
  double max_dx{0.0};
  double max_dy{0.0};
  SmoothResult result;
  result.costs.emplace_back(cost(current, origin_points, options));
  for(std::size_t iter = 0; iter < options.max_iter; ++iter)
  {
    for(std::size_t j = 0; j < n; ++j)
    {
      gridden[j].x() =
          2 * options.w_ref * (current[j].x() - origin_points[j].x());
      gridden[j].y() =
          2 * options.w_ref * (current[j].y() - origin_points[j].y());
    }

    for(std::size_t j = 1; j < n - 1; ++j)
    {
      double dx = current[j - 1].x() - 2 * current[j].x() + current[j + 1].x();
      double dy = current[j - 1].y() - 2 * current[j].y() + current[j + 1].y();

      double gx = 2.0 * options.w_smooth * dx;
      double gy = 2.0 * options.w_smooth * dy;

      gridden[j - 1].x() += gx;
      gridden[j - 1].y() += gy;
      gridden[j].x() -= 2 * gx;
      gridden[j].y() -= 2 * gy;
      gridden[j + 1].x() += gx;
      gridden[j + 1].y() += gy;
    }
    max_move = 0.0;

    for(std::size_t j = 0; j < n; ++j)
    {
      // 这里最核心的是,将当前轮的步长,融合到了相对于origin的了!!!
      // double candidate_x = current[i].x - step * gradient[i].x;
      // double dx = candidate_x - original[i].x;
      double dx = current[j].x() - gridden[j].x() * step - origin_points[j].x();
      double dy = current[j].y() - gridden[j].y() * step - origin_points[j].y();

      if(options.shape == RegionShape::Circle)
      {
        if((dx * dx + dy * dy) > options.radius * options.radius)
        {
          double sclar = options.radius / std::hypot(dx, dy);
          dx *= sclar;
          dy *= sclar;
        }
      }
      else
      {
        dx = std::clamp(dx, -options.radius, options.radius);
        dy = std::clamp(dy, -options.radius, options.radius);
      }
      max_dx = std::max(dx, max_dx);
      max_dy = std::max(dy, max_dy);
      next[j].x() = origin_points[j].x() + dx;
      next[j].y() = origin_points[j].y() + dy;
    }
    current.swap(next);
    result.costs.emplace_back(cost(current, origin_points, options));
  }
  // std::cout << "max dx: " << max_dx << ", max_dy: " << max_dy << '\n';
  result.points = current;
  result.max_iter = 1000;
  return result;
}



int
main()
{
  rerun::RecordingStream const rec("smooth_line");
  rec.spawn().exit_on_failure();
  rec.log_static("/", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);
  std::vector<Eigen::Vector2d> original;
  std::vector<rerun::Vec3D> ori;
  std::vector<rerun::Vec3D> smoths;
  // 向右走 60 厘米。
  for(int i = 0; i <= 30; ++i)
  {
    original.emplace_back(0.02 * i, 0.0);
    ori.emplace_back(static_cast<float>(0.02 * i), 0.0, 0.0);
  }

  // 再向上走 50 厘米，不重复添加拐点。
  for(int i = 1; i <= 25; ++i)
  {
    original.emplace_back(0.6, 0.02 * i);
    ori.emplace_back(0.6, 0.02 * i, 0.0);
  }

  SmoothOptions options;
  options.radius = 0.05;
  options.shape = RegionShape::Circle;
  options.w_ref = 1.0;
  options.w_smooth = 20.0;

  auto result = smooth(original, options);
  smoths.reserve(result.points.size());
  for(auto const& point : result.points)
  {
    smoths.emplace_back(point.x(), point.y(), 0.0);
  }

  // 曲线样式只需要设置一次。
  // 样式与数据必须使用同一个实体路径。
  rec.log_static("optimization/cost",
                 rerun::SeriesLines()
                     .with_names("Total cost")
                     .with_colors(rerun::Color(255, 160, 50))
                     .with_widths(2.0F));
  int i = 0;
  for(auto data : result.costs)
  {
    // 纵轴：当前 cost。变化的数据使用 log，不用 log_static。
    // 横轴：当前迭代次数。
    rec.set_time_sequence("iteration", i++);
    rec.log("optimization/cost", rerun::Scalars(data));
  }

  rec.log("/origin", rerun::LineStrips3D{rerun::LineStrip3D{ori}});
  rec.log("/smooth", rerun::LineStrips3D{rerun::LineStrip3D{smoths}});

  return 0;
}
