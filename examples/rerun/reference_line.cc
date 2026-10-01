#include <math.h>

#include <numbers>
#include <rerun.hpp>


#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>


constexpr double pi = std::numbers::pi;

struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

struct Geometry
{
  std::vector<double> s;         // Accumulated polyline length [m].
  std::vector<double> curvature; // Signed curvature [1/m]. Left positive.
};

// y = amplitude * sin(2*pi*x / wavelength).
// x_length is the x-axis span, NOT the arc length.
// Uniform x sampling gives nonuniform arc-length sampling.
std::vector<Point2D>
generateSinePath2(double x_length = 16.0,
                  double amplitude = 0.8,
                  double wavelength = 4.0,
                  double max_dx = 0.02)
{
  if(!std::isfinite(x_length) || !(x_length > 0.0) ||
     !std::isfinite(amplitude) || !std::isfinite(wavelength) ||
     !(wavelength > 0.0) || !std::isfinite(max_dx) || !(max_dx > 0.0))
  {
    throw std::invalid_argument("Invalid sine-path parameters");
  }
  double const count = std::max(2.0, std::ceil(x_length / max_dx));
  if(!std::isfinite(count) || count > 10000000.0)
  {
    throw std::invalid_argument("Too many path samples");
  }
  auto const segments = static_cast<std::size_t>(count);
  std::vector<Point2D> points;
  points.reserve(segments + 1);
  double const omega = 2.0 * pi / wavelength;
  for(std::size_t i = 0; i <= segments; ++i)
  {
    double const x = x_length * (static_cast<double>(i) / segments);
    points.push_back({x, amplitude * std::sin(omega * x)});
  }
  return points;
}
// Include this file AFTER defining your existing Point2D { double x, y; }.
// Replaces the old generateSinePath; parameter meanings have changed.
// All distances are in metres. +x right, +y up; the corner turns LEFT.
// Open path: straight -> quarter circle -> straight -> S-shaped section.
// Joins have continuous tangent. Straight/circle joins have curvature jumps.
inline std::vector<Point2D>
generateSinePath(double first_straight = 3.0,
                 double corner_radius = 0.6,
                 double second_straight = 3.0,
                 double s_length = 8.0,
                 double s_amplitude = 0.8,
                 double max_ds = 0.02)
{
  for(double v :
      {first_straight, corner_radius, second_straight, s_length, max_ds})
  {
    if(!std::isfinite(v) || v <= 0.0)
    {
      throw std::invalid_argument(
          "Lengths, radius and max_ds must be positive");
    }
  }
  if(!std::isfinite(s_amplitude))
  {
    throw std::invalid_argument("Invalid S amplitude");
  }

  // Conservative arc-length bound for the S-section sampling.
  double const omega = 2.0 * pi / s_length;
  double const s_speed_bound =
      std::hypot(1.0, 3.0 * std::abs(s_amplitude) * omega);
  auto const segmentCount = [max_ds](double length_bound)
  {
    double const count = std::max(1.0, std::ceil(length_bound / max_ds));
    if(!std::isfinite(count) || count > 2500000.0)
    {
      throw std::invalid_argument("Too many samples");
    }
    return static_cast<std::size_t>(count);
  };
  auto const n1 = segmentCount(first_straight);
  auto const n2 = segmentCount(pi * corner_radius / 2.0);
  auto const n3 = segmentCount(second_straight);
  auto const n4 = segmentCount(s_length * s_speed_bound);

  std::vector<Point2D> points;
  points.reserve(1 + n1 + n2 + n3 + n4);
  points.push_back({0.0, 0.0});

  // 1. Straight along +x.
  for(std::size_t i = 1; i <= n1; ++i)
  {
    points.push_back({first_straight * (double(i) / n1), 0.0});
  }

  // 2. Quarter circle: heading +x -> +y, curvature +1/radius.
  for(std::size_t i = 1; i <= n2; ++i)
  {
    double const angle = 0.5 * pi * (double(i) / n2);
    points.push_back({first_straight + corner_radius * std::sin(angle),
                      corner_radius * (1.0 - std::cos(angle))});
  }
  double const x0 = first_straight + corner_radius;
  points.back() = {x0, corner_radius};

  // 3. Straight along +y.
  for(std::size_t i = 1; i <= n3; ++i)
  {
    points.push_back({x0, corner_radius + second_straight * (double(i) / n3)});
  }

  // 4. S-section in local (forward, left) coordinates.
  // lateral(u) = amplitude * sin(2*pi*u / length)^3.
  // At both endpoints lateral, lateral' and lateral'' are zero.
  // Positive/negative lateral lobes give alternating left/right turns.
  double const y0 = corner_radius + second_straight;
  for(std::size_t i = 1; i <= n4; ++i)
  {
    double const t = double(i) / n4;
    double const sine = std::sin(2.0 * pi * t);
    double const lateral = s_amplitude * sine * sine * sine;
    points.push_back({x0 - lateral, y0 + s_length * t});
  }
  points.back() = {x0, y0 + s_length};
  return points;
}


// For an OPEN path in Cartesian coordinates (x right, y up).
// Three-point signed circumcircle curvature, valid for nonuniform spacing.
// Empty input returns empty arrays. One/two points have zero curvature.
// Endpoint curvature copies the nearest interior estimate (an approximation).
// Reject repeated/near-repeated adjacent points and degenerate triples.
Geometry
computeGeometry(std::vector<Point2D> const& points, double min_distance = 1e-9)
{
  if(!std::isfinite(min_distance) || !(min_distance > 0.0))
  {
    throw std::invalid_argument("Invalid minimum point distance");
  }
  std::size_t const n = points.size();
  Geometry result{std::vector<double>(n, 0.0), std::vector<double>(n, 0.0)};
  for(auto const& p : points)
  {
    if(!std::isfinite(p.x) || !std::isfinite(p.y))
    {
      throw std::invalid_argument("Nonfinite point coordinate");
    }
  }
  for(std::size_t i = 1; i < n; ++i)
  {
    double const ds = std::hypot(points[i].x - points[i - 1].x,
                                 points[i].y - points[i - 1].y);
    if(!std::isfinite(ds) || ds <= min_distance)
    {
      throw std::invalid_argument("Repeated/invalid adjacent points");
    }
    result.s[i] = result.s[i - 1] + ds;
    if(!std::isfinite(result.s[i]))
    {
      throw std::invalid_argument("Arc-length overflow");
    }
  }
  if(n < 3)
  {
    return result;
  }
  for(std::size_t i = 1; i + 1 < n; ++i)
  {
    double const ax = points[i].x - points[i - 1].x;
    double const ay = points[i].y - points[i - 1].y;
    double const bx = points[i + 1].x - points[i].x;
    double const by = points[i + 1].y - points[i].y;
    double const a = std::hypot(ax, ay);
    double const b = std::hypot(bx, by);
    double const c = std::hypot(points[i + 1].x - points[i - 1].x,
                                points[i + 1].y - points[i - 1].y);
    if(!std::isfinite(c) || c <= min_distance)
    {
      throw std::invalid_argument("Degenerate three-point stencil");
    }
    // Equivalent to 2 * cross(a_vec, b_vec) / (a * b * c).
    double const signed_sine = (ax / a) * (by / b) - (ay / a) * (bx / b);
    result.curvature[i] = 2.0 * signed_sine / c;
  }
  result.curvature.front() = result.curvature[1];
  result.curvature.back() = result.curvature[n - 2];
  return result;
}


// Analytic reference for the GENERATED sine path, not used by computeGeometry.
double
sineCurvature(double x, double amplitude = 0.8, double wavelength = 4.0)
{
  double const omega = 2.0 * pi / wavelength;
  double const dy = amplitude * omega * std::cos(omega * x);
  double const ddy = -amplitude * omega * omega * std::sin(omega * x);
  return ddy / std::pow(1.0 + dy * dy, 1.5);
}

// curvature[i]：第 i 个路径点的有符号曲率
// s[i]：第 i 个路径点的累计弧长，要求非递减
// rate：每米路径允许的最大宽度变化
// tau：曲率系数的增长尺度，与曲率单位相同
std::vector<double>
compute_widths(std::vector<double> const& curvature,
               std::vector<double> const& s,
               double rate = 0.2,
               double tau = 1.0)
{
  if(curvature.size() != s.size() || !(rate > 0.0) || !(tau > 0.0))
  {
    throw std::invalid_argument("Invalid width parameters");
  }

  std::size_t const n = curvature.size();
  std::vector<double> width(n);

  for(std::size_t i = 0; i < n; ++i)
  {
    if(!std::isfinite(curvature[i]) || !std::isfinite(s[i]) ||
       (i > 0 && s[i] < s[i - 1]))
    {
      throw std::invalid_argument("Invalid path data");
    }

    double const k = std::abs(curvature[i]);
    double const default_width = 2.0;

    if(k <= 1.0)
    {
      width[i] = default_width;
    }
    else
    {
      double const coefficient = 0.5 + 0.5 * (-std::expm1(-(k - 1.0) / tau));

      width[i] = default_width * coefficient / k;
    }
  }

  // 前向：限制离开窄区域后的宽度恢复速度。
  for(std::size_t i = 1; i < n; ++i)
  {
    double const ds = s[i] - s[i - 1];
    width[i] = std::min(width[i], width[i - 1] + rate * ds);
  }

  // 反向：让进入窄区域前的宽度提前收窄。
  for(std::size_t i = n; i > 1; --i)
  {
    std::size_t const j = i - 2;
    double const ds = s[j + 1] - s[j];
    width[j] = std::min(width[j], width[j + 1] + rate * ds);
  }

  return width;
}

struct BoundaryPoints
{
  Point2D left;
  Point2D right;
};

BoundaryPoints
computeBoundaryPoints(Point2D const& center,
                      double yaw,
                      double left_width,
                      double right_width)
{
  double const nx = -std::sin(yaw);
  double const ny = std::cos(yaw);

  return {{center.x + left_width * nx, center.y + left_width * ny},

          {center.x - right_width * nx, center.y - right_width * ny}};
}

// points：中心路径点
// s：之前 computeGeometry() 得到的累计弧长
// i：当前点索引
//
// 前提：至少两个点，相邻点不重复，s 严格递增。
Point2D
estimateUnitTangent(std::vector<Point2D> const& points,
                    std::vector<double> const& s,
                    std::size_t i)
{
  std::size_t const n = points.size();
  double tx = NAN;
  double ty = NAN;

  if(i == 0)
  {
    // 起点：前向差分。
    tx = points[1].x - points[0].x;
    ty = points[1].y - points[0].y;
  }
  else if(i == n - 1)
  {
    // 终点：后向差分。
    tx = points[i].x - points[i - 1].x;
    ty = points[i].y - points[i - 1].y;
  }
  else
  {
    // 内点：按不等间距弧长构造三点差分。
    double const h0 = s[i] - s[i - 1];
    double const h1 = s[i + 1] - s[i];

    double const vx0 = (points[i].x - points[i - 1].x) / h0;
    double const vy0 = (points[i].y - points[i - 1].y) / h0;

    double const vx1 = (points[i + 1].x - points[i].x) / h1;
    double const vy1 = (points[i + 1].y - points[i].y) / h1;

    tx = (h1 * vx0 + h0 * vx1) / (h0 + h1);
    ty = (h1 * vy0 + h0 * vy1) / (h0 + h1);
  }

  double const norm = std::hypot(tx, ty);
  if(norm <= 1e-12)
  {
    throw std::runtime_error("无法确定路径切线方向");
  }

  return {tx / norm, ty / norm};
}

int
main()
{
  rerun::RecordingStream const rec("reference_line");
  rec.spawn().exit_on_failure();

  // 世界坐标系：X 向右、Y 向前、Z 向上。
  rec.log_static("/", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);

  // 生成正弦轨迹：x 方向长度、振幅、波长、x 采样间距。
  // 长度单位均为 m。
  try
  {
    auto const points2 =
        generateSinePath(32.0, // x 方向长度，注意不等于路径弧长
                         0.4,  // 振幅
                         8.0,  // 波长
                         0.02  // x 采样间距
        );
    // 从离散坐标计算累计弧长和曲率。
    auto const geometry = computeGeometry(points2);

    // 使用你之前的宽度计算函数。
    auto const widths =
        compute_widths(geometry.curvature, geometry.s, 0.2, 1.0);

    auto s = geometry.s;

    std::vector<rerun::Vector3D> points;
    std::vector<rerun::Vector3D> center;

    points.reserve(s.size());
    center.reserve(s.size());
    for(auto si : s)
    {
      points.emplace_back(si, 0, 0);
    }
    std::vector<rerun::Vector3D> lboundary;
    lboundary.reserve(s.size());
    std::vector<rerun::Vector3D> rboundary;
    rboundary.reserve(s.size());
    for(int i = 0; i < s.size(); ++i)
    {
      lboundary.emplace_back(s[i], widths[i], 0);
      rboundary.emplace_back(s[i], -widths[i], 0);
      center.emplace_back(points2[i].x, points2[i].y, 0);
    }


    std::vector<rerun::Vector3D> left_boundary;
    std::vector<rerun::Vector3D> right_boundary;

    left_boundary.reserve(points2.size());
    right_boundary.reserve(points2.size());

    for(std::size_t i = 0; i < points.size(); ++i)
    {
      auto const t = estimateUnitTangent(points2, geometry.s, i);

      // 左法线 = 切线逆时针旋转 90°。
      double const nx = -t.y;
      double const ny = t.x;

      // 如果左右宽度不同，分别使用 left_widths[i]、right_widths[i]。
      double const wl = widths[i];
      double const wr = widths[i];

      left_boundary.emplace_back(points2[i].x + wl * nx,
                                 points2[i].y + wl * ny,
                                 0);

      right_boundary.emplace_back(points2[i].x - wr * nx,
                                  points2[i].y - wr * ny,
                                  0);
    }

    rec.log("/center_line",
            rerun::LineStrips3D{rerun::LineStrip3D(center)}.with_colors(
                {rerun::Color{255, 255, 255}}));
    rec.log("/left_boundary",
            rerun::LineStrips3D{rerun::LineStrip3D(left_boundary)});
    rec.log("/right_boundary",
            rerun::LineStrips3D{rerun::LineStrip3D(right_boundary)});
  }
  catch(std::exception const& e)
  {}



  return 0;
}