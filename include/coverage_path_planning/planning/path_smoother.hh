#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

namespace coverage_path_planning::planning
{

/// 每个点允许偏离其原始位置的区域形状。
enum class RegionShape : std::int8_t
{
  /// 以原始点为圆心、radius 为半径的圆盘。
  Circle,
  /// 以原始点为中心、radius 为半边长的正方形，x 与 y 各自独立受限。
  Square,
};

/// 平滑的目标函数为
///   J(p) = w_ref * sum_i |p_i - o_i|^2
///        + w_smooth * sum_{i=1..n-2} |p_{i-1} - 2 p_i + p_{i+1}|^2
/// 第一项把点拉回原始位置，第二项惩罚二阶差分即折角。
struct SmoothOptions
{
  /// 允许偏离原始位置的最大距离，含义随 shape 变化。必须有限且非负。
  double radius{0.05};
  RegionShape shape{RegionShape::Circle};

  /// 贴近原始位置的权重。必须有限且非负。
  double w_ref{1.0};
  /// 二阶差分平滑的权重。必须有限且非负。
  double w_smooth{20.0};

  /// 为真时首尾点锁定在原始位置，不参与移动。
  bool fix_ends{true};

  /// 迭代上限。正常情况下应由 tolerance 提前终止，而不是耗尽上限。
  std::size_t max_iter{10000};
  /// 相邻两轮之间所有点的最大位移小于该值即认为收敛。
  double tolerance{1e-8};
};

struct SmoothResult
{
  std::vector<Eigen::Vector2d> points;
  /// 实际执行的迭代轮数。
  std::size_t iterations{0};
  /// 最后一轮的最大单点位移，即收敛判据的实测值。
  double max_step{0.0};
  /// 为真表示 max_step 已降到 tolerance 以下；为假表示耗尽了 max_iter。
  bool converged{false};
};

/// smooth_path 接受的最少点数。两个点之间不存在二阶差分，无平滑可言。
inline constexpr std::size_t k_min_smooth_points = 3;

/// 在每个点偏离原位不超过 radius 的约束下最小化 J。
///
/// 采用投影梯度下降，步长固定为 1/L，L 是 J 的梯度 Lipschitz 常数上界，
/// 该步长保证 J 单调不增，这也是测试与调试时最有力的判据。
///
/// 抛出 std::invalid_argument：点数少于 k_min_smooth_points、
/// 坐标非有限、或 options 中的权重与半径非法。
[[nodiscard]] SmoothResult
smooth_path(std::vector<Eigen::Vector2d> const& origin_points,
            SmoothOptions const& options = {});

/// 计算目标函数 J。公开它是为了让调用方和测试能够观测优化过程：
/// 单调下降是实现正确性的硬性判据。
///
/// 抛出 std::invalid_argument：两个序列长度不一致。
[[nodiscard]] double
smoothing_cost(std::vector<Eigen::Vector2d> const& points,
               std::vector<Eigen::Vector2d> const& origin_points,
               SmoothOptions const& options);

/// 计算 J 在 points 处的解析梯度，不包含 radius 约束与 fix_ends，
/// 这两者由投影步骤处理。公开它是为了能用中心差分做梯度数值校验。
///
/// 抛出 std::invalid_argument：两个序列长度不一致。
[[nodiscard]] std::vector<Eigen::Vector2d>
smoothing_gradient(std::vector<Eigen::Vector2d> const& points,
                   std::vector<Eigen::Vector2d> const& origin_points,
                   SmoothOptions const& options);

} // namespace coverage_path_planning::planning
