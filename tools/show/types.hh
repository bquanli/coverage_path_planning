#pragma once

#include <Eigen/Core>

#include <chrono>
#include <cstddef>
#include <optional>

namespace show
{
struct RobotState
{
  Eigen::Vector3d position{0.0, 0.0, 0.0};
  double yaw = 0.0;
  std::optional<double> linear_velocity;
  std::optional<double> angular_velocity;
};

struct Frame
{
  RobotState state;
  std::chrono::milliseconds log_time{};
  std::size_t velocity_count = 0;
};

} // namespace show
