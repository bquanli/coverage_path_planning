#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

namespace show
{
struct Options
{
  std::filesystem::path input;
  std::optional<std::string> mcap_path;
  std::string host;
  std::string frame_id;
  std::uint16_t port{};
  double footprint_spacing_m = 0.6;
  double footprint_yaw_step_deg = 5.0;
  double speed{};
  double frame_rate_hz{};
  double start_delay{};
  bool loop{};
  bool websocket{};
  bool show_footprint{};
  bool show_trajectory{};
};

// Read configs/show.yaml; relative paths are resolved from the project root.
Options
load_options();

} // namespace show
