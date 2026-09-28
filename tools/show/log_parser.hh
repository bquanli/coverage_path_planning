#pragma once

#include "types.hh"

#include <filesystem>
#include <fstream>
#include <optional>
#include <regex>

namespace show
{
// Groups a pose with the following velocity records, up to the next pose.
class LogParser
{
public:
  explicit LogParser(std::filesystem::path const& path);

  bool
  next_frame(Frame& frame);

private:
  struct Event
  {
    Frame frame;
    bool pose_changed = false;
  };

  bool
  next(Event& event);

  std::ifstream file_;
  RobotState state_;
  bool has_pose_ = false;
  std::optional<std::chrono::milliseconds> last_time_;
  std::optional<Event> pending_pose_;
  std::chrono::milliseconds day_offset_{};
  std::regex clock_regex_{R"(^\d+:(\d{1,2}):(\d{1,2}):(\d{1,2})\s+(\d{1,3}))"};
  std::regex pose_regex_{
      R"(current pos\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+yaw\s+([-\d.eE+]+))"};
  std::regex velocity_regex_{
      R"(final linearV\s*=\s*([-\d.eE+]+),\s*angularV\s*=\s*([-\d.eE+]+))"};
};

} // namespace show
