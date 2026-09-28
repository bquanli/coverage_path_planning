#pragma once

#include <Eigen/Core>
#include <span>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace planning_viz
{

// 明确表示 xyz；如果数据是 x、y、yaw，需要单独适配。
using Points3 = std::span<Eigen::Vector3d const>;

struct Config
{
  bool enabled = true;

  bool websocket_enabled = true;
  std::string host = "127.0.0.1";
  std::uint16_t port = 8765;

  std::optional<std::string> mcap_path;
  std::string default_frame = "map";
};

struct DrawContext
{
  // 未指定时使用当前时间；显式传入 0 仍是合法值。
  // 同一 topic 的更新和清除必须严格递增；相同或倒退时间会被拒绝。
  // 不同 topic 可以使用同一规划周期的时间戳。
  std::optional<std::uint64_t> stamp_ns;

  // 空字符串表示使用 default_frame。
  std::string_view frame_id;
};

// init/shutdown 必须与彼此及全部发布调用互斥。
// 启动业务线程前 init，停止并 join 后 shutdown；成功清空 error。
// 重复 init 失败（包括 enabled=false）；重复 shutdown 成功。
bool
init(Config const& config, std::string& error);
bool
shutdown(std::string& error);

// 同步消费 points/ctx，不保存视图；调用期间不得修改它们的底层数据。
// 未初始化或禁用时无操作。空输入清除该 topic 的旧实体；无效几何
// 报告错误并清除旧实体。无效/非递增时间戳则跳过整次调用。
void
global_path(Points3 points, DrawContext ctx = {});
void
local_path(Points3 points, DrawContext ctx = {});
void
footprint(Points3 vertices, DrawContext ctx = {});

// 时间戳必须晚于最近一次 localPath/clearLocalPath 成功提交的时间。
void
clear_local_path(DrawContext ctx = {});

} // namespace planning_viz
