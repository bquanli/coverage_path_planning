#pragma once

#include "coverage_path_planning/viz/viz.hh"

#include <foxglove/context.hpp>
#include <foxglove/messages.hpp>
#include <foxglove/websocket.hpp>
#include <foxglove/mcap.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>


namespace planning_viz::detail
{

class Backend
{
public:
  bool
  init(Config const&, std::string& error);
  bool
  shutdown(std::string& error);

  void globalPath(Points3, DrawContext);
  void local_path(Points3, DrawContext);
  void history_path(Points3, DrawContext);
  void trajectory(Points3, DrawContext);
  void trajectory_footprints(std::span<Points3 const>, DrawContext);
  void footprint(Points3, DrawContext);
  void water_leak(WaterLeakView const&, DrawContext);
  void
  odometry(RobotOdometry const&, DrawContext);

  void clearLocalPath(DrawContext);

private:
  using SceneChannel = foxglove::messages::SceneUpdateChannel;

  struct PublicationState
  {
    std::mutex mutex;
    std::optional<std::uint64_t> last_stamp;
  };

  struct Resources
  {
    Config config;

    foxglove::Context context = foxglove::Context::create();

    std::optional<SceneChannel> global_path;
    std::optional<SceneChannel> local_path;
    std::optional<foxglove::messages::PosesInFrameChannel> history_path_channel;
    std::optional<SceneChannel> trajectory;
    std::optional<SceneChannel> footprint;
    std::optional<SceneChannel> water_leak;
    std::optional<foxglove::messages::FrameTransformChannel> transform;
    std::optional<foxglove::messages::OdometryChannel> odometry;

    std::optional<foxglove::McapWriter> writer;
    std::optional<foxglove::WebSocketServer> server;

    PublicationState global_path_state;
    PublicationState local_path_state;
    PublicationState path_state;
    PublicationState trajectory_state;
    PublicationState footprint_state;
    PublicationState water_leak_state;
    PublicationState odometry_state;
  };

  static void
  publish_line(SceneChannel& channel,
               PublicationState& state,
               Config const& config,
               Points3 points,
               DrawContext ctx,
               std::string_view entity_id,
               foxglove::messages::Color color,
               double width_m,
               bool closed,
               char const* operation) noexcept;

  static void
  publish_lines(SceneChannel& channel,
                PublicationState& state,
                Config const& config,
                std::span<Points3 const> lines,
                DrawContext ctx,
                std::string_view entity_id,
                foxglove::messages::Color color,
                double width_m,
                bool closed,
                char const* operation) noexcept;

  static void
  publish_path(foxglove::messages::PosesInFrameChannel& channel,
               PublicationState& state,
               Config const& config,
               Points3 lines,
               DrawContext ctx,
               char const* operation);

  bool initialized_ = false;
  std::unique_ptr<Resources> resources_;
};

} // namespace planning_viz::detail
