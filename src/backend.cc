// Contract: init/shutdown never overlap any publishing call.

#include "backend.hh"
#include "converters.hh"

#include <foxglove/context.hpp>
#include <foxglove/error.hpp>
#include <foxglove/mcap.hpp>
#include <foxglove/messages.hpp>
#include <foxglove/websocket.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace planning_viz::detail
{
namespace
{

namespace msg = foxglove::messages;
using Error = foxglove::FoxgloveError;

// A minimal fallback logger. Replace with the project's logger as needed.
// Throttles ALL publishing errors together to at most one line per second.
void
reportError(char const* operation, char const* message) noexcept
{
  using namespace std::chrono;
  auto const now =
      duration_cast<nanoseconds>(steady_clock::now().time_since_epoch())
          .count();
  static std::atomic<std::int64_t> next_log{
      std::numeric_limits<std::int64_t>::min()};
  auto next = next_log.load(std::memory_order_relaxed);
  if(now < next || !next_log.compare_exchange_strong(next,
                                                     now + 1'000'000'000,
                                                     std::memory_order_relaxed))
  {
    return;
  }
  std::fprintf(stderr, "[planning_viz] %s: %s\n", operation, message);
}

void
appendError(std::string& target, char const* operation, char const* message)
{
  if(!target.empty())
  {
    target += "; ";
  }
  target += operation;
  target += ": ";
  target += message;
}

// Only initialization uses exceptions to unwind a series of SDK create calls.
// The public init() catches them and returns false with an error string.
template <typename T>
T
takeOrThrow(foxglove::FoxgloveResult<T> result, char const* operation)
{
  if(!result.has_value())
  {
    throw std::runtime_error(std::string(operation) + ": " +
                             foxglove::strerror(result.error()));
  }
  return std::move(result.value());
}

// Used both for initialization rollback and explicit shutdown.
// Continue to the writer even if stopping the server reports an error.
void
closeOutputs(std::optional<foxglove::WebSocketServer>& server,
             std::optional<foxglove::McapWriter>& writer,
             std::string& error)
{
  auto const attempt = [&](char const* operation, auto&& action)
  {
    try
    {
      auto const code = action();
      if(code != Error::Ok)
      {
        appendError(error, operation, foxglove::strerror(code));
      }
    }
    catch(std::exception const& ex)
    {
      appendError(error, operation, ex.what());
    }
    catch(...)
    {
      appendError(error, operation, "unknown exception");
    }
  };

  if(server)
  {
    attempt("stop websocket", [&] { return server->stop(); });
    server.reset();
  }
  if(writer)
  {
    attempt("close mcap", [&] { return writer->close(); });
    writer.reset();
  }
}

std::uint64_t
resolveStamp(std::optional<std::uint64_t> stamp)
{
  if(!stamp)
  {
    using namespace std::chrono;
    auto const now =
        duration_cast<nanoseconds>(system_clock::now().time_since_epoch())
            .count();
    if(now < 0)
    {
      throw std::invalid_argument("negative timestamp");
    }
    stamp = static_cast<std::uint64_t>(now);
  }
  // Validate before comparing timestamps or deciding whether to clear geometry.
  (void)toMessageTimestamp(*stamp);
  return *stamp;
}

} // namespace

// Synchronous: consumes points and ctx during this call; saves neither view.
void
Backend::publishLine(SceneChannel& channel,
                     PublicationState& state,
                     Config const& config,
                     Points3 points,
                     DrawContext ctx,
                     std::string_view entity_id,
                     msg::Color color,
                     double width_m,
                     bool closed,
                     char const* operation) noexcept
{
  try
  {
    std::lock_guard lock(state.mutex);
    auto const stamp = resolveStamp(ctx.stamp_ns);
    if(state.last_stamp && stamp <= *state.last_stamp)
    {
      reportError(operation, "timestamp must strictly increase for this topic");
      return;
    }
    std::string_view const frame = ctx.frame_id.empty()
                                       ? std::string_view(config.default_frame)
                                       : ctx.frame_id;

    msg::SceneUpdate message;
    try
    {
      message = makeLineUpdate(points,
                               entity_id,
                               frame,
                               stamp,
                               color,
                               width_m,
                               closed);
    }
    catch(std::invalid_argument const& ex)
    {
      reportError(operation, ex.what());
      message = makeDeletion(entity_id, stamp);
    }

    auto const error = channel.log(message, stamp);
    if(error != Error::Ok)
    {
      reportError(operation, foxglove::strerror(error));
    }
    else
    {
      state.last_stamp = stamp;
    }
  }
  catch(std::exception const& ex)
  {
    reportError(operation, ex.what());
  }
  catch(...)
  {
    reportError(operation, "unknown exception");
  }
}

bool
Backend::init(Config const& config, std::string& error)
{
  error.clear();
  if(initialized_)
  {
    error = "planning_viz is already initialized; call shutdown first";
    return false;
  }
  if(!config.enabled)
  {
    initialized_ = true;
    return true;
  }
  if(config.default_frame.empty())
  {
    error = "default_frame must not be empty";
    return false;
  }
  if(!config.websocket_enabled && !config.mcap_path)
  {
    error = "enable websocket or provide an mcap_path";
    return false;
  }
  if(config.mcap_path && config.mcap_path->empty())
  {
    error = "mcap_path must not be empty when configured";
    return false;
  }

  std::unique_ptr<Resources> candidate;
  try
  {
    candidate = std::make_unique<Resources>();
    candidate->config = config;
    auto& r = *candidate;

    r.global_path.emplace(
        takeOrThrow(SceneChannel::create("/planning/global_path", r.context),
                    "create global_path"));
    r.local_path.emplace(
        takeOrThrow(SceneChannel::create("/planning/local_path", r.context),
                    "create local_path"));
    r.footprint.emplace(
        takeOrThrow(SceneChannel::create("/planning/footprint", r.context),
                    "create footprint"));

    if(config.mcap_path)
    {
      foxglove::McapWriterOptions options;
      options.context = r.context;
      options.path = *r.config.mcap_path;
      options.truncate = false;
      r.writer.emplace(takeOrThrow(foxglove::McapWriter::create(options),
                                   "create mcap writer"));
    }
    if(config.websocket_enabled)
    {
      foxglove::WebSocketServerOptions options;
      options.context = r.context;
      options.host = r.config.host;
      options.port = r.config.port;
      r.server.emplace(
          takeOrThrow(foxglove::WebSocketServer::create(std::move(options)),
                      "create websocket server"));
    }

    resources_ = std::move(candidate);
    initialized_ = true;
    return true;
  }
  catch(std::exception const& ex)
  {
    error = std::string("initialize planning_viz: ") + ex.what();
  }
  catch(...)
  {
    error = "initialize planning_viz: unknown exception";
  }

  if(candidate)
  {
    closeOutputs(candidate->server, candidate->writer, error);
  }
  // candidate destruction releases channels, then their Context.
  // A newly created MCAP file is retained, not deleted on failure.
  return false;
}

bool
Backend::shutdown(std::string& error)
{
  error.clear();
  // Precondition: all publishing threads have stopped and been joined.
  auto old = std::move(resources_);
  initialized_ = false;
  if(old)
  {
    closeOutputs(old->server, old->writer, error);
  }
  return error.empty();
}

void
Backend::globalPath(Points3 points, DrawContext ctx)
{
  if(!resources_)
  {
    return;
  }
  auto& r = *resources_;
  publishLine(*r.global_path,
              r.global_path_state,
              r.config,
              points,
              ctx,
              "path",
              msg::Color{0.2, 0.8, 0.3, 1.0},
              0.04,
              false,
              "globalPath");
}

void
Backend::localPath(Points3 points, DrawContext ctx)
{
  if(!resources_)
  {
    return;
  }
  auto& r = *resources_;
  publishLine(*r.local_path,
              r.local_path_state,
              r.config,
              points,
              ctx,
              "path",
              msg::Color{.r = 1.0, .g = 0.6, .b = 0.1, .a = 1.0},
              0.05,
              false,
              "localPath");
}

void
Backend::footprint(Points3 vertices, DrawContext ctx)
{
  if(!resources_)
  {
    return;
  }
  auto& r = *resources_;
  publishLine(*r.footprint,
              r.footprint_state,
              r.config,
              vertices,
              ctx,
              "footprint",
              msg::Color{0.2, 0.6, 1.0, 1.0},
              0.03,
              true,
              "footprint");
}

void
Backend::clearLocalPath(DrawContext ctx)
{
  // Empty input is handled as MATCHING_ID deletion by publishLine().
  // Use a timestamp later than the entity being removed.
  localPath(Points3{}, ctx);
}

} // namespace planning_viz::detail
