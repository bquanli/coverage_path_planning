#include "converters.hh"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace planning_viz::detail
{
namespace msg = foxglove::messages;

msg::Timestamp
toMessageTimestamp(std::uint64_t stamp_ns)
{
  constexpr std::uint64_t ns_per_second = 1'000'000'000;
  auto const seconds = stamp_ns / ns_per_second;
  if(seconds > std::numeric_limits<std::uint32_t>::max())
  {
    throw std::out_of_range("timestamp exceeds Foxglove Timestamp range");
  }
  return msg::Timestamp{
      .sec = static_cast<std::uint32_t>(seconds),
      .nsec = static_cast<std::uint32_t>(stamp_ns % ns_per_second)};
}

msg::SceneUpdate
makeDeletion(std::string_view entity_id, std::uint64_t stamp_ns)
{
  msg::SceneEntityDeletion deletion;
  deletion.timestamp = toMessageTimestamp(stamp_ns);
  deletion.type =
      msg::SceneEntityDeletion::SceneEntityDeletionType::MATCHING_ID;
  deletion.id = std::string(entity_id);

  msg::SceneUpdate update;
  update.deletions.push_back(std::move(deletion));
  return update;
}

msg::SceneUpdate
makeLineUpdate(Points3 points,
               std::string_view entity_id,
               std::string_view frame_id,
               std::uint64_t stamp_ns,
               msg::Color color,
               double width_m,
               bool closed)
{
  auto const timestamp = toMessageTimestamp(stamp_ns);
  if(points.empty())
  {
    return makeDeletion(entity_id, stamp_ns);
  }
  if(points.size() < (closed ? 3U : 2U))
  {
    throw std::invalid_argument("too few points; clearing previous geometry");
  }
  if(!std::isfinite(width_m) || width_m <= 0.0)
  {
    throw std::invalid_argument(
        "invalid line width; clearing previous geometry");
  }
  for(auto const& point : points)
  {
    if(!point.allFinite())
    {
      throw std::invalid_argument(
          "non-finite point; clearing previous geometry");
    }
  }

  msg::LinePrimitive line;
  line.type = closed ? msg::LinePrimitive::LineType::LINE_LOOP
                     : msg::LinePrimitive::LineType::LINE_STRIP;
  line.thickness = width_m;
  line.scale_invariant = false;
  line.color = color;

  msg::Pose identity;
  identity.position = msg::Vector3{0.0, 0.0, 0.0};
  identity.orientation = msg::Quaternion{0.0, 0.0, 0.0, 1.0};
  line.pose = identity;

  line.points.reserve(points.size());
  for(auto const& point : points)
  {
    line.points.push_back(msg::Point3{point.x(), point.y(), point.z()});
  }

  msg::SceneEntity entity;
  entity.id = std::string(entity_id);
  entity.frame_id = std::string(frame_id);
  entity.timestamp = timestamp;
  entity.lifetime = msg::Duration{0, 0};
  entity.frame_locked = false;
  entity.lines.push_back(std::move(line));

  msg::SceneUpdate update;
  update.entities.push_back(std::move(entity));
  return update;
}

} // namespace planning_viz::detail
