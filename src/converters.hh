#pragma once

#include "foxglove_viz/viz.hh"

#include <foxglove/messages.hpp>

#include <cstdint>
#include <string_view>

namespace planning_viz::detail
{

foxglove::messages::Timestamp
toMessageTimestamp(std::uint64_t stamp_ns);

foxglove::messages::SceneUpdate
makeDeletion(std::string_view entity_id, std::uint64_t stamp_ns);

// Empty input deletes the entity. Invalid nonempty geometry throws
// invalid_argument; the publisher reports it and clears the old entity.
foxglove::messages::SceneUpdate
makeLineUpdate(Points3 points,
               std::string_view entity_id,
               std::string_view frame_id,
               std::uint64_t stamp_ns,
               foxglove::messages::Color color,
               double width_m,
               bool closed);

} // namespace planning_viz::detail
