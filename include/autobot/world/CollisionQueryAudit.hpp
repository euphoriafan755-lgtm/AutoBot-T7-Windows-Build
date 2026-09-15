#pragma once

#include "autobot/world/CollisionWorld.hpp"

namespace autobot::world {

[[nodiscard]] CollisionQueryResult bruteForceQueryRegion(
    CollisionWorld const& collisionWorld,
    WorldRect const& region
);

} // namespace autobot::world
