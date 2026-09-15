#pragma once

#include "autobot/world/CollisionTypes.hpp"

class PlayLayer;

namespace autobot::world {

class CollisionWorld;
struct StaticWorld;

class DynamicColliderSync final {
public:
    [[nodiscard]] DynamicSyncStats sync(
        PlayLayer* playLayer,
        StaticWorld& staticWorld,
        CollisionWorld& collisionWorld
    ) const;
};

} // namespace autobot::world
