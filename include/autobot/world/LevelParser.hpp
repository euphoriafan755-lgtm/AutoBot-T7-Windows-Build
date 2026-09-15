#pragma once

#include "autobot/world/WorldObject.hpp"

#include <cstddef>

class PlayLayer;
enum class GameObjectType;

namespace autobot::world {

class LevelParser final {
public:
    [[nodiscard]] static StaticWorld parse(PlayLayer* playLayer);
    [[nodiscard]] static bool snapshotObjectAt(PlayLayer* playLayer, std::size_t objectArrayIndex, WorldObject& out);

private:
    [[nodiscard]] static GameplayObjectType classify(GameObjectType type, int objectID);
    [[nodiscard]] static V01Support classifyV01Support(GameObjectType type, int objectID);
};

} // namespace autobot::world
