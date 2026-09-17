#pragma once

#include "autobot/core/GameSnapshot.hpp"

class PlayLayer;
class PlayerObject;

namespace autobot::core {

class GameStateReader final {
public:
    [[nodiscard]] static GameSnapshot capture(PlayLayer* playLayer, std::uint64_t gameTick);

    // Runtime mode is latched from Geometry Dash' actual switchedToMode event.
    // Direct PlayerObject mode booleans remain fallback-only because their
    // member layout is not a reliable runtime source on every target build.
    static void resetRuntimeModes();
    static void setRuntimeMode(PlayerObject const* player, GameMode mode);

public:
    [[nodiscard]] static GameMode detectMode(PlayerObject const* player);
};

} // namespace autobot::core
