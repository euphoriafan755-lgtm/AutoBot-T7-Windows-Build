#pragma once

#include "autobot/core/GameSnapshot.hpp"

class PlayLayer;
class PlayerObject;

namespace autobot::core {

class GameStateReader final {
public:
    [[nodiscard]] static GameSnapshot capture(PlayLayer* playLayer, std::uint64_t gameTick);

public:
    [[nodiscard]] static GameMode detectMode(PlayerObject const* player);
};

} // namespace autobot::core
