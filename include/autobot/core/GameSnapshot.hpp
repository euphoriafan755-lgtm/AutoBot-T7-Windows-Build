#pragma once

#include <cstdint>
#include <string_view>

namespace autobot::core {

enum class GameMode {
    Cube,
    Ship,
    Ball,
    Ufo,
    Wave,
    Robot,
    Spider,
    Swing,
    Unknown,
};

inline constexpr std::string_view toString(GameMode mode) {
    switch (mode) {
        case GameMode::Cube: return "CUBE";
        case GameMode::Ship: return "SHIP";
        case GameMode::Ball: return "BALL";
        case GameMode::Ufo: return "UFO";
        case GameMode::Wave: return "WAVE";
        case GameMode::Robot: return "ROBOT";
        case GameMode::Spider: return "SPIDER";
        case GameMode::Swing: return "SWING";
        default: return "UNKNOWN";
    }
}

struct PlayerState {
    double x = 0.0;
    double y = 0.0;
    double velocityX = 0.0;
    double velocityY = 0.0;

    double gravity = 0.0;
    double gravityModifier = 1.0;
    double jumpAcceleration = 0.0;
    double speed = 0.0;

    // PlayerObject object bounds only; not claimed to equal exact gameplay hitbox.
    double objectBoundsWidth = 0.0;
    double objectBoundsHeight = 0.0;

    GameMode mode = GameMode::Unknown;

    bool mini = false;
    bool grounded = false;
    bool upsideDown = false;
    bool holding = false;
    bool dead = false;
};

enum class TickSource {
    PostUpdateSequence,
};

struct GameSnapshot {
    bool valid = false;

    std::uint64_t gameTick = 0;
    std::uint64_t solverSampleID = 0;
    TickSource tickSource = TickSource::PostUpdateSequence;
    bool exactGameTickVerified = false;

    double levelTime = 0.0;
    float levelProgress = 0.0f;
    PlayerState player{};
};

} // namespace autobot::core
