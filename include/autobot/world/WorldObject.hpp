#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace autobot::world {

enum class GameplayObjectType {
    Solid,
    Hazard,
    Orb,
    Pad,
    Portal,
    Decoration,
    Unknown,
};

enum class V01Support {
    Supported,
    NotSupported,
    NonGameplay,
};

struct WorldRect {
    float x = 0.0f;
    float y = 0.0f;
    float width = 0.0f;
    float height = 0.0f;
};

struct WorldObject {
    int objectID = 0;
    int rawGameObjectType = -1;
    GameplayObjectType type = GameplayObjectType::Unknown;
    V01Support v01Support = V01Support::NotSupported;

    float x = 0.0f;
    float y = 0.0f;
    float rotation = 0.0f;
    float scaleX = 1.0f;
    float scaleY = 1.0f;

    WorldRect objectRect{};

    bool enabled = true;
    bool slope = false;
};

struct StaticWorld {
    bool parsed = false;
    std::string error;
    std::vector<WorldObject> objects;

    std::size_t solids = 0;
    std::size_t hazards = 0;
    std::size_t orbs = 0;
    std::size_t pads = 0;
    std::size_t portals = 0;
    std::size_t decorations = 0;
    std::size_t unknown = 0;
    std::size_t unsupportedGameplay = 0;
};

} // namespace autobot::world
