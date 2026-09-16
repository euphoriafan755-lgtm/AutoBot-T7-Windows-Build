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


enum class TriggerKind {
    None,
    Move,
    Rotate,
    Scale,
    Toggle,
    Spawn,
    Follow,
    FollowPlayerY,
    Stop,
    TimeWarp,
    Gravity,
};

enum class TriggerCommandMode {
    Stop,
    Pause,
    Resume,
};

struct TriggerDescriptor {
    TriggerKind kind = TriggerKind::None;
    TriggerCommandMode commandMode = TriggerCommandMode::Stop;
    bool gameplayRelevant = false;
    bool spawnTriggered = false;
    bool touchTriggered = false;
    bool multiTriggered = false;
    bool uncertain = false;

    int targetGroupID = 0;
    int centerGroupID = 0;
    double durationSeconds = 0.0;
    double spawnDelaySeconds = 0.0;

    double moveX = 0.0;
    double moveY = 0.0;
    double rotationDegrees = 0.0;
    double scaleX = 1.0;
    double scaleY = 1.0;
    bool divideScaleX = false;
    bool divideScaleY = false;

    bool toggleOn = true;
    double followXMod = 1.0;
    double followYMod = 1.0;
    double followYSpeed = 1.0;
    double followYDelay = 0.0;
    double followYOffset = 0.0;
    double followYMaxSpeed = 0.0;

    double timeWarp = 1.0;
    double gravityValue = 1.0;
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
    // Persistent copied identity. objectID is the GD object kind and is NOT
    // unique per instance. sourceIndex (the index in StaticWorld::objects)
    // remains the canonical in-memory identity for this parse.
    int objectID = 0;
    int uniqueID = 0;
    std::size_t playLayerObjectIndex = 0;
    int rawGameObjectType = -1;
    GameplayObjectType type = GameplayObjectType::Unknown;
    V01Support v01Support = V01Support::NotSupported;

    // World/real position used by CollisionWorld.
    float x = 0.0f;
    float y = 0.0f;

    // Raw node transform copied for diagnostics. These are never used to
    // mutate broad-phase bounds or invent gameplay hitboxes.
    float nodeX = 0.0f;
    float nodeY = 0.0f;
    float rotation = 0.0f;
    float rotationX = 0.0f;
    float rotationY = 0.0f;
    float scaleX = 1.0f;
    float scaleY = 1.0f;
    float anchorX = 0.5f;
    float anchorY = 0.5f;
    float contentWidth = 0.0f;
    float contentHeight = 0.0f;

    // GameObject::getObjectRect() copied in world/object-layer coordinates.
    // This is an OBJECT BOUND / broad-phase bound only.
    WorldRect objectRect{};

    bool enabled = true;
    bool groupDisabled = false;
    bool noTouch = false;
    bool passable = false;
    bool flipX = false;
    bool flipY = false;
    bool slope = false;
    int groupCount = 0;
    std::vector<int> groups;
    TriggerDescriptor trigger{};
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
