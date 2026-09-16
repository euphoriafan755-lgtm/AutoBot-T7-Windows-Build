#pragma once

#include "autobot/core/GameSnapshot.hpp"
#include "autobot/world/CollisionWorld.hpp"

#include <cstddef>
#include <vector>

namespace autobot::solver {

enum class GlobalEventKind {
    Solid,
    Hazard,
    Portal,
    Interactive,
    Unknown,
};

struct GlobalWorldEvent {
    std::size_t primitiveIndex = world::kInvalidPrimitiveIndex;
    GlobalEventKind kind = GlobalEventKind::Unknown;
    world::WorldRect bounds{};
    double forwardDistance = 0.0;
    bool dynamic = false;
    bool verifiedGeometry = false;
};

struct GeneralWorldView {
    world::WorldRect queryRect{};
    std::vector<GlobalWorldEvent> events;

    double lookaheadX = 0.0;
    double lookaheadY = 0.0;
    double direction = 1.0;
    std::size_t solids = 0;
    std::size_t hazards = 0;
    std::size_t portals = 0;
    std::size_t interactive = 0;
    std::size_t unknown = 0;
    std::size_t dynamic = 0;
    bool complete = false;
};

class GeneralWorldModelBuilder final {
public:
    [[nodiscard]] GeneralWorldView build(
        core::GameSnapshot const& snapshot,
        world::CollisionWorld const& collisionWorld,
        double lookaheadX,
        double lookaheadY
    ) const;
};

} // namespace autobot::solver
