#pragma once

#include "autobot/core/GameSnapshot.hpp"
#include "autobot/solver/SolverTypes.hpp"
#include "autobot/world/CollisionWorld.hpp"

namespace autobot::solver {

class LocalWorldViewBuilder final {
public:
    [[nodiscard]] LocalWorldView build(
        core::GameSnapshot const& snapshot,
        world::CollisionWorld const& collisionWorld
    ) const;
};

} // namespace autobot::solver
