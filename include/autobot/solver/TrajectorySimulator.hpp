#pragma once

#include "autobot/core/GameSnapshot.hpp"
#include "autobot/solver/LocalWorldView.hpp"
#include "autobot/solver/PhysicsModels.hpp"
#include "autobot/solver/PortalTransition.hpp"
#include "autobot/solver/SolverTypes.hpp"
#include "autobot/world/CollisionWorld.hpp"
#include "autobot/world/DynamicWorldModel.hpp"
#include "autobot/world/TriggerWorldModel.hpp"

#include <cstddef>

namespace autobot::solver {

class TrajectorySimulator final {
public:
    [[nodiscard]] TrajectoryResult simulate(
        core::GameSnapshot const& snapshot,
        world::CollisionWorld const& collisionWorld,
        LocalWorldView const& localWorld,
        PhysicsValidationHarness const& validation,
        ActionCandidate const& candidate,
        std::size_t horizonTicks,
        bool initialHolding,
        double requiredForwardDistance,
        world::DynamicWorldModel const* dynamicWorld = nullptr,
        world::TriggerWorldModel const* triggerWorld = nullptr,
        bool exactHorizon = false,
        bool stopAtRequiredDistance = false
    ) const;

private:
    ModePhysicsRegistry m_registry{};
};

} // namespace autobot::solver
