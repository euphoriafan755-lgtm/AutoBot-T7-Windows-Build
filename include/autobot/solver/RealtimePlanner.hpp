#pragma once

#include "autobot/core/GameSnapshot.hpp"
#include "autobot/solver/ActionGenerator.hpp"
#include "autobot/solver/LocalWorldView.hpp"
#include "autobot/solver/PhysicsValidationHarness.hpp"
#include "autobot/solver/SolverTypes.hpp"
#include "autobot/solver/TrajectoryScorer.hpp"
#include "autobot/solver/TrajectorySimulator.hpp"
#include "autobot/world/CollisionWorld.hpp"

namespace autobot::solver {

class RealtimePlanner final {
public:
    [[nodiscard]] PlanDecision plan(
        core::GameSnapshot const& snapshot,
        world::CollisionWorld const& collisionWorld,
        PhysicsValidationHarness const& validation,
        bool botHolding
    ) const;

private:
    [[nodiscard]] static control::InputAction firstInput(
        ActionCandidate const& candidate,
        bool botHolding
    );
    [[nodiscard]] static std::size_t horizonTicks(
        core::GameSnapshot const& snapshot,
        LocalWorldView const& local,
        ModeCalibration const& calibration
    );

    LocalWorldViewBuilder m_localWorldBuilder{};
    ModeActionGenerator m_actionGenerator{};
    TrajectorySimulator m_simulator{};
    TrajectoryScorer m_scorer{};
};

} // namespace autobot::solver
