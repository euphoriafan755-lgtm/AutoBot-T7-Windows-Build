#pragma once

#include "autobot/core/GameSnapshot.hpp"
#include "autobot/solver/ActionGenerator.hpp"
#include "autobot/solver/AdaptiveSearchBudget.hpp"
#include "autobot/solver/GeneralWorldModel.hpp"
#include "autobot/solver/DualJointPlanner.hpp"
#include "autobot/solver/GlobalPlanner.hpp"
#include "autobot/solver/LocalWorldView.hpp"
#include "autobot/solver/PhysicsValidationHarness.hpp"
#include "autobot/solver/SolverTypes.hpp"
#include "autobot/solver/TrajectoryScorer.hpp"
#include "autobot/solver/TrajectorySimulator.hpp"
#include "autobot/world/CollisionWorld.hpp"
#include "autobot/world/DynamicWorldModel.hpp"
#include "autobot/world/TriggerWorldModel.hpp"

namespace autobot::solver {

class RealtimePlanner final {
public:
    [[nodiscard]] PlanDecision plan(
        core::GameSnapshot const& snapshot,
        world::CollisionWorld const& collisionWorld,
        PhysicsValidationHarness const& validation,
        bool botHolding,
        world::DynamicWorldModel const& dynamicWorld,
        world::TriggerWorldModel const* triggerWorld = nullptr,
        bool botHoldingP2 = false,
        PhysicsValidationHarness const* p2Validation = nullptr
    ) const;

private:
    [[nodiscard]] static control::InputAction firstInput(
        ActionCandidate const& candidate,
        bool botHolding
    );
    [[nodiscard]] static std::size_t horizonTicks(
        core::GameSnapshot const& snapshot,
        LocalWorldView const& local,
        ModeCalibration const& calibration,
        double requiredForwardDistance,
        SearchBudget const& budget
    );

    LocalWorldViewBuilder m_localWorldBuilder{};
    GeneralWorldModelBuilder m_generalWorldBuilder{};
    GlobalPlanner m_globalPlanner{};
    ModeActionGenerator m_actionGenerator{};
    TrajectorySimulator m_simulator{};
    TrajectoryScorer m_scorer{};
    DualJointPlanner m_dualPlanner{};
};

} // namespace autobot::solver
