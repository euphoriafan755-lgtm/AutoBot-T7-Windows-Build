#pragma once

#include "autobot/control/InputController.hpp"
#include "autobot/core/GameSnapshot.hpp"
#include "autobot/solver/PhysicsValidationHarness.hpp"
#include "autobot/solver/RealtimePlanner.hpp"
#include "autobot/world/CollisionWorld.hpp"

#include <cstddef>
#include <string>

namespace autobot::control {

struct AutonomousDecision {
    bool enabled = false;
    bool active = false;
    InputOwnership ownership = InputOwnership::User;
    InputAction action = InputAction::NoPress;
    std::string reason = "AUTOBOT DISABLED";
    std::size_t targetPrimitiveIndex = world::kInvalidPrimitiveIndex;
    int targetObjectID = 0;
    double targetDistance = 0.0;

    solver::PlanDecision plan{};
};

class AutonomousTestDriver final {
public:
    [[nodiscard]] AutonomousDecision decide(
        core::GameSnapshot const& snapshot,
        world::CollisionWorld const& collisionWorld,
        world::CollisionQueryResult const& query,
        bool enabled,
        bool botHolding
    );

    [[nodiscard]] solver::PhysicsValidationHarness const& validation() const {
        return m_validation;
    }

private:
    solver::PhysicsValidationHarness m_validation{};
    solver::RealtimePlanner m_planner{};

    InputAction m_previousAction = InputAction::SafeStop;
    bool m_previousDesiredHold = false;
    bool m_hasPredictedNextState = false;
    solver::SimState m_predictedNextState{};
};

[[nodiscard]] char const* toString(InputOwnership value);
[[nodiscard]] char const* toString(InputAction value);

} // namespace autobot::control
