#pragma once

#include "autobot/control/InputController.hpp"
#include "autobot/solver/SolverTypes.hpp"

namespace autobot::control {

bool applyGlobalPolicyGuidance(
    solver::PlanDecision& plan,
    bool globalDesiredHold,
    bool botHolding
);

} // namespace autobot::control
