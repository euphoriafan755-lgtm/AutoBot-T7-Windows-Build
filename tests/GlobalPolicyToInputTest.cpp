#include "autobot/control/AutonomousTestDriver.hpp"
#include "autobot/control/InputController.hpp"

#include <cassert>
#include <iostream>

using namespace autobot;

namespace {

solver::TrajectoryResult trajectory(char const* label, bool hold, bool fatal, double score) {
    solver::TrajectoryResult t{};
    t.candidate.label = label;
    t.candidate.segments.push_back({hold, 8});
    t.fatalCollision = fatal;
    t.horizonConclusive = true;
    t.globalRouteCompatible = true;
    t.score = score;
    t.classification = fatal
        ? solver::TrajectoryClass::Collision
        : solver::TrajectoryClass::Safe;
    return t;
}

} // namespace

int main() {
    solver::PlanDecision plan{};
    plan.gameStateReady = true;
    plan.worldReady = true;
    plan.physicsReady = true;
    plan.plannerReady = true;
    plan.trajectories.push_back(trajectory("LOCAL NO PRESS", false, false, 100.0));
    plan.trajectories.push_back(trajectory("GLOBAL PRESS", true, false, 80.0));

    const bool applied = control::AutonomousTestDriver::applyGlobalPolicyGuidance(
        plan,
        true,
        false
    );
    assert(applied);
    assert(plan.active);
    assert(plan.inputAction == control::InputAction::Press);
    assert(plan.selectedTrajectory == 1);

    const auto transition = control::InputController::transition(false, plan.inputAction);
    assert(transition.emitPress);
    assert(!transition.emitRelease);
    assert(transition.nextHolding);
    assert(transition.effectiveAction == control::InputAction::Press);

    // Local MPC remains authoritative for immediate safety: a fatal branch
    // cannot be forced by global guidance.
    solver::PlanDecision veto{};
    veto.trajectories.push_back(trajectory("FATAL PRESS", true, true, 1000.0));
    assert(!control::AutonomousTestDriver::applyGlobalPolicyGuidance(veto, true, false));

    std::cout
        << "GLOBAL_POLICY_TO_INPUT_TEST=PASS "
        << "global_policy=PRESS local_mpc_compatible=YES "
        << "final_action=PRESS queueButton=PRESS fatal_global_branch_vetoed=YES\n";
    return 0;
}
