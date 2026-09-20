#pragma once

#include "autobot/control/InputController.hpp"
#include "autobot/solver/SolverTypes.hpp"

#include <limits>

namespace autobot::control {
namespace detail {

inline void selectGlobalPolicyTrajectory(
    solver::PlanDecision& plan,
    std::size_t trajectoryIndex,
    bool botHolding
) {
    if (trajectoryIndex >= plan.trajectories.size()) return;

    plan.selectedTrajectory = trajectoryIndex;
    auto const& selected = plan.trajectories[trajectoryIndex];
    const bool wantHold = selected.candidate.desiredHoldAt(0);

    if (wantHold && !botHolding) plan.inputAction = InputAction::Press;
    else if (wantHold && botHolding) plan.inputAction = InputAction::Hold;
    else if (!wantHold && botHolding) plan.inputAction = InputAction::Release;
    else plan.inputAction = InputAction::NoPress;

    plan.confidence = selected.confidence;
    plan.status = solver::SolverStatus::Active;
    plan.active = true;
    plan.plannerReady = true;
    plan.reason = selected.candidate.label;

    plan.hasPredictedNextState = false;
    if (selected.points.size() > 1) {
        auto const& p = selected.points[1];
        plan.hasPredictedNextState = true;
        plan.predictedNextState.x = p.x;
        plan.predictedNextState.y = p.y;
        plan.predictedNextState.vx = p.vx;
        plan.predictedNextState.vy = p.vy;
        plan.predictedNextState.mode = p.mode;
        plan.predictedNextState.grounded = p.landing;
        plan.predictedNextState.holding = wantHold;
        plan.predictedNextState.alive = !p.collision;
    }
}

} // namespace detail

inline bool applyGlobalPolicyGuidance(
    solver::PlanDecision& plan,
    bool globalDesiredHold,
    bool botHolding
) {
    const InputAction desired = globalDesiredHold
        ? (botHolding ? InputAction::Hold : InputAction::Press)
        : (botHolding ? InputAction::Release : InputAction::NoPress);

    std::size_t best = std::numeric_limits<std::size_t>::max();
    int bestPriority = -1;
    double bestScore = -std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < plan.trajectories.size(); ++i) {
        auto const& trajectory = plan.trajectories[i];
        // Global guidance may only choose a branch the local MPC has already
        // proved immediately executable. It must never revive a false-safe
        // horizon or override a local fatality veto.
        if (trajectory.fatalCollision || !trajectory.horizonConclusive) continue;

        const bool wantHold = trajectory.candidate.desiredHoldAt(0);
        const InputAction candidateAction = wantHold
            ? (botHolding ? InputAction::Hold : InputAction::Press)
            : (botHolding ? InputAction::Release : InputAction::NoPress);
        if (candidateAction != desired) continue;

        int priority = 1;
        if (trajectory.horizonConclusive) ++priority;
        if (trajectory.globalRouteCompatible) ++priority;

        if (best == std::numeric_limits<std::size_t>::max()
            || priority > bestPriority
            || (priority == bestPriority && trajectory.score > bestScore)) {
            best = i;
            bestPriority = priority;
            bestScore = trajectory.score;
        }
    }

    if (best == std::numeric_limits<std::size_t>::max()) return false;

    detail::selectGlobalPolicyTrajectory(plan, best, botHolding);
    plan.reason = "GLOBAL POLICY + LOCAL MPC: " + plan.trajectories[best].candidate.label;
    return true;
}

} // namespace autobot::control
