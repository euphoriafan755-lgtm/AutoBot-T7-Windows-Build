#include "autobot/solver/RealtimePlanner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace autobot::solver {

control::InputAction RealtimePlanner::firstInput(
    ActionCandidate const& candidate,
    bool botHolding
) {
    const bool wantHold = candidate.desiredHoldAt(0);
    if (wantHold && !botHolding) return control::InputAction::Press;
    if (wantHold && botHolding) return control::InputAction::Hold;
    if (!wantHold && botHolding) return control::InputAction::Release;
    return control::InputAction::NoPress;
}

std::size_t RealtimePlanner::horizonTicks(
    core::GameSnapshot const& snapshot,
    LocalWorldView const& local,
    ModeCalibration const& calibration
) {
    if (calibration.sampleDt <= 0.0) return 0;
    const double distancePerSample = std::max(
        std::abs(snapshot.player.velocityX) * calibration.sampleDt,
        0.01
    );
    const double raw = local.horizonX / distancePerSample;
    return static_cast<std::size_t>(std::clamp(raw, 24.0, 144.0));
}

PlanDecision RealtimePlanner::plan(
    core::GameSnapshot const& snapshot,
    world::CollisionWorld const& collisionWorld,
    PhysicsValidationHarness const& validation,
    bool botHolding
) const {
    using Clock = std::chrono::steady_clock;
    const auto plannerStart = Clock::now();

    PlanDecision decision{};
    auto finish = [&]() {
        decision.plannerDurationMs = std::chrono::duration<double, std::milli>(
            Clock::now() - plannerStart
        ).count();
    };

    decision.gameStateReady = snapshot.valid && !snapshot.player.dead;
    decision.worldReady = collisionWorld.ready();
    decision.lastModelError = validation.stats().lastError;

    if (!snapshot.valid) {
        decision.reason = "AUTOBOT WAITING FOR GAME STATE";
        decision.inputAction = control::InputAction::SafeStop;
        finish();
        return decision;
    }
    if (snapshot.player.dead) {
        decision.status = SolverStatus::Stopped;
        decision.reason = "PLAYER DEAD - WAITING FOR RESTART";
        decision.inputAction = control::InputAction::SafeStop;
        finish();
        return decision;
    }
    if (!collisionWorld.ready()) {
        decision.reason = "AUTOBOT WAITING FOR WORLD";
        decision.inputAction = control::InputAction::SafeStop;
        finish();
        return decision;
    }
    if (snapshot.player.mode == core::GameMode::Unknown) {
        decision.status = SolverStatus::Stopped;
        decision.reason = "MODEL UNAVAILABLE FOR UNKNOWN MODE";
        decision.inputAction = control::InputAction::SafeStop;
        finish();
        return decision;
    }

    const auto& calibration = validation.calibration(snapshot.player.mode);
    decision.physicsReady = validation.modeReady(snapshot.player.mode);
    if (!decision.physicsReady) {
        decision.reason = "AUTOBOT WAITING FOR PHYSICS MODEL SAMPLE";
        decision.inputAction = control::InputAction::SafeStop;
        finish();
        return decision;
    }

    const auto local = m_localWorldBuilder.build(snapshot, collisionWorld);
    if (!local.complete) {
        decision.reason = "AUTOBOT WAITING FOR LOCAL WORLD";
        decision.inputAction = control::InputAction::SafeStop;
        finish();
        return decision;
    }

    const auto horizon = horizonTicks(snapshot, local, calibration);
    if (horizon == 0) {
        decision.reason = "AUTOBOT WAITING FOR HORIZON";
        decision.inputAction = control::InputAction::SafeStop;
        finish();
        return decision;
    }

    const auto generationStart = Clock::now();
    auto candidates = m_actionGenerator.generate(snapshot, horizon);
    decision.candidateGenerationMs = std::chrono::duration<double, std::milli>(
        Clock::now() - generationStart
    ).count();

    if (candidates.empty()) {
        decision.status = SolverStatus::Stopped;
        decision.reason = "NO ACTION CANDIDATES";
        decision.inputAction = control::InputAction::SafeStop;
        finish();
        return decision;
    }

    decision.trajectories.reserve(candidates.size());
    std::size_t bestIndex = std::numeric_limits<std::size_t>::max();
    double bestScore = -std::numeric_limits<double>::infinity();
    bool anyNonFatal = false;

    for (auto const& candidate : candidates) {
        const auto simulationStart = Clock::now();
        auto trajectory = m_simulator.simulate(
            snapshot,
            collisionWorld,
            local,
            validation,
            candidate,
            horizon
        );
        decision.physicsSimulationMs += std::chrono::duration<double, std::milli>(
            Clock::now() - simulationStart
        ).count();

        const auto scoringStart = Clock::now();
        const double score = m_scorer.score(trajectory);
        decision.trajectoryScoringMs += std::chrono::duration<double, std::milli>(
            Clock::now() - scoringStart
        ).count();

        if (!trajectory.fatalCollision) anyNonFatal = true;

        const auto index = decision.trajectories.size();
        decision.trajectories.push_back(std::move(trajectory));

        if (score > bestScore) {
            bestScore = score;
            bestIndex = index;
        }
    }

    if (bestIndex == std::numeric_limits<std::size_t>::max()) {
        decision.status = SolverStatus::NoSafePath;
        decision.reason = "NO SAFE PATH";
        decision.inputAction = control::InputAction::SafeStop;
        finish();
        return decision;
    }

    decision.selectedTrajectory = bestIndex;
    auto const& selected = decision.trajectories[bestIndex];
    decision.inputAction = firstInput(selected.candidate, botHolding);
    decision.confidence = selected.confidence;
    decision.plannerReady = true;
    decision.active = true;

    if (anyNonFatal && !selected.fatalCollision) {
        decision.status = SolverStatus::Active;
        decision.reason = selected.candidate.label;
    } else {
        decision.status = SolverStatus::NoSafePath;
        decision.reason = "NO SAFE PATH - EXECUTING LEAST-BAD LEGAL ACTION";
    }

    auto chooseTarget = [&](auto const& list) {
        if (list.empty()) return;
        auto const& target = list.front();
        if (target.primitiveIndex >= collisionWorld.primitives().size()) return;
        decision.targetPrimitiveIndex = target.primitiveIndex;
        decision.targetObjectID = collisionWorld.primitives()[target.primitiveIndex].objectID;
        decision.targetDistance = target.forwardDistance;
    };
    if (!local.hazards.empty()) chooseTarget(local.hazards);
    else if (!local.solids.empty()) chooseTarget(local.solids);
    else if (!local.portals.empty()) chooseTarget(local.portals);

    if (selected.points.size() > 1) {
        auto const& p = selected.points[1];
        decision.hasPredictedNextState = true;
        decision.predictedNextState.x = p.x;
        decision.predictedNextState.y = p.y;
        decision.predictedNextState.vx = p.vx;
        decision.predictedNextState.vy = p.vy;
        decision.predictedNextState.mode = p.mode;
        decision.predictedNextState.mini = snapshot.player.mini;
        decision.predictedNextState.grounded = p.landing;
        decision.predictedNextState.upsideDown = snapshot.player.upsideDown;
        decision.predictedNextState.holding = selected.candidate.desiredHoldAt(0);
        decision.predictedNextState.alive = !p.collision;
    }

    finish();
    return decision;
}

} // namespace autobot::solver
