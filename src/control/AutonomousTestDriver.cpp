#include "autobot/control/AutonomousTestDriver.hpp"

#include <algorithm>

namespace autobot::control {

char const* toString(InputOwnership value) {
    switch (value) {
        case InputOwnership::User: return "USER";
        case InputOwnership::Bot: return "BOT";
        case InputOwnership::None: return "NONE";
    }
    return "NONE";
}

char const* toString(InputAction value) {
    switch (value) {
        case InputAction::NoPress: return "NO PRESS";
        case InputAction::Press: return "PRESS";
        case InputAction::Hold: return "HOLD";
        case InputAction::Release: return "RELEASE";
        case InputAction::SafeStop: return "SAFE STOP";
    }
    return "SAFE STOP";
}


AutonomousTestDriver::HoldTransition AutonomousTestDriver::firstHoldTransition(
    solver::ActionCandidate const& candidate,
    bool botHolding
) {
    HoldTransition result{};
    bool previous = botHolding;
    std::size_t tick = 0;

    for (auto const& segment : candidate.segments) {
        const bool desired = segment.hold;
        if (desired != previous) {
            result.valid = true;
            result.delay = tick;
            result.targetHold = desired;
            return result;
        }
        tick += static_cast<std::size_t>(segment.ticks);
        previous = desired;
    }
    return result;
}

std::size_t AutonomousTestDriver::findCountdownTrajectory(
    solver::PlanDecision const& plan,
    bool botHolding,
    std::size_t dueIn,
    bool targetHold
) {
    for (std::size_t i = 0; i < plan.trajectories.size(); ++i) {
        auto const& trajectory = plan.trajectories[i];
        if (trajectory.fatalCollision || !trajectory.horizonConclusive) continue;
        const auto transition = firstHoldTransition(trajectory.candidate, botHolding);
        if (transition.valid
            && transition.delay == dueIn
            && transition.targetHold == targetHold) {
            return i;
        }
    }
    return std::numeric_limits<std::size_t>::max();
}

void AutonomousTestDriver::selectTrajectoryForCurrentSample(
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

void AutonomousTestDriver::clearActionCountdown() {
    m_actionCountdown = {};
}

void AutonomousTestDriver::applyActionCountdown(
    core::GameSnapshot const& snapshot,
    bool botHolding,
    AutonomousDecision& decision
) {
    decision.countdown = {};
    if (!snapshot.valid || snapshot.player.dead || !decision.plan.active) {
        clearActionCountdown();
        return;
    }

    const bool hadPending = m_actionCountdown.active;
    if (m_actionCountdown.active) {
        const auto dueIn = snapshot.solverSampleID >= m_actionCountdown.targetSampleID
            ? std::size_t{0}
            : static_cast<std::size_t>(m_actionCountdown.targetSampleID - snapshot.solverSampleID);
        const auto committedIndex = findCountdownTrajectory(
            decision.plan,
            botHolding,
            dueIn,
            m_actionCountdown.targetHold
        );

        if (committedIndex != std::numeric_limits<std::size_t>::max()) {
            selectTrajectoryForCurrentSample(decision.plan, committedIndex, botHolding);
            decision.action = decision.plan.inputAction;
            decision.reason = decision.plan.reason;
            decision.countdown.targetSampleID = m_actionCountdown.targetSampleID;
            decision.countdown.dueIn = dueIn;
            decision.countdown.label = decision.plan.trajectories[committedIndex].candidate.label;

            if (dueIn == 0) {
                decision.countdown.fired = decision.action == InputAction::Press
                    || decision.action == InputAction::Release;
                clearActionCountdown();
            } else {
                decision.countdown.active = true;
                decision.reason += " COUNTDOWN DUE IN " + std::to_string(dueIn);
            }
            return;
        }

        // A committed countdown must never slide forward silently. If the exact
        // remaining action no longer exists as a conclusive non-fatal candidate,
        // cancel it and use the fresh planner result instead of re-arming the same
        // relative delay on this sample.
        clearActionCountdown();
    }

    if (hadPending) return;
    if (decision.plan.selectedTrajectory >= decision.plan.trajectories.size()) return;

    auto const& selected = decision.plan.trajectories[decision.plan.selectedTrajectory];
    const auto transition = firstHoldTransition(selected.candidate, botHolding);
    if (!transition.valid || transition.delay == 0) return;

    m_actionCountdown.active = true;
    m_actionCountdown.targetSampleID = snapshot.solverSampleID + transition.delay;
    m_actionCountdown.targetHold = transition.targetHold;
    m_actionCountdown.originLabel = selected.candidate.label;

    decision.countdown.active = true;
    decision.countdown.targetSampleID = m_actionCountdown.targetSampleID;
    decision.countdown.dueIn = transition.delay;
    decision.countdown.label = selected.candidate.label;
    decision.reason += " COUNTDOWN DUE IN " + std::to_string(transition.delay);
}

SolverModelTruthSample AutonomousTestDriver::makeTruthSample(
    core::GameSnapshot const& snapshot,
    solver::PlanDecision const& plan
) {
    SolverModelTruthSample sample{};
    sample.solverSampleID = snapshot.solverSampleID;
    sample.levelTime = snapshot.levelTime;
    sample.player = snapshot.player;
    sample.diagnostics = plan.modelTruth;
    sample.selectedTrajectory = plan.selectedTrajectory;
    sample.selectedInput = plan.inputAction;
    sample.candidates.reserve(plan.trajectories.size());

    for (auto const& trajectory : plan.trajectories) {
        CandidateModelTruthTrace trace{};
        trace.label = trajectory.candidate.label;
        trace.classification = trajectory.classification;
        trace.fatalCollision = trajectory.fatalCollision;
        trace.hazardCollision = trajectory.hazardCollision;
        trace.horizonConclusive = trajectory.horizonConclusive;
        trace.portalCrossed = trajectory.portalCrossed;
        trace.uncertainGeometry = trajectory.uncertainGeometry;
        trace.nearestHazardSeen = trajectory.nearestHazardSeen;
        trace.collisionPrimitive = trajectory.collisionPrimitive;
        trace.collisionTick = trajectory.collisionTick;
        trace.minimumClearance = trajectory.minimumClearance;
        trace.progress = trajectory.progress;
        trace.confidence = trajectory.confidence;
        trace.score = trajectory.score;
        trace.finalX = trajectory.predictedFinalX;
        trace.finalY = trajectory.predictedFinalY;
        sample.candidates.push_back(std::move(trace));
    }

    if (plan.selectedTrajectory < sample.candidates.size()) {
        auto const& selected = sample.candidates[plan.selectedTrajectory];
        sample.selectedLabel = selected.label;
        sample.predictedDeath = selected.fatalCollision;
    }
    return sample;
}

void AutonomousTestDriver::pushTruthSample(SolverModelTruthSample sample) {
    constexpr std::size_t kHistoryLimit = 120;
    m_truthHistory.push_back(std::move(sample));
    while (m_truthHistory.size() > kHistoryLimit) m_truthHistory.pop_front();
}

DeathCausalSnapshot AutonomousTestDriver::buildDeathSnapshot(
    core::GameSnapshot const& snapshot
) {
    DeathCausalSnapshot report{};
    report.valid = true;
    report.deathSampleID = snapshot.solverSampleID;
    report.history.assign(m_truthHistory.begin(), m_truthHistory.end());

    for (auto it = m_truthHistory.rbegin(); it != m_truthHistory.rend(); ++it) {
        auto const& sample = *it;
        if (sample.selectedTrajectory >= sample.candidates.size()) continue;
        auto const& selected = sample.candidates[sample.selectedTrajectory];

        const double elapsed = snapshot.levelTime - sample.levelTime;
        const double horizon = sample.diagnostics.predictedHorizonSeconds;
        if (elapsed < -0.0001 || (horizon > 0.0 && elapsed > horizon + 0.05)) continue;

        report.lastSelectedInput = sample.selectedInput;
        report.lastSelectedLabel = selected.label;
        report.lastSelectedClass = selected.classification;
        report.predictedDeath = selected.fatalCollision;

        const bool modelledHazard = sample.diagnostics.hazardPrimitiveExists
            && sample.diagnostics.hazardLocalWorldContains
            && sample.diagnostics.hazardBruteForceContains
            && selected.nearestHazardSeen;
        const bool noPortalBranch = !selected.portalCrossed;
        report.falseSafe = !selected.fatalCollision
            && selected.horizonConclusive
            && modelledHazard
            && noPortalBranch;
        break;
    }

    if (report.falseSafe) ++m_falseSafeTotal;
    return report;
}

AutonomousDecision AutonomousTestDriver::decide(
    core::GameSnapshot const& snapshot,
    world::CollisionWorld const& collisionWorld,
    world::CollisionQueryResult const& query,
    bool enabled,
    bool botHolding
) {
    (void)query;

    AutonomousDecision decision{};
    decision.enabled = enabled;
    decision.falseSafeTotal = m_falseSafeTotal;

    const bool deathTransition = snapshot.valid
        && snapshot.player.dead
        && m_previousSnapshotValid
        && !m_previousDead;
    if (deathTransition) {
        decision.deathSnapshot = buildDeathSnapshot(snapshot);
        decision.falseSafeDetected = decision.deathSnapshot.falseSafe;
        decision.falseSafeTotal = m_falseSafeTotal;
    }

    if (m_hasPredictedNextState && snapshot.valid) {
        m_validation.recordPrediction(m_predictedNextState, snapshot);
    }

    m_validation.observe(snapshot, m_previousAction, m_previousDesiredHold);

    if (!enabled) {
        decision.active = false;
        decision.ownership = InputOwnership::User;
        decision.action = InputAction::SafeStop;
        decision.reason = "AUTOBOT DISABLED";
        m_previousAction = InputAction::SafeStop;
        m_previousDesiredHold = false;
        m_hasPredictedNextState = false;
        clearActionCountdown();
        m_previousSnapshotValid = snapshot.valid;
        m_previousDead = snapshot.valid && snapshot.player.dead;
        return decision;
    }

    decision.plan = m_planner.plan(
        snapshot,
        collisionWorld,
        m_validation,
        botHolding
    );

    decision.action = decision.plan.inputAction;
    decision.reason = decision.plan.reason;
    decision.targetPrimitiveIndex = decision.plan.targetPrimitiveIndex;
    decision.targetObjectID = decision.plan.targetObjectID;
    decision.targetDistance = decision.plan.targetDistance;
    decision.active = decision.plan.active;
    decision.ownership = decision.active ? InputOwnership::Bot : InputOwnership::None;

    applyActionCountdown(snapshot, botHolding, decision);

    if (snapshot.valid && !snapshot.player.dead) {
        pushTruthSample(makeTruthSample(snapshot, decision.plan));
    }

    m_previousAction = decision.action;
    m_previousDesiredHold = decision.action == InputAction::Press
        || decision.action == InputAction::Hold;

    if (decision.plan.hasPredictedNextState) {
        m_predictedNextState = decision.plan.predictedNextState;
        m_hasPredictedNextState = true;
    } else {
        m_hasPredictedNextState = false;
    }

    if (snapshot.player.dead) {
        m_previousAction = InputAction::SafeStop;
        m_previousDesiredHold = false;
        m_hasPredictedNextState = false;
        clearActionCountdown();
    }

    m_previousSnapshotValid = snapshot.valid;
    m_previousDead = snapshot.valid && snapshot.player.dead;
    decision.falseSafeTotal = m_falseSafeTotal;
    return decision;
}

} // namespace autobot::control
