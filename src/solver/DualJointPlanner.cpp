#include "autobot/solver/DualJointPlanner.hpp"

#include <algorithm>
#include <cmath>

namespace autobot::solver {

control::InputAction DualJointPlanner::firstInput(
    ActionCandidate const& candidate,
    bool holding
) {
    const bool want = candidate.desiredHoldAt(0);
    if (want && !holding) return control::InputAction::Press;
    if (want && holding) return control::InputAction::Hold;
    if (!want && holding) return control::InputAction::Release;
    return control::InputAction::NoPress;
}

core::GameSnapshot DualJointPlanner::playerSnapshot(
    core::GameSnapshot const& joint,
    bool player2
) {
    auto copy = joint;
    if (player2) copy.player = joint.player2;
    copy.dualMode = false;
    copy.player2Valid = false;
    copy.player2 = {};
    copy.valid = joint.valid && (!player2 || joint.player2Valid);
    return copy;
}

double DualJointPlanner::requiredDistance(
    core::GameSnapshot const& snapshot,
    LocalWorldView const& local,
    ModeCalibration const& calibration,
    std::size_t horizon
) {
    const double perSample = std::max(
        0.01,
        std::abs(snapshot.player.velocityX) * calibration.normalizedStepDt()
    );
    double result = std::min(local.horizonX * 0.90, perSample * static_cast<double>(horizon));
    if (!local.hazards.empty()) {
        auto const& hazard = local.hazards.front();
        const double recovery = perSample * 12.0;
        result = std::max(
            result,
            hazard.forwardDistance + static_cast<double>(hazard.bounds.width)
                + snapshot.player.objectBoundsWidth * 0.5 + recovery
        );
    }
    return std::max(perSample * 16.0, result);
}

bool DualJointPlanner::routeCompatible(
    TrajectoryResult& trajectory,
    GlobalRoutePlan const& route
) {
    if (!route.valid) {
        trajectory.globalRouteCompatible = true;
        trajectory.globalRouteErrorY = 0.0;
        return true;
    }
    if (trajectory.points.empty() || trajectory.fatalCollision) return false;
    const double y = trajectory.points.back().y;
    if (y < route.targetYMin) trajectory.globalRouteErrorY = route.targetYMin - y;
    else if (y > route.targetYMax) trajectory.globalRouteErrorY = y - route.targetYMax;
    else trajectory.globalRouteErrorY = 0.0;
    trajectory.globalRouteCompatible = trajectory.globalRouteErrorY <= 0.001;
    return trajectory.globalRouteCompatible;
}

int DualJointPlanner::safetyTier(
    TrajectoryResult const& p1,
    TrajectoryResult const& p2,
    bool globalCompatible
) {
    if (p1.fatalCollision || p2.fatalCollision) return 0;
    if (p1.horizonConclusive && p2.horizonConclusive) {
        return globalCompatible ? 3 : 2;
    }
    return 1;
}

JointTrajectoryPair DualJointPlanner::selectBestPair(
    std::vector<TrajectoryResult> const& p1,
    std::vector<TrajectoryResult> const& p2
) {
    JointTrajectoryPair best{};
    bool found = false;
    for (std::size_t i = 0; i < p1.size(); ++i) {
        for (std::size_t j = 0; j < p2.size(); ++j) {
            const bool compatible = p1[i].globalRouteCompatible && p2[j].globalRouteCompatible;
            JointTrajectoryPair candidate{};
            candidate.p1Index = i;
            candidate.p2Index = j;
            candidate.fatalCollision = p1[i].fatalCollision || p2[j].fatalCollision;
            candidate.horizonConclusive = p1[i].horizonConclusive && p2[j].horizonConclusive;
            candidate.globalCompatible = compatible;
            candidate.safetyTier = safetyTier(p1[i], p2[j], compatible);
            candidate.score = std::min(p1[i].score, p2[j].score)
                + 0.05 * (p1[i].score + p2[j].score);

            if (!found
                || candidate.safetyTier > best.safetyTier
                || (candidate.safetyTier == best.safetyTier && candidate.score > best.score)) {
                best = candidate;
                found = true;
            }
        }
    }
    return best;
}

DualPlanResult DualJointPlanner::plan(
    core::GameSnapshot const& snapshot,
    world::CollisionWorld const& collisionWorld,
    PhysicsValidationHarness const& p1Validation,
    PhysicsValidationHarness const& p2Validation,
    bool p1Holding,
    bool p2Holding,
    world::DynamicWorldModel const& dynamicWorld,
    world::TriggerWorldModel const* triggerWorld,
    SearchBudget const& budget
) const {
    DualPlanResult result{};
    if (!snapshot.valid || !snapshot.dualMode || !snapshot.player2Valid
        || snapshot.player.dead || snapshot.player2.dead || !collisionWorld.ready()) {
        return result;
    }
    if (!p1Validation.modeReady(snapshot.player.mode)
        || !p2Validation.modeReady(snapshot.player2.mode)) {
        return result;
    }

    auto p1Snapshot = playerSnapshot(snapshot, false);
    auto p2Snapshot = playerSnapshot(snapshot, true);
    const auto p1Local = m_localWorldBuilder.build(p1Snapshot, collisionWorld);
    const auto p2Local = m_localWorldBuilder.build(p2Snapshot, collisionWorld);
    if (!p1Local.complete || !p2Local.complete) return result;

    const auto p1General = m_generalWorldBuilder.build(
        p1Snapshot, collisionWorld, budget.globalLookaheadX, budget.globalLookaheadY
    );
    const auto p2General = m_generalWorldBuilder.build(
        p2Snapshot, collisionWorld, budget.globalLookaheadX, budget.globalLookaheadY
    );
    const auto p1Route = m_globalPlanner.plan(p1Snapshot, p1General, collisionWorld, budget);
    const auto p2Route = m_globalPlanner.plan(p2Snapshot, p2General, collisionWorld, budget);

    const std::size_t horizon = std::clamp<std::size_t>(
        budget.horizonMax,
        std::max<std::size_t>(24, budget.horizonMin),
        512
    );
    const auto& p1Cal = p1Validation.calibration(snapshot.player.mode);
    const auto& p2Cal = p2Validation.calibration(snapshot.player2.mode);
    const double p1Required = requiredDistance(p1Snapshot, p1Local, p1Cal, horizon);
    const double p2Required = requiredDistance(p2Snapshot, p2Local, p2Cal, horizon);

    auto p1Candidates = m_actionGenerator.generate(p1Snapshot, horizon, budget);
    auto p2Candidates = m_actionGenerator.generate(p2Snapshot, horizon, budget);
    if (p1Candidates.empty() || p2Candidates.empty()) return result;

    result.p1Trajectories.reserve(p1Candidates.size());
    for (auto const& candidate : p1Candidates) {
        auto trajectory = m_simulator.simulate(
            p1Snapshot, collisionWorld, p1Local, p1Validation, candidate, horizon,
            p1Holding, p1Required, &dynamicWorld, triggerWorld
        );
        trajectory.score = m_scorer.score(trajectory);
        const bool compatible = routeCompatible(trajectory, p1Route);
        (void)compatible;
        result.p1Trajectories.push_back(std::move(trajectory));
    }

    result.p2Trajectories.reserve(p2Candidates.size());
    for (auto const& candidate : p2Candidates) {
        auto trajectory = m_simulator.simulate(
            p2Snapshot, collisionWorld, p2Local, p2Validation, candidate, horizon,
            p2Holding, p2Required, &dynamicWorld, triggerWorld
        );
        trajectory.score = m_scorer.score(trajectory);
        const bool compatible = routeCompatible(trajectory, p2Route);
        (void)compatible;
        result.p2Trajectories.push_back(std::move(trajectory));
    }

    result.pairs.reserve(result.p1Trajectories.size() * result.p2Trajectories.size());
    for (std::size_t i = 0; i < result.p1Trajectories.size(); ++i) {
        for (std::size_t j = 0; j < result.p2Trajectories.size(); ++j) {
            const auto& a = result.p1Trajectories[i];
            const auto& b = result.p2Trajectories[j];
            JointTrajectoryPair pair{};
            pair.p1Index = i;
            pair.p2Index = j;
            pair.fatalCollision = a.fatalCollision || b.fatalCollision;
            pair.horizonConclusive = a.horizonConclusive && b.horizonConclusive;
            pair.globalCompatible = a.globalRouteCompatible && b.globalRouteCompatible;
            pair.safetyTier = safetyTier(a, b, pair.globalCompatible);
            pair.score = std::min(a.score, b.score) + 0.05 * (a.score + b.score);
            result.pairs.push_back(pair);
        }
    }

    const auto best = selectBestPair(result.p1Trajectories, result.p2Trajectories);
    if (best.p1Index >= result.p1Trajectories.size()
        || best.p2Index >= result.p2Trajectories.size()) {
        return result;
    }

    result.selectedP1 = best.p1Index;
    result.selectedP2 = best.p2Index;
    for (std::size_t k = 0; k < result.pairs.size(); ++k) {
        if (result.pairs[k].p1Index == best.p1Index
            && result.pairs[k].p2Index == best.p2Index) {
            result.selectedPair = k;
            break;
        }
    }
    result.ready = true;
    result.active = best.safetyTier >= 2;
    result.p1Action = firstInput(result.p1Trajectories[best.p1Index].candidate, p1Holding);
    result.p2Action = firstInput(result.p2Trajectories[best.p2Index].candidate, p2Holding);

    auto fillPrediction = [](TrajectoryResult const& trajectory, SimState& state, bool& has) {
        if (trajectory.points.size() <= 1) return;
        auto const& p = trajectory.points[1];
        state.x = p.x;
        state.y = p.y;
        state.vx = p.vx;
        state.vy = p.vy;
        state.mode = p.mode;
        state.grounded = p.landing;
        state.alive = !p.collision;
        has = true;
    };
    fillPrediction(result.p1Trajectories[best.p1Index], result.predictedP1, result.hasPredictedP1);
    fillPrediction(result.p2Trajectories[best.p2Index], result.predictedP2, result.hasPredictedP2);
    return result;
}

} // namespace autobot::solver
