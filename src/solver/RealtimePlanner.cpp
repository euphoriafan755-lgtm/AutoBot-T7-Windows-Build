#include "autobot/solver/RealtimePlanner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace autobot::solver {
namespace {

struct RectEdges {
    double left;
    double right;
    double bottom;
    double top;
};

RectEdges edges(world::WorldRect const& r) {
    const double x2 = static_cast<double>(r.x + r.width);
    const double y2 = static_cast<double>(r.y + r.height);
    return {
        std::min<double>(r.x, x2),
        std::max<double>(r.x, x2),
        std::min<double>(r.y, y2),
        std::max<double>(r.y, y2),
    };
}

bool rectIntersects(world::WorldRect const& a, world::WorldRect const& b) {
    const auto ae = edges(a);
    const auto be = edges(b);
    return ae.left <= be.right && ae.right >= be.left
        && ae.bottom <= be.top && ae.top >= be.bottom;
}

double forwardDistance(world::WorldRect const& rect, double x, double direction) {
    const auto e = edges(rect);
    if (direction >= 0.0) {
        if (e.right < x) return -1.0;
        return std::max(0.0, e.left - x);
    }
    if (e.left > x) return -1.0;
    return std::max(0.0, x - e.right);
}

double forwardExtent(world::WorldRect const& rect) {
    const auto e = edges(rect);
    return std::max(0.0, e.right - e.left);
}

} // namespace

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
    ModeCalibration const& calibration,
    double requiredForwardDistance,
    SearchBudget const& budget
) {
    const double normalizedDt = calibration.normalizedStepDt();
    if (normalizedDt <= 0.0) return 0;
    const double distancePerSample = std::max(
        std::abs(snapshot.player.velocityX) * normalizedDt,
        0.01
    );

    double distance = requiredForwardDistance;
    if (distance <= 0.0) {
        distance = std::min(local.horizonX, distancePerSample * 96.0);
    }
    const double raw = distance / distancePerSample;
    return static_cast<std::size_t>(std::clamp(
        std::ceil(raw),
        static_cast<double>(budget.horizonMin),
        static_cast<double>(budget.horizonMax)
    ));
}

PlanDecision RealtimePlanner::plan(
    core::GameSnapshot const& snapshot,
    world::CollisionWorld const& collisionWorld,
    PhysicsValidationHarness const& validation,
    bool botHolding,
    world::DynamicWorldModel const& dynamicWorld,
    world::TriggerWorldModel const* triggerWorld,
    bool botHoldingP2,
    PhysicsValidationHarness const* p2Validation
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
    if (snapshot.dualMode && (!snapshot.player2Valid || snapshot.player2.mode == core::GameMode::Unknown)) {
        decision.status = SolverStatus::Stopped;
        decision.reason = "DUAL STATE INCOMPLETE";
        decision.inputAction = control::InputAction::SafeStop;
        decision.p2InputAction = control::InputAction::SafeStop;
        finish();
        return decision;
    }

    const auto& calibration = validation.calibration(snapshot.player.mode);
    decision.physicsReady = validation.modeReady(snapshot.player.mode);
    decision.modelTruth.realDtSeconds = calibration.sampleDt;
    decision.modelTruth.simDtNormalized = calibration.normalizedStepDt();
    decision.modelTruth.physicsTicksPerSecond = calibration.physicsTicksPerSecond;
    decision.modelTruth.rawVelocityX = snapshot.player.velocityX;
    decision.modelTruth.observedWorldVelocityX = calibration.observedWorldVelocityX;
    decision.modelTruth.observedWorldVelocityY = calibration.observedWorldVelocityY;
    decision.modelTruth.verticalPositionScale = calibration.yPositionScale();

    if (!decision.physicsReady) {
        decision.reason = "AUTOBOT WAITING FOR PHYSICS UNIT CALIBRATION";
        decision.inputAction = control::InputAction::SafeStop;
        finish();
        return decision;
    }

    const auto local = m_localWorldBuilder.build(snapshot, collisionWorld);
    decision.modelTruth.localQueryRect = local.queryRect;
    if (!local.complete) {
        decision.reason = "AUTOBOT WAITING FOR LOCAL WORLD";
        decision.inputAction = control::InputAction::SafeStop;
        finish();
        return decision;
    }

    auto generalWorld = m_generalWorldBuilder.build(
        snapshot, collisionWorld, 1800.0, 1000.0
    );
    auto searchBudget = AdaptiveSearchBudget::choose(
        snapshot, local, generalWorld, decision.lastModelError
    );
    if (searchBudget.globalLookaheadX > generalWorld.lookaheadX + 1.0
        || searchBudget.globalLookaheadY > generalWorld.lookaheadY + 1.0) {
        generalWorld = m_generalWorldBuilder.build(
            snapshot,
            collisionWorld,
            searchBudget.globalLookaheadX,
            searchBudget.globalLookaheadY
        );
        searchBudget = AdaptiveSearchBudget::choose(
            snapshot, local, generalWorld, decision.lastModelError
        );
    }

    const auto globalRoute = m_globalPlanner.plan(
        snapshot, generalWorld, collisionWorld, searchBudget
    );
    decision.globalPlan.valid = globalRoute.valid;
    decision.globalPlan.uncertain = globalRoute.uncertain;
    decision.globalPlan.targetX = globalRoute.targetX;
    decision.globalPlan.targetY = globalRoute.targetY;
    decision.globalPlan.targetYMin = globalRoute.targetYMin;
    decision.globalPlan.targetYMax = globalRoute.targetYMax;
    decision.globalPlan.plannedForwardDistance = globalRoute.plannedForwardDistance;
    decision.globalPlan.minimumCorridorClearance = globalRoute.minimumCorridorClearance;
    decision.globalPlan.observedComplexity = searchBudget.complexity;
    decision.globalPlan.globalLookaheadX = searchBudget.globalLookaheadX;
    decision.globalPlan.globalLookaheadY = searchBudget.globalLookaheadY;
    decision.globalPlan.routeSteps = globalRoute.steps.size();
    decision.globalPlan.exploredStates = globalRoute.exploredStates;
    decision.globalPlan.branchesConsidered = globalRoute.branchesConsidered;
    decision.globalPlan.nextPortalPrimitive = globalRoute.nextPortalPrimitive;

    if (snapshot.dualMode) {
        decision.dualMode = true;
        if (!p2Validation || !p2Validation->modeReady(snapshot.player2.mode)) {
            decision.physicsReady = false;
            decision.reason = "AUTOBOT WAITING FOR P2 PHYSICS UNIT CALIBRATION";
            decision.inputAction = control::InputAction::SafeStop;
            decision.p2InputAction = control::InputAction::SafeStop;
            finish();
            return decision;
        }
        auto joint = m_dualPlanner.plan(
            snapshot, collisionWorld, validation, *p2Validation, botHolding, botHoldingP2,
            dynamicWorld, triggerWorld, searchBudget
        );
        decision.physicsReady = validation.modeReady(snapshot.player.mode)
            && p2Validation->modeReady(snapshot.player2.mode);
        decision.plannerReady = joint.ready;
        decision.active = joint.active;
        decision.inputAction = joint.p1Action;
        decision.p2InputAction = joint.p2Action;
        decision.trajectories = std::move(joint.p1Trajectories);
        decision.p2Trajectories = std::move(joint.p2Trajectories);
        decision.selectedTrajectory = joint.selectedP1;
        decision.selectedP2Trajectory = joint.selectedP2;
        decision.jointPairsEvaluated = joint.pairs.size();
        decision.hasPredictedNextState = joint.hasPredictedP1;
        decision.predictedNextState = joint.predictedP1;
        decision.hasPredictedNextStateP2 = joint.hasPredictedP2;
        decision.predictedNextStateP2 = joint.predictedP2;
        if (joint.active) {
            decision.status = SolverStatus::Active;
            decision.reason = "DUAL JOINT PLAN";
        } else if (joint.ready) {
            decision.status = SolverStatus::NoSafePath;
            decision.reason = "DUAL NO JOINT SAFE PATH";
        } else {
            decision.status = SolverStatus::Waiting;
            decision.reason = "DUAL JOINT PLANNER WAITING";
        }
        finish();
        return decision;
    }

    const double direction = snapshot.player.velocityX < -0.001 ? -1.0 : 1.0;
    const double distancePerSample = std::max(
        std::abs(snapshot.player.velocityX) * calibration.normalizedStepDt(),
        0.01
    );

    // Independent brute-force nearest-hazard scan inside the exact LocalWorld
    // query rectangle. This is the runtime cross-check against SpatialHash.
    std::size_t bruteHazard = world::kInvalidPrimitiveIndex;
    double bruteDistance = std::numeric_limits<double>::infinity();
    auto const& primitives = collisionWorld.primitives();
    for (std::size_t i = 0; i < primitives.size(); ++i) {
        auto const& primitive = primitives[i];
        if (primitive.classification != world::GameplayObjectType::Hazard
            || !primitive.enabled
            || !primitive.indexable
            || !rectIntersects(local.queryRect, primitive.broadphaseBounds)) {
            continue;
        }
        const double fwd = forwardDistance(
            primitive.broadphaseBounds,
            snapshot.player.x,
            direction
        );
        if (fwd >= 0.0 && fwd < bruteDistance) {
            bruteDistance = fwd;
            bruteHazard = i;
        }
    }

    std::size_t localHazard = world::kInvalidPrimitiveIndex;
    if (!local.hazards.empty()) localHazard = local.hazards.front().primitiveIndex;

    const std::size_t truthHazard = bruteHazard != world::kInvalidPrimitiveIndex
        ? bruteHazard
        : localHazard;

    double requiredForwardDistance = 0.0;
    if (truthHazard != world::kInvalidPrimitiveIndex && truthHazard < primitives.size()) {
        auto const& hazard = primitives[truthHazard];
        auto& truth = decision.modelTruth;
        truth.nearestHazardPrimitive = truthHazard;
        truth.nearestHazardSourceIndex = hazard.sourceIndex;
        truth.nearestHazardObjectID = hazard.objectID;
        truth.nearestHazardUniqueID = hazard.sourceUniqueID;
        truth.nearestHazardBounds = hazard.broadphaseBounds;
        truth.nearestHazardGeometry = hazard.geometryVerification;
        truth.nearestHazardDistance = forwardDistance(
            hazard.broadphaseBounds,
            snapshot.player.x,
            direction
        );
        truth.hazardExistsInCollisionWorld = true;
        truth.hazardClassifiedHazard = hazard.classification == world::GameplayObjectType::Hazard;
        truth.hazardPrimitiveExists = true;
        truth.hazardHashIndexed = collisionWorld.primitiveIndexed(truthHazard);
        truth.hazardInsideLocalQueryRect = rectIntersects(local.queryRect, hazard.broadphaseBounds);
        truth.hazardLocalWorldContains = std::any_of(
            local.hazards.begin(),
            local.hazards.end(),
            [&](LocalWorldObject const& object) { return object.primitiveIndex == truthHazard; }
        );
        truth.hazardBruteForceContains = bruteHazard == truthHazard;
        truth.hazardEnabled = hazard.enabled;
        truth.hazardIndexable = hazard.indexable;
        truth.hazardNoTouch = hazard.noTouch;
        truth.hazardHashCellCount = collisionWorld.cellsForPrimitive(truthHazard).size();

        if (truth.nearestHazardDistance >= 0.0 && std::isfinite(truth.nearestHazardDistance)) {
            truth.samplesToHazard = truth.nearestHazardDistance / distancePerSample;
            truth.timeToHazardSeconds = truth.samplesToHazard * calibration.sampleDt;
        }

        // The trajectory must pass the hazard, the player's leading half-width,
        // and a 12-sample recovery window before it can be called conclusive.
        constexpr double kRecoverySamples = 12.0;
        requiredForwardDistance = std::max(0.0, truth.nearestHazardDistance)
            + forwardExtent(hazard.broadphaseBounds)
            + snapshot.player.objectBoundsWidth * 0.5
            + distancePerSample * kRecoverySamples;

        // If brute-force sees the hazard but LocalWorld/SpatialHash omitted it,
        // do not plan on incomplete geometry.
        if (truth.hazardBruteForceContains && !truth.hazardLocalWorldContains) {
            decision.status = SolverStatus::Stopped;
            decision.reason = "MODEL TRUTH QUERY MISMATCH - HAZARD MISSING FROM LOCAL WORLD";
            decision.inputAction = control::InputAction::SafeStop;
            finish();
            return decision;
        }
    }

    if (globalRoute.valid) {
        const double targetForward = direction * (globalRoute.targetX - snapshot.player.x);
        if (targetForward > 0.0) {
            // Global guidance influences how far the local MPC must prove the
            // future, but never extends beyond the currently modelled local
            // collision window.
            const double globalRequirement = std::min(
                targetForward + distancePerSample * 6.0,
                local.horizonX * 0.95
            );
            requiredForwardDistance = std::max(requiredForwardDistance, globalRequirement);
        }
    }

    decision.modelTruth.requiredForwardDistance = requiredForwardDistance;
    const auto horizon = horizonTicks(
        snapshot, local, calibration, requiredForwardDistance, searchBudget
    );
    decision.modelTruth.horizonTicks = horizon;
    decision.modelTruth.predictedHorizonSeconds = horizon * calibration.sampleDt;
    if (horizon == 0) {
        decision.reason = "AUTOBOT WAITING FOR HORIZON";
        decision.inputAction = control::InputAction::SafeStop;
        finish();
        return decision;
    }

    const auto generationStart = Clock::now();
    auto candidates = m_actionGenerator.generate(snapshot, horizon, searchBudget);
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
    bool anyConclusiveNonFatal = false;
    bool anyRouteCompatibleNonFatal = false;
    int bestPriority = -1;

    for (auto const& candidate : candidates) {
        const auto simulationStart = Clock::now();
        auto trajectory = m_simulator.simulate(
            snapshot,
            collisionWorld,
            local,
            validation,
            candidate,
            horizon,
            botHolding,
            requiredForwardDistance,
            &dynamicWorld,
            triggerWorld
        );
        decision.physicsSimulationMs += std::chrono::duration<double, std::milli>(
            Clock::now() - simulationStart
        ).count();

        const auto scoringStart = Clock::now();
        const double score = m_scorer.score(trajectory);
        decision.trajectoryScoringMs += std::chrono::duration<double, std::milli>(
            Clock::now() - scoringStart
        ).count();

        if (!trajectory.fatalCollision && trajectory.horizonConclusive) {
            anyConclusiveNonFatal = true;
        }

        if (globalRoute.valid
            && !trajectory.fatalCollision
            && trajectory.horizonConclusive
            && !trajectory.points.empty()) {
            auto const& probe = trajectory.points.back();
            if (probe.y < globalRoute.targetYMin) {
                trajectory.globalRouteErrorY = globalRoute.targetYMin - probe.y;
            } else if (probe.y > globalRoute.targetYMax) {
                trajectory.globalRouteErrorY = probe.y - globalRoute.targetYMax;
            } else {
                trajectory.globalRouteErrorY = 0.0;
                trajectory.globalRouteCompatible = true;
            }
        } else if (!globalRoute.valid) {
            trajectory.globalRouteCompatible = true;
            trajectory.globalRouteErrorY = 0.0;
        }

        if (!trajectory.fatalCollision
            && trajectory.horizonConclusive
            && trajectory.globalRouteCompatible) {
            anyRouteCompatibleNonFatal = true;
        }

        const auto index = decision.trajectories.size();
        decision.trajectories.push_back(std::move(trajectory));

        auto const& ranked = decision.trajectories[index];
        int priority = 0;
        if (!ranked.fatalCollision && ranked.horizonConclusive) {
            priority = ranked.globalRouteCompatible ? 3 : 2;
        } else if (!ranked.fatalCollision) {
            priority = 1;
        }
        if (bestIndex == std::numeric_limits<std::size_t>::max()
            || priority > bestPriority
            || (priority == bestPriority && score > bestScore)) {
            bestPriority = priority;
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
    decision.confidence = selected.confidence;
    decision.plannerReady = true;

    if (!selected.horizonConclusive && !selected.fatalCollision) {
        decision.status = SolverStatus::Stopped;
        decision.reason = "HORIZON INCONCLUSIVE - REFUSING FALSE SAFE";
        decision.inputAction = control::InputAction::SafeStop;
        decision.active = false;
        finish();
        return decision;
    }

    decision.inputAction = firstInput(selected.candidate, botHolding);
    decision.active = true;

    if (anyConclusiveNonFatal && !selected.fatalCollision
        && (!globalRoute.valid || anyRouteCompatibleNonFatal)) {
        decision.status = SolverStatus::Active;
        decision.reason = selected.candidate.label;
    } else if (globalRoute.valid && anyConclusiveNonFatal && !selected.fatalCollision) {
        decision.status = SolverStatus::NoSafePath;
        decision.reason = "GLOBAL ROUTE UNRESOLVED - EXECUTING LEAST-BAD LEGAL ACTION";
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
