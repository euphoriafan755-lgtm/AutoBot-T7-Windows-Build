#include "autobot/solver/TrajectorySimulator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <vector>

namespace autobot::solver {
namespace {

struct Edges {
    double left;
    double right;
    double bottom;
    double top;
};

Edges edges(world::WorldRect const& r) {
    const double x2 = static_cast<double>(r.x + r.width);
    const double y2 = static_cast<double>(r.y + r.height);
    return {
        std::min<double>(r.x, x2),
        std::max<double>(r.x, x2),
        std::min<double>(r.y, y2),
        std::max<double>(r.y, y2),
    };
}

Edges playerEdges(SimState const& state) {
    const double halfW = std::max(0.5, state.objectWidth * 0.5);
    const double halfH = std::max(0.5, state.objectHeight * 0.5);
    return {
        state.x - halfW,
        state.x + halfW,
        state.y - halfH,
        state.y + halfH,
    };
}

world::WorldRect rectFromEdges(Edges const& e) {
    return {
        static_cast<float>(e.left),
        static_cast<float>(e.bottom),
        static_cast<float>(e.right - e.left),
        static_cast<float>(e.top - e.bottom),
    };
}

bool intersects(Edges const& a, Edges const& b) {
    return a.left <= b.right && a.right >= b.left
        && a.bottom <= b.top && a.top >= b.bottom;
}

double clearance(Edges const& a, Edges const& b) {
    const double dx = std::max({b.left - a.right, a.left - b.right, 0.0});
    const double dy = std::max({b.bottom - a.top, a.bottom - b.top, 0.0});
    return std::hypot(dx, dy);
}

bool canLand(core::GameMode mode) {
    return mode == core::GameMode::Cube
        || mode == core::GameMode::Ball
        || mode == core::GameMode::Robot
        || mode == core::GameMode::Spider;
}

bool resolveSolidContact(
    SimState const& previous,
    SimState& current,
    world::WorldRect const& solidBounds
) {
    if (!canLand(current.mode)) return false;

    const auto solid = edges(solidBounds);
    const auto prevPlayer = playerEdges(previous);
    const auto curPlayer = playerEdges(current);

    if (!current.upsideDown) {
        const bool descending = current.vy <= 0.0;
        const bool crossedTop = prevPlayer.bottom >= solid.top - 1.0
            && curPlayer.bottom <= solid.top + 1.0;
        if (descending && crossedTop) {
            current.y = solid.top + current.objectHeight * 0.5;
            current.vy = 0.0;
            current.grounded = true;
            return true;
        }
    } else {
        const bool ascendingTowardCeiling = current.vy >= 0.0;
        const bool crossedBottom = prevPlayer.top <= solid.bottom + 1.0
            && curPlayer.top >= solid.bottom - 1.0;
        if (ascendingTowardCeiling && crossedBottom) {
            current.y = solid.bottom - current.objectHeight * 0.5;
            current.vy = 0.0;
            current.grounded = true;
            return true;
        }
    }
    return false;
}

std::vector<std::size_t> localPrimitiveIndices(LocalWorldView const& local) {
    std::vector<std::size_t> result;
    result.reserve(
        local.solids.size() + local.hazards.size() + local.portals.size() + local.unknown.size()
    );
    auto append = [&](auto const& objects) {
        for (auto const& object : objects) {
            if (object.primitiveIndex != world::kInvalidPrimitiveIndex) {
                result.push_back(object.primitiveIndex);
            }
        }
    };
    append(local.solids);
    append(local.hazards);
    append(local.portals);
    append(local.unknown);
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}

SimState makeState(core::GameSnapshot const& snapshot, ModeCalibration const& calibration) {
    SimState state{};
    state.x = snapshot.player.x;
    state.y = snapshot.player.y;
    state.vx = snapshot.player.velocityX;
    state.vy = snapshot.player.velocityY;
    state.mode = snapshot.player.mode;
    state.mini = snapshot.player.mini;
    state.grounded = snapshot.player.grounded;
    state.upsideDown = snapshot.player.upsideDown;
    state.holding = snapshot.player.holding;
    state.alive = !snapshot.player.dead;
    state.objectWidth = std::max(1.0, snapshot.player.objectBoundsWidth);
    state.objectHeight = std::max(1.0, snapshot.player.objectBoundsHeight);
    state.sampleDt = calibration.normalizedStepDt();
    state.verticalPositionScale = calibration.yPositionScale();
    state.rawGravity = snapshot.player.gravity;
    state.gravityModifier = snapshot.player.gravityModifier;
    state.jumpVelocity = snapshot.player.jumpVelocity;
    state.speedScalar = snapshot.player.speed;
    return state;
}

TrajectoryPoint makePoint(
    SimState const& state,
    bool collision,
    bool hazard,
    bool landing,
    bool portal,
    bool modeChange,
    world::WorldRect const* nearestHazard
) {
    TrajectoryPoint point{};
    point.x = state.x;
    point.y = state.y;
    point.vx = state.vx;
    point.vy = state.vy;
    point.mode = state.mode;
    point.collision = collision;
    point.hazard = hazard;
    point.landing = landing;
    point.portal = portal;
    point.modeChange = modeChange;
    const auto player = playerEdges(state);
    point.playerBounds = rectFromEdges(player);
    if (nearestHazard) {
        point.nearestHazardBounds = *nearestHazard;
        const auto hazardEdges = edges(*nearestHazard);
        point.nearestHazardDistance = clearance(player, hazardEdges);
        point.nearestHazardIntersects = intersects(player, hazardEdges);
    }
    return point;
}

} // namespace

TrajectoryResult TrajectorySimulator::simulate(
    core::GameSnapshot const& snapshot,
    world::CollisionWorld const& collisionWorld,
    LocalWorldView const& localWorld,
    PhysicsValidationHarness const& validation,
    ActionCandidate const& candidate,
    std::size_t horizonTicks,
    bool initialHolding,
    double requiredForwardDistance,
    world::DynamicWorldModel const* dynamicWorld,
    world::TriggerWorldModel const* triggerWorld,
    bool exactHorizon,
    bool stopAtRequiredDistance
) const {
    TrajectoryResult result{};
    result.candidate = candidate;
    result.finalMode = snapshot.player.mode;
    result.requiredForwardDistance = std::max(0.0, requiredForwardDistance);

    if (!snapshot.valid || !collisionWorld.ready() || !localWorld.complete) {
        result.classification = TrajectoryClass::Unknown;
        result.uncertainGeometry = true;
        result.confidence = 0.0;
        return result;
    }

    auto calibration = validation.calibration(snapshot.player.mode);
    SimState state = makeState(snapshot, calibration);
    if (state.sampleDt <= 0.0) {
        result.classification = TrajectoryClass::Unknown;
        result.uncertainGeometry = true;
        return result;
    }

    const double startX = state.x;
    const double direction = state.vx < -0.001 ? -1.0 : 1.0;
    bool desiredHoldPrevious = initialHolding;
    auto triggerSimulation = triggerWorld && triggerWorld->ready()
        ? triggerWorld->createSimulation()
        : world::TriggerWorldModel::Simulation{};
    const bool triggerSimulationReady = triggerWorld && triggerWorld->ready();

    const auto relevant = localPrimitiveIndices(localWorld);
    auto const& primitives = collisionWorld.primitives();
    std::unordered_set<std::size_t> appliedPortals;
    appliedPortals.reserve(localWorld.portals.size() + 1);

    world::WorldRect nearestHazardBounds{};
    world::WorldRect const* nearestHazardPtr = nullptr;
    if (!localWorld.hazards.empty()) {
        result.nearestHazardPrimitive = localWorld.hazards.front().primitiveIndex;
        result.nearestHazardSeen = result.nearestHazardPrimitive != world::kInvalidPrimitiveIndex;
        nearestHazardBounds = localWorld.hazards.front().bounds;
        nearestHazardPtr = &nearestHazardBounds;
    }

    const std::size_t horizon = exactHorizon
        ? std::max<std::size_t>(horizonTicks, 1)
        : std::clamp<std::size_t>(horizonTicks, 8, 512);
    result.horizonTicks = horizon;
    result.points.reserve(horizon + 1);
    result.points.push_back(makePoint(
        state, false, false, false, false, false, nearestHazardPtr
    ));

    for (std::size_t tick = 0; tick < horizon && state.alive; ++tick) {
        const bool desiredHold = candidate.desiredHoldAt(tick);
        const bool pressEdge = desiredHold && !desiredHoldPrevious;
        const bool releaseEdge = !desiredHold && desiredHoldPrevious;
        desiredHoldPrevious = desiredHold;

        SimState previous = state;
        calibration = validation.calibration(state.mode);
        const double timeScale = triggerSimulationReady ? triggerSimulation.timeScale() : 1.0;
        const double modeDt = calibration.normalizedStepDt() * timeScale;
        if (modeDt > 0.0) {
            state.sampleDt = modeDt;
            state.verticalPositionScale = calibration.yPositionScale();
        }
        PhysicsStepContext context{calibration, state.sampleDt};
        auto const& model = m_registry.modelFor(state.mode);
        model.step(state, desiredHold, pressEdge, releaseEdge, context);
        state.holding = desiredHold;

        if (triggerSimulationReady) {
            const auto triggerEffects = triggerSimulation.step(
                previous.x,
                state.x,
                state.y,
                calibration.sampleDt
            );
            if (triggerEffects.gravityChanged) {
                state.gravityModifier = triggerEffects.gravityValue;
            }
            result.uncertainGeometry = result.uncertainGeometry || triggerEffects.uncertain;
        }

        bool pointCollision = false;
        bool pointHazard = false;
        bool pointLanding = false;
        bool pointPortal = false;
        bool pointModeChange = false;

        auto player = playerEdges(state);

        for (auto primitiveIndex : relevant) {
            if (primitiveIndex >= primitives.size()) continue;
            auto const& primitive = primitives[primitiveIndex];
            if (primitive.classification != world::GameplayObjectType::Portal
                || !primitive.enabled
                || appliedPortals.contains(primitiveIndex)) {
                continue;
            }
            auto portalBounds = primitive.broadphaseBounds;
            bool portalEnabled = primitive.enabled;
            bool causalPortal = false;
            if (triggerSimulationReady) {
                const auto causal = triggerSimulation.predictPrimitive(primitiveIndex);
                if (causal.available) {
                    portalEnabled = causal.enabled;
                    if (causal.causal) {
                        portalBounds = causal.bounds;
                        causalPortal = true;
                        result.uncertainGeometry = result.uncertainGeometry || causal.uncertain;
                    }
                }
            }
            if (!causalPortal && dynamicWorld) {
                const auto prediction = dynamicWorld->predict(
                    collisionWorld, primitiveIndex, tick + 1
                );
                if (prediction.available) portalBounds = prediction.bounds;
                if (prediction.dynamic && prediction.uncertain) {
                    result.uncertainGeometry = true;
                }
            }
            if (!portalEnabled || !intersects(player, edges(portalBounds))) continue;

            appliedPortals.insert(primitiveIndex);
            const auto application = PortalTransitionResolver::apply(primitive, state);
            result.portalCrossed = true;
            result.portalPrimitive = primitiveIndex;
            result.modeChanged = result.modeChanged || application.modeChanged;
            result.uncertainGeometry = result.uncertainGeometry || application.modelUncertain;
            pointPortal = true;
            pointModeChange = application.modeChanged;
            result.finalMode = state.mode;
        }

        player = playerEdges(state);
        for (auto primitiveIndex : relevant) {
            if (primitiveIndex >= primitives.size()) continue;
            auto const& primitive = primitives[primitiveIndex];
            if (!primitive.indexable) continue;

            bool predictedEnabled = primitive.enabled;
            bool causalBounds = false;
            auto predictedBounds = primitive.broadphaseBounds;
            if (triggerSimulationReady) {
                const auto causal = triggerSimulation.predictPrimitive(primitiveIndex);
                if (causal.available) {
                    predictedEnabled = causal.enabled;
                    if (causal.causal) {
                        predictedBounds = causal.bounds;
                        causalBounds = true;
                        result.uncertainGeometry = result.uncertainGeometry || causal.uncertain;
                    }
                }
            }
            if (!predictedEnabled) continue;

            if (primitive.geometryVerification != world::GeometryVerification::Verified
                && (primitive.classification == world::GameplayObjectType::Solid
                    || primitive.classification == world::GameplayObjectType::Hazard)) {
                result.uncertainGeometry = true;
            }

            if (!causalBounds && dynamicWorld) {
                const auto prediction = dynamicWorld->predict(
                    collisionWorld, primitiveIndex, tick + 1
                );
                if (prediction.available) predictedBounds = prediction.bounds;
                if (prediction.dynamic && prediction.uncertain) {
                    result.uncertainGeometry = true;
                }
            }
            const auto object = edges(predictedBounds);

            if (primitive.classification == world::GameplayObjectType::Hazard) {
                result.minimumClearance = std::min(
                    result.minimumClearance,
                    clearance(player, object)
                );
                if (intersects(player, object)) {
                    result.hazardCollision = true;
                    result.fatalCollision = true;
                    result.collisionPrimitive = primitiveIndex;
                    result.collisionTick = tick + 1;
                    state.alive = false;
                    pointCollision = true;
                    pointHazard = true;
                    break;
                }
            }

            if (primitive.classification == world::GameplayObjectType::Solid
                && intersects(player, object)) {
                if (resolveSolidContact(previous, state, predictedBounds)) {
                    result.landed = true;
                    pointLanding = true;
                    player = playerEdges(state);
                } else {
                    result.fatalCollision = true;
                    result.collisionPrimitive = primitiveIndex;
                    result.collisionTick = tick + 1;
                    state.alive = false;
                    pointCollision = true;
                    break;
                }
            }
        }

        result.points.push_back(makePoint(
            state,
            pointCollision,
            pointHazard,
            pointLanding,
            pointPortal,
            pointModeChange,
            nearestHazardPtr
        ));
        result.simulatedTicks = tick + 1;

        const double replayProgress = direction * (state.x - startX);
        if (stopAtRequiredDistance
            && !result.fatalCollision
            && replayProgress + 0.001 >= result.requiredForwardDistance) {
            break;
        }
    }

    result.progress = direction * (state.x - startX);
    result.predictedFinalX = state.x;
    result.predictedFinalY = state.y;
    result.finalMode = state.mode;
    // Crossing a portal is not an end condition. A general solver must keep
    // simulating through Cube -> Ship -> Wave (etc.) and validate the state
    // after the transition. The horizon is conclusive only after reaching the
    // requested forward objective or a real fatal event.
    result.relevantEventReached = result.fatalCollision
        || result.progress + 0.001 >= result.requiredForwardDistance;
    result.horizonConclusive = result.relevantEventReached;

    if (result.fatalCollision) {
        result.classification = TrajectoryClass::Collision;
    } else if (!result.horizonConclusive) {
        result.classification = TrajectoryClass::HorizonInconclusive;
    } else if (result.uncertainGeometry) {
        result.classification = TrajectoryClass::Risky;
    } else {
        result.classification = TrajectoryClass::Safe;
    }

    const auto& finalCalibration = validation.calibration(state.mode);
    const double calibrationConfidence = std::min(
        validation.calibration(snapshot.player.mode).confidence,
        finalCalibration.confidence > 0.0 ? finalCalibration.confidence : 1.0
    );
    const double geometryFactor = result.uncertainGeometry ? 0.58 : 1.0;
    const double horizonFactor = result.horizonConclusive ? 1.0 : 0.2;
    result.confidence = std::clamp(
        calibrationConfidence * geometryFactor * horizonFactor,
        0.0,
        1.0
    );

    if (!std::isfinite(result.minimumClearance)) {
        result.minimumClearance = localWorld.horizonY;
    }
    return result;
}

} // namespace autobot::solver
