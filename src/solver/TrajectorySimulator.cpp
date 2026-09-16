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
    world::CollisionPrimitive const& primitive
) {
    if (!canLand(current.mode)) return false;

    const auto solid = edges(primitive.broadphaseBounds);
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
    state.sampleDt = calibration.sampleDt;
    state.rawGravity = snapshot.player.gravity;
    state.gravityModifier = snapshot.player.gravityModifier;
    state.jumpAcceleration = snapshot.player.jumpAcceleration;
    state.speedScalar = snapshot.player.speed;
    return state;
}

} // namespace

TrajectoryResult TrajectorySimulator::simulate(
    core::GameSnapshot const& snapshot,
    world::CollisionWorld const& collisionWorld,
    LocalWorldView const& localWorld,
    PhysicsValidationHarness const& validation,
    ActionCandidate const& candidate,
    std::size_t horizonTicks
) const {
    TrajectoryResult result{};
    result.candidate = candidate;
    result.finalMode = snapshot.player.mode;

    if (!snapshot.valid || !collisionWorld.ready() || !localWorld.complete) {
        result.classification = TrajectoryClass::Unknown;
        result.uncertainGeometry = true;
        result.confidence = 0.0;
        return result;
    }

    auto calibration = validation.calibration(snapshot.player.mode);
    SimState state = makeState(snapshot, calibration);
    const double startX = state.x;
    const double direction = state.vx < -0.001 ? -1.0 : 1.0;
    bool desiredHoldPrevious = snapshot.player.holding;

    const auto relevant = localPrimitiveIndices(localWorld);
    auto const& primitives = collisionWorld.primitives();
    std::unordered_set<std::size_t> appliedPortals;
    appliedPortals.reserve(localWorld.portals.size() + 1);

    const std::size_t horizon = std::clamp<std::size_t>(horizonTicks, 8, 192);
    result.points.reserve(horizon + 1);
    result.points.push_back({
        state.x, state.y, state.vx, state.vy, state.mode,
        false, false, false, false, false
    });

    for (std::size_t tick = 0; tick < horizon && state.alive; ++tick) {
        const bool desiredHold = candidate.desiredHoldAt(tick);
        const bool pressEdge = desiredHold && !desiredHoldPrevious;
        const bool releaseEdge = !desiredHold && desiredHoldPrevious;
        desiredHoldPrevious = desiredHold;

        SimState previous = state;
        calibration = validation.calibration(state.mode);
        PhysicsStepContext context{calibration, calibration.sampleDt};
        auto const& model = m_registry.modelFor(state.mode);
        model.step(state, desiredHold, pressEdge, releaseEdge, context);
        state.holding = desiredHold;

        bool pointCollision = false;
        bool pointHazard = false;
        bool pointLanding = false;
        bool pointPortal = false;
        bool pointModeChange = false;

        auto player = playerEdges(state);

        // Portals are processed before collision scoring so a candidate can
        // continue under the new mode model on subsequent simulated samples.
        for (auto primitiveIndex : relevant) {
            if (primitiveIndex >= primitives.size()) continue;
            auto const& primitive = primitives[primitiveIndex];
            if (primitive.classification != world::GameplayObjectType::Portal
                || !primitive.enabled
                || appliedPortals.contains(primitiveIndex)) {
                continue;
            }
            if (!intersects(player, edges(primitive.broadphaseBounds))) continue;

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
            if (!primitive.enabled || !primitive.indexable) continue;

            if (primitive.geometryVerification != world::GeometryVerification::Verified
                && (primitive.classification == world::GameplayObjectType::Solid
                    || primitive.classification == world::GameplayObjectType::Hazard)) {
                result.uncertainGeometry = true;
            }

            const auto object = edges(primitive.broadphaseBounds);

            if (primitive.classification == world::GameplayObjectType::Hazard) {
                result.minimumClearance = std::min(
                    result.minimumClearance,
                    clearance(player, object)
                );
                if (intersects(player, object)) {
                    result.hazardCollision = true;
                    result.fatalCollision = true;
                    result.collisionPrimitive = primitiveIndex;
                    state.alive = false;
                    pointCollision = true;
                    pointHazard = true;
                    break;
                }
            }

            if (primitive.classification == world::GameplayObjectType::Solid
                && intersects(player, object)) {
                if (resolveSolidContact(previous, state, primitive)) {
                    result.landed = true;
                    pointLanding = true;
                    player = playerEdges(state);
                } else {
                    result.fatalCollision = true;
                    result.collisionPrimitive = primitiveIndex;
                    state.alive = false;
                    pointCollision = true;
                    break;
                }
            }
        }

        result.points.push_back({
            state.x,
            state.y,
            state.vx,
            state.vy,
            state.mode,
            pointCollision,
            pointHazard,
            pointLanding,
            pointPortal,
            pointModeChange
        });
    }

    result.progress = direction * (state.x - startX);
    result.finalMode = state.mode;

    if (result.fatalCollision) {
        result.classification = TrajectoryClass::Collision;
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
    result.confidence = std::clamp(calibrationConfidence * geometryFactor, 0.0, 1.0);

    if (!std::isfinite(result.minimumClearance)) {
        result.minimumClearance = localWorld.horizonY;
    }
    return result;
}

} // namespace autobot::solver
