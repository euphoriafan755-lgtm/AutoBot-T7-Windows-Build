#include "autobot/control/AutonomousTestDriver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace autobot::control {
namespace {

struct Extents {
    float left = 0.0f;
    float right = 0.0f;
    float bottom = 0.0f;
    float top = 0.0f;
};

Extents extents(world::WorldRect const& rect) {
    const float x2 = rect.x + rect.width;
    const float y2 = rect.y + rect.height;
    return {
        std::min(rect.x, x2),
        std::max(rect.x, x2),
        std::min(rect.y, y2),
        std::max(rect.y, y2),
    };
}

bool isCollisionSurface(world::CollisionPrimitive const& primitive) {
    return primitive.classification == world::GameplayObjectType::Solid
        || primitive.classification == world::GameplayObjectType::Hazard;
}

bool pathRelevant(
    world::CollisionPrimitive const& primitive,
    double playerY,
    bool upsideDown
) {
    const auto e = extents(primitive.objectBounds);

    // This is deliberately only a broad state-based path band. It is NOT a
    // player hitbox and does not claim exact physics/collision validation.
    const double aheadSide = upsideDown ? e.bottom : e.top;
    if (primitive.classification == world::GameplayObjectType::Solid) {
        return upsideDown
            ? aheadSide < playerY - 6.0 && e.top > playerY - 42.0
            : aheadSide > playerY + 6.0 && e.bottom < playerY + 42.0;
    }

    return e.top >= playerY - 30.0 && e.bottom <= playerY + 30.0;
}

double forwardDistance(
    world::CollisionPrimitive const& primitive,
    double playerX,
    double direction
) {
    const auto e = extents(primitive.objectBounds);
    if (direction >= 0.0) {
        if (playerX > e.right) return -1.0;
        return std::max(0.0, static_cast<double>(e.left) - playerX);
    }
    if (playerX < e.left) return -1.0;
    return std::max(0.0, playerX - static_cast<double>(e.right));
}

} // namespace

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

AutonomousDecision AutonomousTestDriver::decide(
    core::GameSnapshot const& snapshot,
    world::CollisionWorld const& collisionWorld,
    world::CollisionQueryResult const& query,
    bool enabled,
    bool botHolding
) const {
    AutonomousDecision decision{};
    decision.enabled = enabled;
    decision.ownership = enabled ? InputOwnership::Bot : InputOwnership::User;

    if (!enabled) {
        decision.reason = "AUTOBOT DISABLED";
        return decision;
    }
    if (!snapshot.valid) {
        decision.action = InputAction::SafeStop;
        decision.ownership = InputOwnership::None;
        decision.reason = "INVALID GAME STATE";
        return decision;
    }
    if (snapshot.player.dead) {
        decision.action = InputAction::SafeStop;
        decision.ownership = InputOwnership::None;
        decision.reason = "PLAYER DEAD - WAITING FOR RESTART";
        return decision;
    }
    if (!collisionWorld.ready()) {
        decision.action = InputAction::SafeStop;
        decision.ownership = InputOwnership::None;
        decision.reason = "COLLISION WORLD NOT READY";
        return decision;
    }
    if (snapshot.player.mode != core::GameMode::Cube) {
        decision.action = InputAction::SafeStop;
        decision.ownership = InputOwnership::None;
        decision.reason = "UNSUPPORTED MODE - CONTROL STOPPED";
        return decision;
    }

    const double speed = std::abs(snapshot.player.velocityX);
    const double direction = snapshot.player.velocityX < -0.001 ? -1.0 : 1.0;
    const double lookAhead = std::clamp(42.0 + speed * 18.0, 54.0, 180.0);
    const double triggerDistance = std::clamp(24.0 + speed * 10.0, 36.0, lookAhead * 0.82);

    std::size_t bestIndex = world::kInvalidPrimitiveIndex;
    double bestDistance = std::numeric_limits<double>::infinity();
    bool bestUnsupported = false;

    for (auto primitiveIndex : query.primitiveIndices) {
        if (primitiveIndex >= collisionWorld.primitives().size()) continue;
        auto const& primitive = collisionWorld.primitives()[primitiveIndex];
        if (!primitive.enabled || !primitive.indexable || !isCollisionSurface(primitive)) continue;
        if (!pathRelevant(primitive, snapshot.player.y, snapshot.player.upsideDown)) continue;

        const double distance = forwardDistance(primitive, snapshot.player.x, direction);
        if (distance < 0.0 || distance > lookAhead) continue;

        if (distance < bestDistance) {
            bestDistance = distance;
            bestIndex = primitiveIndex;
            bestUnsupported = primitive.support != world::V01Support::Supported;
        }
    }

    if (bestIndex != world::kInvalidPrimitiveIndex) {
        auto const& target = collisionWorld.primitives()[bestIndex];
        decision.targetPrimitiveIndex = bestIndex;
        decision.targetObjectID = target.objectID;
        decision.targetDistance = bestDistance;

        if (bestUnsupported && bestDistance <= triggerDistance) {
            decision.action = InputAction::SafeStop;
            decision.ownership = InputOwnership::None;
            decision.reason = "UNSUPPORTED WORLD STATE - SAFE STOP";
            return decision;
        }
    }

    decision.active = true;
    decision.ownership = InputOwnership::Bot;

    const bool risingAwayFromGround = snapshot.player.upsideDown
        ? snapshot.player.velocityY < -0.05
        : snapshot.player.velocityY > 0.05;

    if (botHolding) {
        if (risingAwayFromGround && bestIndex != world::kInvalidPrimitiveIndex && bestDistance <= lookAhead) {
            decision.action = InputAction::Hold;
            decision.reason = "ASCENDING PAST CURRENT OBSTACLE";
        } else {
            decision.action = InputAction::Release;
            decision.reason = bestIndex == world::kInvalidPrimitiveIndex
                ? "FORWARD PATH CLEAR - RELEASE"
                : "ASCENT ENDED OR TARGET PASSED";
        }
        return decision;
    }

    if (snapshot.player.grounded
        && bestIndex != world::kInvalidPrimitiveIndex
        && bestDistance <= triggerDistance) {
        decision.action = InputAction::Press;
        decision.reason = "SUPPORTED COLLISION RISK ON CURRENT PATH";
        return decision;
    }

    decision.action = InputAction::NoPress;
    decision.reason = snapshot.player.grounded
        ? "NO SUPPORTED COLLISION RISK INSIDE TRIGGER DISTANCE"
        : "AIRBORNE - NO NEW CUBE INPUT";
    return decision;
}

} // namespace autobot::control
