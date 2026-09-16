#include "autobot/solver/GeneralWorldModel.hpp"

#include <algorithm>
#include <cmath>

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

double forwardDistance(world::WorldRect const& rect, double x, double direction) {
    const auto e = edges(rect);
    if (direction >= 0.0) {
        if (e.right < x) return -1.0;
        return std::max(0.0, e.left - x);
    }
    if (e.left > x) return -1.0;
    return std::max(0.0, x - e.right);
}

GlobalEventKind mapKind(world::GameplayObjectType type) {
    switch (type) {
        case world::GameplayObjectType::Solid: return GlobalEventKind::Solid;
        case world::GameplayObjectType::Hazard: return GlobalEventKind::Hazard;
        case world::GameplayObjectType::Portal: return GlobalEventKind::Portal;
        case world::GameplayObjectType::Orb:
        case world::GameplayObjectType::Pad:
            return GlobalEventKind::Interactive;
        case world::GameplayObjectType::Decoration:
        case world::GameplayObjectType::Unknown:
            return GlobalEventKind::Unknown;
    }
    return GlobalEventKind::Unknown;
}

} // namespace

GeneralWorldView GeneralWorldModelBuilder::build(
    core::GameSnapshot const& snapshot,
    world::CollisionWorld const& collisionWorld,
    double lookaheadX,
    double lookaheadY
) const {
    GeneralWorldView result{};
    if (!snapshot.valid || !collisionWorld.ready()) return result;

    result.direction = snapshot.player.velocityX < -0.001 ? -1.0 : 1.0;
    result.lookaheadX = std::clamp(lookaheadX, 240.0, 12000.0);
    result.lookaheadY = std::clamp(lookaheadY, 300.0, 4000.0);

    const double behind = std::min(120.0, result.lookaheadX * 0.1);
    const double x0 = result.direction >= 0.0
        ? snapshot.player.x - behind
        : snapshot.player.x - result.lookaheadX + behind;
    const double y0 = snapshot.player.y - result.lookaheadY * 0.5;
    result.queryRect = {
        static_cast<float>(x0),
        static_cast<float>(y0),
        static_cast<float>(result.lookaheadX),
        static_cast<float>(result.lookaheadY),
    };

    const auto query = collisionWorld.queryRegion(result.queryRect);
    auto const& primitives = collisionWorld.primitives();
    result.events.reserve(query.primitiveIndices.size());

    for (auto primitiveIndex : query.primitiveIndices) {
        if (primitiveIndex >= primitives.size()) continue;
        auto const& primitive = primitives[primitiveIndex];
        if (!primitive.enabled || !primitive.indexable) continue;
        if (primitive.classification == world::GameplayObjectType::Decoration) continue;

        const double distance = forwardDistance(
            primitive.broadphaseBounds,
            snapshot.player.x,
            result.direction
        );
        if (distance < 0.0 || distance > result.lookaheadX) continue;

        GlobalWorldEvent event{};
        event.primitiveIndex = primitiveIndex;
        event.kind = mapKind(primitive.classification);
        event.bounds = primitive.broadphaseBounds;
        event.forwardDistance = distance;
        event.dynamic = primitive.potentiallyDynamic || primitive.observedDynamic;
        event.verifiedGeometry =
            primitive.geometryVerification == world::GeometryVerification::Verified;

        switch (event.kind) {
            case GlobalEventKind::Solid: ++result.solids; break;
            case GlobalEventKind::Hazard: ++result.hazards; break;
            case GlobalEventKind::Portal: ++result.portals; break;
            case GlobalEventKind::Interactive: ++result.interactive; break;
            case GlobalEventKind::Unknown: ++result.unknown; break;
        }
        if (event.dynamic) ++result.dynamic;
        result.events.push_back(event);
    }

    std::stable_sort(
        result.events.begin(),
        result.events.end(),
        [](GlobalWorldEvent const& a, GlobalWorldEvent const& b) {
            if (a.forwardDistance != b.forwardDistance) {
                return a.forwardDistance < b.forwardDistance;
            }
            return a.primitiveIndex < b.primitiveIndex;
        }
    );

    result.complete = true;
    return result;
}

} // namespace autobot::solver
