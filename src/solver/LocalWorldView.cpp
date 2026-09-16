#include "autobot/solver/LocalWorldView.hpp"

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

double verticalDistance(world::WorldRect const& rect, double y) {
    const auto e = edges(rect);
    if (y < e.bottom) return e.bottom - y;
    if (y > e.top) return y - e.top;
    return 0.0;
}

} // namespace

LocalWorldView LocalWorldViewBuilder::build(
    core::GameSnapshot const& snapshot,
    world::CollisionWorld const& collisionWorld
) const {
    LocalWorldView view{};
    if (!snapshot.valid || !collisionWorld.ready()) return view;

    const double speed = std::max(std::abs(snapshot.player.velocityX), 1.0);
    const double direction = snapshot.player.velocityX < -0.001 ? -1.0 : 1.0;
    const double horizonX = std::clamp(360.0 + speed * 0.85, 480.0, 1400.0);
    const double horizonY = std::clamp(
        360.0 + std::abs(snapshot.player.velocityY) * 0.65,
        420.0,
        900.0
    );

    const float x0 = static_cast<float>(
        direction >= 0.0 ? snapshot.player.x - 90.0 : snapshot.player.x - horizonX + 90.0
    );
    const float y0 = static_cast<float>(snapshot.player.y - horizonY * 0.5);
    view.queryRect = {
        x0,
        y0,
        static_cast<float>(horizonX),
        static_cast<float>(horizonY),
    };
    view.horizonX = horizonX;
    view.horizonY = horizonY;

    const auto query = collisionWorld.queryRegion(view.queryRect);
    auto const& primitives = collisionWorld.primitives();

    for (auto primitiveIndex : query.primitiveIndices) {
        if (primitiveIndex >= primitives.size()) continue;
        auto const& primitive = primitives[primitiveIndex];
        if (!primitive.enabled || !primitive.indexable) continue;

        const double fwd = forwardDistance(
            primitive.broadphaseBounds,
            snapshot.player.x,
            direction
        );
        if (fwd < 0.0 || fwd > horizonX) continue;

        LocalWorldObject object{};
        object.primitiveIndex = primitiveIndex;
        object.classification = primitive.classification;
        object.bounds = primitive.broadphaseBounds;
        object.forwardDistance = fwd;
        object.verticalDistance = verticalDistance(
            primitive.broadphaseBounds,
            snapshot.player.y
        );
        object.potentiallyDynamic = primitive.potentiallyDynamic || primitive.observedDynamic;
        object.verifiedGeometry =
            primitive.geometryVerification == world::GeometryVerification::Verified;

        switch (primitive.classification) {
            case world::GameplayObjectType::Solid:
                view.solids.push_back(object);
                break;
            case world::GameplayObjectType::Hazard:
                view.hazards.push_back(object);
                break;
            case world::GameplayObjectType::Portal:
                view.portals.push_back(object);
                break;
            case world::GameplayObjectType::Unknown:
                view.unknown.push_back(object);
                break;
            default:
                break;
        }
    }

    auto byForward = [](LocalWorldObject const& a, LocalWorldObject const& b) {
        if (a.forwardDistance != b.forwardDistance) {
            return a.forwardDistance < b.forwardDistance;
        }
        return a.primitiveIndex < b.primitiveIndex;
    };
    std::sort(view.solids.begin(), view.solids.end(), byForward);
    std::sort(view.hazards.begin(), view.hazards.end(), byForward);
    std::sort(view.portals.begin(), view.portals.end(), byForward);
    std::sort(view.unknown.begin(), view.unknown.end(), byForward);
    view.complete = true;
    return view;
}

} // namespace autobot::solver
