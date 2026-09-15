#include "autobot/world/CollisionQueryAudit.hpp"

#include <algorithm>
#include <chrono>

namespace autobot::world {
namespace {
using Clock = std::chrono::steady_clock;

bool intersects(WorldRect const& a, WorldRect const& b) {
    const float aX2 = a.x + a.width;
    const float aY2 = a.y + a.height;
    const float bX2 = b.x + b.width;
    const float bY2 = b.y + b.height;
    const float aLeft = std::min(a.x, aX2);
    const float aRight = std::max(a.x, aX2);
    const float aBottom = std::min(a.y, aY2);
    const float aTop = std::max(a.y, aY2);
    const float bLeft = std::min(b.x, bX2);
    const float bRight = std::max(b.x, bX2);
    const float bBottom = std::min(b.y, bY2);
    const float bTop = std::max(b.y, bY2);
    return !(aRight < bLeft || aLeft > bRight || aTop < bBottom || aBottom > bTop);
}
}

CollisionQueryResult bruteForceQueryRegion(
    CollisionWorld const& collisionWorld,
    WorldRect const& region
) {
    CollisionQueryResult result{};
    result.region = region;
    if (!collisionWorld.ready()) return result;

    const auto start = Clock::now();
    auto const& primitives = collisionWorld.primitives();
    result.primitiveIndices.reserve(primitives.size() / 8 + 1);
    for (std::size_t index = 0; index < primitives.size(); ++index) {
        auto const& primitive = primitives[index];
        if (!primitive.enabled || !primitive.indexable) continue;
        if (!intersects(primitive.broadphaseBounds, region)) continue;

        result.primitiveIndices.push_back(index);
        if (primitive.classification == GameplayObjectType::Solid) {
            ++result.solids;
            if (primitive.slope) ++result.slopes;
        } else if (primitive.classification == GameplayObjectType::Hazard) {
            ++result.hazards;
        }
        if (primitive.geometryVerification == GeometryVerification::NotSupported) {
            ++result.unsupported;
        }
    }
    const auto end = Clock::now();
    result.queryMs = std::chrono::duration<double, std::milli>(end - start).count();
    return result;
}

} // namespace autobot::world
