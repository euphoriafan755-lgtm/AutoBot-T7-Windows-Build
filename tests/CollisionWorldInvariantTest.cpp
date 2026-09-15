#include "autobot/world/CollisionWorld.hpp"
#include "autobot/world/CollisionQueryAudit.hpp"

#include <cassert>
#include <iostream>
#include <unordered_set>
#include <vector>

using namespace autobot::world;

WorldObject solid(float x, float y, float w, float h, int id) {
    WorldObject o{};
    o.objectID = id;
    o.uniqueID = id * 10;
    o.rawGameObjectType = 0;
    o.type = GameplayObjectType::Solid;
    o.v01Support = V01Support::Supported;
    o.x = x + w * .5f;
    o.y = y + h * .5f;
    o.objectRect = {x, y, w, h};
    o.enabled = true;
    return o;
}

WorldObject hazard(float x, float y, float w, float h, int id) {
    auto o = solid(x, y, w, h, id);
    o.rawGameObjectType = 2;
    o.type = GameplayObjectType::Hazard;
    return o;
}

int main() {
    StaticWorld world{};
    world.parsed = true;
    world.objects = {
        solid(-240.f, -30.f, 240.f, 60.f, 1),
        hazard(25.f, 0.f, 30.f, 30.f, 8),
        solid(119.f, 119.f, 150.f, 30.f, 2),
        hazard(500.f, -100.f, 360.f, 200.f, 9),
    };
    world.solids = 2;
    world.hazards = 2;

    CollisionWorld collisionWorld{};
    assert(collisionWorld.build(world, 0.0));
    assert(collisionWorld.ready());
    assert(collisionWorld.metrics().silentlyLost == 0);

    std::vector<WorldRect> regions{
        {-300.f, -200.f, 500.f, 400.f},
        {-10.f, -10.f, 130.f, 130.f},
        {120.f, 120.f, 1.f, 1.f},
        {600.f, -50.f, 20.f, 100.f},
        {900.f, 900.f, 20.f, 20.f},
    };

    for (auto const& region : regions) {
        auto indexed = collisionWorld.queryRegion(region);
        auto brute = bruteForceQueryRegion(collisionWorld, region);
        std::unordered_set<std::size_t> a(indexed.primitiveIndices.begin(), indexed.primitiveIndices.end());
        std::unordered_set<std::size_t> b(brute.primitiveIndices.begin(), brute.primitiveIndices.end());
        assert(a.size() == indexed.primitiveIndices.size());
        assert(a == b);
        assert(indexed.debug.invalidIndices == 0);
    }

    std::cout << "COLLISION_WORLD_QUERY_INVARIANTS=PASS\n";
    return 0;
}
