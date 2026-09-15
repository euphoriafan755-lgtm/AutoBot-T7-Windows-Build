#include "autobot/world/SpatialHash.hpp"

#include <cassert>
#include <iostream>
#include <unordered_set>

using namespace autobot::world;

CollisionPrimitive primitive(WorldRect bounds) {
    CollisionPrimitive p{};
    p.enabled = true;
    p.indexable = true;
    p.broadphaseBounds = bounds;
    p.objectBounds = bounds;
    p.classification = GameplayObjectType::Solid;
    return p;
}

int main() {
    std::vector<CollisionPrimitive> primitives{
        primitive({-250.f, -130.f, 300.f, 260.f}),
        primitive({119.f, 119.f, 4.f, 4.f}),
        primitive({360.f, -20.f, 500.f, 40.f}),
        primitive({1000.f, 1000.f, 30.f, 30.f}),
    };

    SpatialHash hash{120.f};
    hash.build(primitives);

    assert(hash.contains(0));
    assert(hash.contains(1));
    assert(hash.contains(2));
    assert(hash.cellsFor(0).size() > 1);
    assert(hash.cellsFor(1).size() >= 4);
    assert(hash.cellsFor(2).size() > 1);

    SpatialQueryDebug debug{};
    auto result = hash.queryRegionDetailed({-130.f, -60.f, 300.f, 180.f}, primitives, debug);
    std::unordered_set<std::size_t> unique(result.begin(), result.end());
    assert(unique.size() == result.size());
    assert(unique.contains(0));
    assert(unique.contains(1));
    assert(!unique.contains(3));
    assert(debug.invalidIndices == 0);

    auto boundary = hash.queryRegion({120.f, 120.f, 1.f, 1.f}, primitives);
    std::unordered_set<std::size_t> boundarySet(boundary.begin(), boundary.end());
    assert(boundarySet.contains(1));

    auto large = hash.queryRegion({700.f, -10.f, 30.f, 20.f}, primitives);
    std::unordered_set<std::size_t> largeSet(large.begin(), large.end());
    assert(largeSet.contains(2));

    std::cout << "SPATIAL_HASH_INVARIANTS=PASS\n";
    return 0;
}
