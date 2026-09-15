#include "autobot/world/CollisionWorld.hpp"

#include <cassert>
#include <iostream>
#include <unordered_set>

using namespace autobot::world;

WorldObject dynamicSolid(float x, float y, int id) {
    WorldObject o{};
    o.objectID = id;
    o.uniqueID = id * 100;
    o.playLayerObjectIndex = static_cast<std::size_t>(id);
    o.rawGameObjectType = 0;
    o.type = GameplayObjectType::Solid;
    o.v01Support = V01Support::Supported;
    o.x = x + 15.f;
    o.y = y + 15.f;
    o.nodeX = o.x;
    o.nodeY = o.y;
    o.objectRect = {x, y, 30.f, 30.f};
    o.enabled = true;
    o.groupCount = 1;
    return o;
}

int main() {
    StaticWorld world{};
    world.parsed = true;
    world.objects = {dynamicSolid(0.f, 0.f, 1)};
    world.solids = 1;

    CollisionWorld collisionWorld{};
    assert(collisionWorld.build(world, 0.0));
    assert(collisionWorld.dynamicWatchPrimitiveIndices().size() == 1);

    auto const primitiveIndex = collisionWorld.primitiveForSource(0);
    assert(primitiveIndex != kInvalidPrimitiveIndex);
    const auto oldGeneration = collisionWorld.spatialHashGeneration();
    const auto oldCells = collisionWorld.cellsForPrimitive(primitiveIndex);
    assert(!oldCells.empty());

    auto oldQuery = collisionWorld.queryRegion({0.f, 0.f, 30.f, 30.f});
    assert(oldQuery.primitiveIndices.size() == 1);

    auto moved = world.objects[0];
    moved.x = 615.f;
    moved.y = 15.f;
    moved.nodeX = moved.x;
    moved.nodeY = moved.y;
    moved.objectRect = {600.f, 0.f, 30.f, 30.f};

    bool reindexed = false;
    assert(collisionWorld.updatePrimitiveFromWorldObject(0, moved, reindexed));
    assert(reindexed);
    assert(collisionWorld.spatialHashGeneration() > oldGeneration);

    auto oldAfter = collisionWorld.queryRegion({0.f, 0.f, 30.f, 30.f});
    assert(oldAfter.primitiveIndices.empty());

    auto newQuery = collisionWorld.queryRegion({600.f, 0.f, 30.f, 30.f});
    assert(newQuery.primitiveIndices.size() == 1);
    assert(newQuery.primitiveIndices[0] == primitiveIndex);

    std::unordered_set<std::size_t> unique(newQuery.primitiveIndices.begin(), newQuery.primitiveIndices.end());
    assert(unique.size() == newQuery.primitiveIndices.size());

    auto disabled = moved;
    disabled.groupDisabled = true;
    assert(collisionWorld.updatePrimitiveFromWorldObject(0, disabled, reindexed));
    assert(reindexed);
    auto disabledQuery = collisionWorld.queryRegion({600.f, 0.f, 30.f, 30.f});
    assert(disabledQuery.primitiveIndices.empty());

    auto reenabled = disabled;
    reenabled.groupDisabled = false;
    assert(collisionWorld.updatePrimitiveFromWorldObject(0, reenabled, reindexed));
    assert(reindexed);
    auto enabledQuery = collisionWorld.queryRegion({600.f, 0.f, 30.f, 30.f});
    assert(enabledQuery.primitiveIndices.size() == 1);

    std::cout << "DYNAMIC_COLLIDER_SYNC_INVARIANTS=PASS\n";
    return 0;
}
