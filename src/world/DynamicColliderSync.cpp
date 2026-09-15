#include "autobot/world/DynamicColliderSync.hpp"

#include "autobot/world/CollisionWorld.hpp"
#include "autobot/world/LevelParser.hpp"
#include "autobot/world/WorldObject.hpp"

#include <chrono>

namespace autobot::world {

DynamicSyncStats DynamicColliderSync::sync(
    PlayLayer* playLayer,
    StaticWorld& staticWorld,
    CollisionWorld& collisionWorld
) const {
    DynamicSyncStats stats{};
    if (!playLayer || !staticWorld.parsed || !collisionWorld.ready()) return stats;

    const auto start = std::chrono::steady_clock::now();
    auto const& primitives = collisionWorld.primitives();

    for (auto primitiveIndex : collisionWorld.dynamicWatchPrimitiveIndices()) {
        ++stats.watched;
        if (primitiveIndex >= primitives.size()) {
            ++stats.readFailures;
            continue;
        }

        auto const& primitive = primitives[primitiveIndex];
        if (primitive.sourceIndex >= staticWorld.objects.size()) {
            ++stats.readFailures;
            continue;
        }

        auto const& previousObject = staticWorld.objects[primitive.sourceIndex];
        WorldObject liveObject{};
        if (!LevelParser::snapshotObjectAt(
                playLayer,
                previousObject.playLayerObjectIndex,
                liveObject
            )) {
            ++stats.readFailures;
            continue;
        }

        // sourceIndex is our copied-world identity, while uniqueID/objectID are
        // used as a guard that the CCArray slot still points at the same GD
        // object instance/type. The CollisionWorld never stores GameObject*.
        if ((previousObject.uniqueID != 0 && liveObject.uniqueID != previousObject.uniqueID)
            || liveObject.objectID != previousObject.objectID) {
            ++stats.identityMismatches;
            continue;
        }

        bool reindexed = false;
        if (collisionWorld.updatePrimitiveFromWorldObject(
                primitive.sourceIndex,
                liveObject,
                reindexed
            )) {
            staticWorld.objects[primitive.sourceIndex] = liveObject;
            ++stats.changed;
            if (reindexed) ++stats.reindexed;
        }
    }

    const auto end = std::chrono::steady_clock::now();
    stats.syncMs = std::chrono::duration<double, std::milli>(end - start).count();
    return stats;
}

} // namespace autobot::world
