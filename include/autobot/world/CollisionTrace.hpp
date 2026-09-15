#pragma once

#include "autobot/world/CollisionWorld.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace autobot::world {

struct WorldStructureFingerprint {
    std::size_t totalObjects = 0;
    std::array<std::size_t, 7> categoryCounts{};
    std::uint64_t structuralHash = 0;

    [[nodiscard]] bool operator==(WorldStructureFingerprint const& other) const {
        return totalObjects == other.totalObjects
            && categoryCounts == other.categoryCounts
            && structuralHash == other.structuralHash;
    }
};

struct CollisionTraceRecord {
    std::size_t sourceIndex = 0;
    int uniqueID = 0;
    int objectID = 0;
    int rawGameObjectType = -1;

    GameplayObjectType classification = GameplayObjectType::Unknown;
    V01Support support = V01Support::NotSupported;

    float x = 0.0f;
    float y = 0.0f;
    float nodeX = 0.0f;
    float nodeY = 0.0f;
    float rotation = 0.0f;
    float rotationX = 0.0f;
    float rotationY = 0.0f;
    float scaleX = 1.0f;
    float scaleY = 1.0f;
    float anchorX = 0.5f;
    float anchorY = 0.5f;
    float contentWidth = 0.0f;
    float contentHeight = 0.0f;
    bool flipX = false;
    bool flipY = false;
    bool noTouch = false;
    bool passable = false;
    bool groupDisabled = false;
    bool enabled = true;

    WorldRect objectBounds{};

    bool primitiveCreated = false;
    std::size_t primitiveIndex = kInvalidPrimitiveIndex;
    CollisionShapeKind primitiveType = CollisionShapeKind::ReferenceBounds;
    bool insertedIntoSpatialHash = false;
    std::vector<SpatialCell> spatialCells;

    bool intersectsCurrentQuery = false;
    bool returnedByQuery = false;
    bool rendererReceived = false;
    bool rendererDrew = false;

    std::string rejectionReason;
};

struct CollisionTraceQuery {
    WorldRect region{};
    SpatialQueryDebug diagnosticHashDebug{};
    std::vector<CollisionTraceRecord> records;

    std::size_t mainExpectedByFullScan = 0;
    std::size_t mainReturned = 0;
    std::size_t mainMissing = 0;
    std::size_t mainUnexpected = 0;
    std::size_t duplicateMainResults = 0;
    std::size_t invalidMainIndices = 0;
};

class CollisionTrace final {
public:
    bool build(StaticWorld const& source, CollisionWorld const& collisionWorld);
    void clear();

    [[nodiscard]] bool ready() const { return m_ready; }
    [[nodiscard]] CollisionTraceQuery query(
        WorldRect const& region,
        StaticWorld const& source,
        CollisionWorld const& collisionWorld,
        CollisionQueryResult const& mainQuery,
        bool runFullInvariantScan
    ) const;

    [[nodiscard]] static WorldStructureFingerprint fingerprint(StaticWorld const& source);

private:
    bool m_ready = false;
    std::vector<CollisionPrimitive> m_tracePrimitives;
    SpatialHash m_traceHash{120.0f};
};

} // namespace autobot::world
