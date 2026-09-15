#pragma once

#include "autobot/world/WorldObject.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

namespace autobot::world {

enum class CollisionShapeKind {
    ObjectBounds,
    SlopeBounds,
    HazardBounds,
    ReferenceBounds,
};

enum class GeometryVerification {
    Verified,
    NotVerified,
    NotSupported,
};

enum class CollisionParticipation {
    CollisionSurface,
    GameplayReference,
    ExcludedNoTouch,
    DiagnosticOnly,
};

inline constexpr std::string_view toString(GameplayObjectType type) {
    switch (type) {
        case GameplayObjectType::Solid: return "SOLID";
        case GameplayObjectType::Hazard: return "HAZARD";
        case GameplayObjectType::Orb: return "ORB";
        case GameplayObjectType::Pad: return "PAD";
        case GameplayObjectType::Portal: return "PORTAL";
        case GameplayObjectType::Decoration: return "DECORATION";
        case GameplayObjectType::Unknown: return "UNKNOWN";
    }
    return "UNKNOWN";
}

inline constexpr std::string_view toString(V01Support status) {
    switch (status) {
        case V01Support::Supported: return "SUPPORTED";
        case V01Support::NotSupported: return "NOT SUPPORTED";
        case V01Support::NonGameplay: return "NON GAMEPLAY";
    }
    return "NOT SUPPORTED";
}

inline constexpr std::string_view toString(CollisionShapeKind shape) {
    switch (shape) {
        case CollisionShapeKind::ObjectBounds: return "OBJECT_BOUNDS";
        case CollisionShapeKind::SlopeBounds: return "SLOPE_BOUNDS";
        case CollisionShapeKind::HazardBounds: return "HAZARD_BOUNDS";
        case CollisionShapeKind::ReferenceBounds: return "REFERENCE_BOUNDS";
    }
    return "REFERENCE_BOUNDS";
}

inline constexpr std::string_view toString(GeometryVerification status) {
    switch (status) {
        case GeometryVerification::Verified: return "VERIFIED";
        case GeometryVerification::NotVerified: return "NOT VERIFIED";
        case GeometryVerification::NotSupported: return "NOT SUPPORTED";
    }
    return "NOT VERIFIED";
}

inline constexpr std::string_view toString(CollisionParticipation value) {
    switch (value) {
        case CollisionParticipation::CollisionSurface: return "COLLISION_SURFACE";
        case CollisionParticipation::GameplayReference: return "GAMEPLAY_REFERENCE";
        case CollisionParticipation::ExcludedNoTouch: return "EXCLUDED_NO_TOUCH";
        case CollisionParticipation::DiagnosticOnly: return "DIAGNOSTIC_ONLY";
    }
    return "DIAGNOSTIC_ONLY";
}

struct CollisionPoint {
    float x = 0.0f;
    float y = 0.0f;
};

struct CollisionPrimitive {
    std::size_t sourceIndex = 0;
    int sourceUniqueID = 0;
    int objectID = 0;
    int rawGameObjectType = -1;

    GameplayObjectType classification = GameplayObjectType::Unknown;
    V01Support support = V01Support::NotSupported;
    CollisionShapeKind shape = CollisionShapeKind::ReferenceBounds;
    GeometryVerification geometryVerification = GeometryVerification::NotSupported;
    CollisionParticipation participation = CollisionParticipation::DiagnosticOnly;

    WorldRect objectBounds{};
    WorldRect broadphaseBounds{};
    WorldRect gameplayBounds{};

    std::array<CollisionPoint, 4> vertices{};
    std::uint8_t vertexCount = 0;

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
    bool slope = false;
    bool indexable = false;
};

struct SpatialCell {
    int x = 0;
    int y = 0;

    bool operator==(SpatialCell const& other) const {
        return x == other.x && y == other.y;
    }
};

struct SpatialQueryDebug {
    std::size_t cellsVisited = 0;
    std::size_t objectsInCells = 0;
    std::size_t objectsAfterDedup = 0;
    std::size_t objectsAfterIntersection = 0;
    std::size_t duplicatesSuppressed = 0;
    std::size_t invalidIndices = 0;
};

struct CollisionQueryResult {
    WorldRect region{};
    std::vector<std::size_t> primitiveIndices;

    std::size_t solids = 0;
    std::size_t hazards = 0;
    std::size_t slopes = 0;
    std::size_t unsupported = 0;

    SpatialQueryDebug debug{};
    double queryMs = 0.0;
};

struct QueryBenchmark {
    std::size_t samples = 0;
    double averageMs = 0.0;
    double p95Ms = 0.0;
    double p99Ms = 0.0;
};

struct ClassificationAuditEntry {
    int objectID = 0;
    std::array<std::size_t, 7> categoryCounts{};
    std::size_t supported = 0;
    std::size_t notSupported = 0;
    std::size_t nonGameplay = 0;
    std::size_t enabled = 0;
    std::size_t disabled = 0;
    std::size_t noTouch = 0;

    [[nodiscard]] std::size_t total() const {
        std::size_t value = 0;
        for (auto count : categoryCounts) value += count;
        return value;
    }
};

struct RawClassificationAuditEntry {
    int rawGameObjectType = -1;
    std::array<std::size_t, 7> finalCategoryCounts{};
    std::size_t total = 0;
    std::size_t noTouch = 0;
    std::array<int, 5> sampleObjectIDs{};
    std::array<std::size_t, 5> sampleSourceIndices{};
    std::size_t sampleCount = 0;
};

struct CategoryConsistency {
    std::size_t parsed = 0;
    std::size_t supported = 0;
    std::size_t colliderCreated = 0;
    std::size_t indexed = 0;
    std::size_t intentionallySkipped = 0;
    std::size_t silentlyLost = 0;
};

struct CollisionWorldMetrics {
    double parseTimeMs = 0.0;
    double primitiveBuildTimeMs = 0.0;
    double spatialHashBuildTimeMs = 0.0;
    double collisionWorldBuildTimeMs = 0.0;

    std::size_t sourceObjects = 0;
    std::size_t indexedObjects = 0;
    std::size_t collisionCandidates = 0;
    std::size_t notVerifiedShapes = 0;
    std::size_t notSupportedShapes = 0;
    std::size_t spatialCells = 0;
    std::size_t silentlyLost = 0;

    std::array<CategoryConsistency, 7> consistency{};
    QueryBenchmark queryBenchmark{};
};

inline constexpr std::size_t kInvalidPrimitiveIndex = std::numeric_limits<std::size_t>::max();

} // namespace autobot::world
