#pragma once

#include "autobot/world/WorldObject.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

namespace autobot::world {

enum class CollisionShapeKind {
    AxisAlignedRect,
    RotatedRect,
    Triangle,
    HazardBounds,
    ReferenceBounds,
};

enum class GeometryVerification {
    Verified,
    NotVerified,
    NotSupported,
};

inline constexpr std::string_view toString(CollisionShapeKind shape) {
    switch (shape) {
        case CollisionShapeKind::AxisAlignedRect: return "AABB";
        case CollisionShapeKind::RotatedRect: return "ROTATED_RECT";
        case CollisionShapeKind::Triangle: return "TRIANGLE";
        case CollisionShapeKind::HazardBounds: return "HAZARD_BOUNDS";
        case CollisionShapeKind::ReferenceBounds: return "REFERENCE_BOUNDS";
    }
    return "UNKNOWN";
}

inline constexpr std::string_view toString(GeometryVerification status) {
    switch (status) {
        case GeometryVerification::Verified: return "VERIFIED";
        case GeometryVerification::NotVerified: return "NOT VERIFIED";
        case GeometryVerification::NotSupported: return "NOT SUPPORTED";
    }
    return "NOT VERIFIED";
}

struct CollisionPoint {
    float x = 0.0f;
    float y = 0.0f;
};

struct CollisionPrimitive {
    std::size_t sourceIndex = 0;
    int objectID = 0;

    GameplayObjectType classification = GameplayObjectType::Unknown;
    V01Support support = V01Support::NotSupported;
    CollisionShapeKind shape = CollisionShapeKind::ReferenceBounds;
    GeometryVerification geometryVerification = GeometryVerification::NotSupported;

    // Copied from GameObject::getObjectRect() by LevelParser. This is an
    // object/visual broad-phase bound, NOT automatically a gameplay hitbox.
    WorldRect objectBounds{};

    // Broad-phase bounds used only by SpatialHash to find nearby objects.
    WorldRect broadphaseBounds{};

    // Candidate gameplay geometry. It MUST NOT be treated as exact unless
    // geometryVerification == Verified.
    WorldRect gameplayBounds{};

    std::array<CollisionPoint, 4> vertices{};
    std::uint8_t vertexCount = 0;

    float x = 0.0f;
    float y = 0.0f;
    float rotation = 0.0f;

    bool enabled = true;
    bool slope = false;
};

struct CollisionQueryResult {
    WorldRect region{};
    std::vector<std::size_t> primitiveIndices;

    std::size_t solids = 0;
    std::size_t hazards = 0;
    std::size_t slopes = 0;
    std::size_t unsupported = 0;

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

    [[nodiscard]] std::size_t total() const {
        std::size_t value = 0;
        for (auto count : categoryCounts) value += count;
        return value;
    }
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

    QueryBenchmark queryBenchmark{};
};

} // namespace autobot::world
