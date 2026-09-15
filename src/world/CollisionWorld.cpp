#include "autobot/world/CollisionWorld.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <numeric>
#include <unordered_map>
#include <vector>

namespace autobot::world {
namespace {

using Clock = std::chrono::steady_clock;

constexpr float kPi = 3.14159265358979323846f;

std::size_t categoryIndex(GameplayObjectType type) {
    switch (type) {
        case GameplayObjectType::Solid: return 0;
        case GameplayObjectType::Hazard: return 1;
        case GameplayObjectType::Orb: return 2;
        case GameplayObjectType::Pad: return 3;
        case GameplayObjectType::Portal: return 4;
        case GameplayObjectType::Decoration: return 5;
        case GameplayObjectType::Unknown: return 6;
    }
    return 6;
}

float normalizedRotation(float degrees) {
    float value = std::fmod(degrees, 360.0f);
    if (value < 0.0f) value += 360.0f;
    return value;
}

bool nearMultipleOf90(float degrees) {
    const float normalized = normalizedRotation(degrees);
    const float nearest = std::round(normalized / 90.0f) * 90.0f;
    return std::fabs(normalized - nearest) < 0.01f
        || std::fabs(normalized - nearest + 360.0f) < 0.01f;
}

CollisionPoint rotatePoint(CollisionPoint point, CollisionPoint center, float degrees) {
    const float radians = degrees * kPi / 180.0f;
    const float cosine = std::cos(radians);
    const float sine = std::sin(radians);
    const float localX = point.x - center.x;
    const float localY = point.y - center.y;

    return {
        center.x + localX * cosine - localY * sine,
        center.y + localX * sine + localY * cosine,
    };
}

std::array<CollisionPoint, 4> rectVertices(WorldRect const& rect, float rotation) {
    const float x2 = rect.x + rect.width;
    const float y2 = rect.y + rect.height;
    const float left = std::min(rect.x, x2);
    const float right = std::max(rect.x, x2);
    const float bottom = std::min(rect.y, y2);
    const float top = std::max(rect.y, y2);
    const CollisionPoint center{(left + right) * 0.5f, (bottom + top) * 0.5f};

    std::array<CollisionPoint, 4> result{{
        {left, bottom},
        {right, bottom},
        {right, top},
        {left, top},
    }};

    if (std::fabs(rotation) > 0.01f) {
        for (auto& point : result) point = rotatePoint(point, center, rotation);
    }
    return result;
}

std::array<CollisionPoint, 4> slopeCandidateVertices(WorldRect const& rect, float rotation) {
    const float x2 = rect.x + rect.width;
    const float y2 = rect.y + rect.height;
    const float left = std::min(rect.x, x2);
    const float right = std::max(rect.x, x2);
    const float bottom = std::min(rect.y, y2);
    const float top = std::max(rect.y, y2);
    const CollisionPoint center{(left + right) * 0.5f, (bottom + top) * 0.5f};

    // This triangle is deliberately only a diagnostic candidate derived from
    // OBJECT BOUNDS. It is never marked VERIFIED here.
    std::array<CollisionPoint, 4> result{{
        {left, bottom},
        {right, bottom},
        {right, top},
        {0.0f, 0.0f},
    }};

    for (std::size_t i = 0; i < 3; ++i) {
        result[i] = rotatePoint(result[i], center, rotation);
    }
    return result;
}

double elapsedMs(Clock::time_point start, Clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

double percentile(std::vector<double> const& values, double fraction) {
    if (values.empty()) return 0.0;
    const auto index = static_cast<std::size_t>(
        std::ceil(fraction * static_cast<double>(values.size())) - 1.0
    );
    return values[std::min(index, values.size() - 1)];
}

} // namespace

CollisionPrimitive CollisionWorld::makePrimitive(WorldObject const& object, std::size_t sourceIndex) {
    CollisionPrimitive primitive{};
    primitive.sourceIndex = sourceIndex;
    primitive.objectID = object.objectID;
    primitive.classification = object.type;
    primitive.support = object.v01Support;
    primitive.objectBounds = object.objectRect;
    primitive.broadphaseBounds = object.objectRect;
    primitive.gameplayBounds = object.objectRect;
    primitive.x = object.x;
    primitive.y = object.y;
    primitive.rotation = object.rotation;
    primitive.enabled = object.enabled;
    primitive.slope = object.slope;

    if (object.type == GameplayObjectType::Solid) {
        if (object.slope) {
            primitive.shape = CollisionShapeKind::Triangle;
            primitive.vertices = slopeCandidateVertices(object.objectRect, object.rotation);
            primitive.vertexCount = 3;
            primitive.geometryVerification = GeometryVerification::NotVerified;
        } else if (nearMultipleOf90(object.rotation)) {
            primitive.shape = CollisionShapeKind::AxisAlignedRect;
            primitive.vertices = rectVertices(object.objectRect, 0.0f);
            primitive.vertexCount = 4;
            primitive.geometryVerification = GeometryVerification::NotVerified;
        } else {
            primitive.shape = CollisionShapeKind::RotatedRect;
            primitive.vertices = rectVertices(object.objectRect, object.rotation);
            primitive.vertexCount = 4;
            primitive.geometryVerification = GeometryVerification::NotVerified;
        }
    } else if (object.type == GameplayObjectType::Hazard) {
        primitive.shape = CollisionShapeKind::HazardBounds;
        primitive.vertices = rectVertices(object.objectRect, object.rotation);
        primitive.vertexCount = 4;
        primitive.geometryVerification = GeometryVerification::NotVerified;
    } else {
        primitive.shape = CollisionShapeKind::ReferenceBounds;
        primitive.vertices = rectVertices(object.objectRect, object.rotation);
        primitive.vertexCount = 4;
        primitive.geometryVerification = GeometryVerification::NotSupported;
    }

    if (object.v01Support == V01Support::NotSupported) {
        primitive.geometryVerification = GeometryVerification::NotSupported;
    }

    return primitive;
}

std::vector<ClassificationAuditEntry> CollisionWorld::buildClassificationAudit(StaticWorld const& source) {
    std::unordered_map<int, ClassificationAuditEntry> byID;
    byID.reserve(source.objects.size() / 4 + 1);

    for (auto const& object : source.objects) {
        auto& entry = byID[object.objectID];
        entry.objectID = object.objectID;
        ++entry.categoryCounts[categoryIndex(object.type)];

        switch (object.v01Support) {
            case V01Support::Supported: ++entry.supported; break;
            case V01Support::NotSupported: ++entry.notSupported; break;
            case V01Support::NonGameplay: ++entry.nonGameplay; break;
        }

        if (object.enabled) ++entry.enabled;
        else ++entry.disabled;
    }

    std::vector<ClassificationAuditEntry> result;
    result.reserve(byID.size());
    for (auto const& [id, entry] : byID) {
        (void)id;
        result.push_back(entry);
    }

    std::sort(result.begin(), result.end(), [](auto const& a, auto const& b) {
        if (a.total() != b.total()) return a.total() > b.total();
        return a.objectID < b.objectID;
    });

    return result;
}

QueryBenchmark CollisionWorld::benchmarkQueries(
    SpatialHash const& index,
    std::vector<CollisionPrimitive> const& primitives
) {
    QueryBenchmark benchmark{};
    if (primitives.empty()) return benchmark;

    constexpr std::size_t kMaxSamples = 512;
    const std::size_t samples = std::min(kMaxSamples, primitives.size());
    const std::size_t stride = std::max<std::size_t>(1, primitives.size() / samples);

    std::vector<double> times;
    times.reserve(samples);

    for (std::size_t sample = 0, primitiveIndex = 0;
         sample < samples;
         ++sample, primitiveIndex = std::min(primitives.size() - 1, primitiveIndex + stride)) {
        auto const& primitive = primitives[primitiveIndex];
        const float centerX = primitive.broadphaseBounds.x + primitive.broadphaseBounds.width * 0.5f;
        const float centerY = primitive.broadphaseBounds.y + primitive.broadphaseBounds.height * 0.5f;
        const WorldRect region{centerX - 120.0f, centerY - 180.0f, 720.0f, 360.0f};

        const auto start = Clock::now();
        auto nearby = index.queryRegion(region, primitives);
        const auto end = Clock::now();

        // Keep the query observable to the optimizer without changing behavior.
        if (nearby.size() == static_cast<std::size_t>(-1)) {
            return benchmark;
        }
        times.push_back(elapsedMs(start, end));
    }

    std::sort(times.begin(), times.end());
    benchmark.samples = times.size();
    benchmark.averageMs = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
    benchmark.p95Ms = percentile(times, 0.95);
    benchmark.p99Ms = percentile(times, 0.99);
    return benchmark;
}

bool CollisionWorld::build(StaticWorld const& source, double parseTimeMs) {
    m_ready = false;
    m_primitives.clear();
    m_spatialHash.clear();
    m_audit.clear();
    m_metrics = {};
    m_metrics.parseTimeMs = parseTimeMs;
    m_metrics.sourceObjects = source.objects.size();

    if (!source.parsed) return false;

    const auto totalStart = Clock::now();
    const auto primitiveStart = Clock::now();

    m_audit = buildClassificationAudit(source);
    m_primitives.reserve(source.solids + source.hazards + source.orbs + source.pads + source.portals);

    for (std::size_t index = 0; index < source.objects.size(); ++index) {
        auto const& object = source.objects[index];
        if (!object.enabled) continue;

        const bool gameplayRelevant =
            object.type == GameplayObjectType::Solid
            || object.type == GameplayObjectType::Hazard
            || object.type == GameplayObjectType::Orb
            || object.type == GameplayObjectType::Pad
            || object.type == GameplayObjectType::Portal;

        if (!gameplayRelevant) continue;

        auto primitive = makePrimitive(object, index);
        if (primitive.classification == GameplayObjectType::Solid
            || primitive.classification == GameplayObjectType::Hazard) {
            ++m_metrics.collisionCandidates;
        }

        if (primitive.geometryVerification == GeometryVerification::NotVerified) {
            ++m_metrics.notVerifiedShapes;
        } else if (primitive.geometryVerification == GeometryVerification::NotSupported) {
            ++m_metrics.notSupportedShapes;
        }

        m_primitives.push_back(primitive);
    }

    const auto primitiveEnd = Clock::now();
    m_metrics.primitiveBuildTimeMs = elapsedMs(primitiveStart, primitiveEnd);

    const auto hashStart = Clock::now();
    m_spatialHash.build(m_primitives);
    const auto hashEnd = Clock::now();

    m_metrics.spatialHashBuildTimeMs = elapsedMs(hashStart, hashEnd);
    m_metrics.indexedObjects = m_primitives.size();
    m_metrics.spatialCells = m_spatialHash.cellCount();
    m_metrics.queryBenchmark = benchmarkQueries(m_spatialHash, m_primitives);

    const auto totalEnd = Clock::now();
    m_metrics.collisionWorldBuildTimeMs = elapsedMs(totalStart, totalEnd);

    m_ready = true;
    return true;
}

CollisionQueryResult CollisionWorld::queryRegion(WorldRect const& region) const {
    CollisionQueryResult result{};
    result.region = region;
    if (!m_ready) return result;

    const auto start = Clock::now();
    result.primitiveIndices = m_spatialHash.queryRegion(region, m_primitives);
    const auto end = Clock::now();
    result.queryMs = elapsedMs(start, end);

    for (auto index : result.primitiveIndices) {
        if (index >= m_primitives.size()) continue;
        auto const& primitive = m_primitives[index];

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

    return result;
}

CollisionQueryResult CollisionWorld::queryAhead(
    float playerX,
    float playerY,
    float width,
    float height
) const {
    const float normalizedWidth = std::max(width, 1.0f);
    const float normalizedHeight = std::max(height, 1.0f);
    const WorldRect region{
        playerX - 60.0f,
        playerY - normalizedHeight * 0.5f,
        normalizedWidth,
        normalizedHeight,
    };
    return queryRegion(region);
}

} // namespace autobot::world
