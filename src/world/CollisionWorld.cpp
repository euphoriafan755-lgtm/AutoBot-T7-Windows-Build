#include "autobot/world/CollisionWorld.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <unordered_map>
#include <vector>

namespace autobot::world {
namespace {

using Clock = std::chrono::steady_clock;

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

std::array<CollisionPoint, 4> boundsVertices(WorldRect const& rect) {
    const float x2 = rect.x + rect.width;
    const float y2 = rect.y + rect.height;
    const float left = std::min(rect.x, x2);
    const float right = std::max(rect.x, x2);
    const float bottom = std::min(rect.y, y2);
    const float top = std::max(rect.y, y2);
    return {{{left, bottom}, {right, bottom}, {right, top}, {left, top}}};
}

bool gameplayRelevant(GameplayObjectType type) {
    return type == GameplayObjectType::Solid
        || type == GameplayObjectType::Hazard
        || type == GameplayObjectType::Orb
        || type == GameplayObjectType::Pad
        || type == GameplayObjectType::Portal;
}

bool collisionSurface(GameplayObjectType type) {
    return type == GameplayObjectType::Solid || type == GameplayObjectType::Hazard;
}

double elapsedMs(Clock::time_point start, Clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

bool nearlyEqual(float a, float b, float epsilon = 0.0005f) {
    return std::abs(a - b) <= epsilon;
}

bool sameRect(WorldRect const& a, WorldRect const& b) {
    return nearlyEqual(a.x, b.x)
        && nearlyEqual(a.y, b.y)
        && nearlyEqual(a.width, b.width)
        && nearlyEqual(a.height, b.height);
}

bool samePrimitiveState(CollisionPrimitive const& a, CollisionPrimitive const& b) {
    return sameRect(a.objectBounds, b.objectBounds)
        && nearlyEqual(a.x, b.x)
        && nearlyEqual(a.y, b.y)
        && nearlyEqual(a.nodeX, b.nodeX)
        && nearlyEqual(a.nodeY, b.nodeY)
        && nearlyEqual(a.rotation, b.rotation)
        && nearlyEqual(a.rotationX, b.rotationX)
        && nearlyEqual(a.rotationY, b.rotationY)
        && nearlyEqual(a.scaleX, b.scaleX)
        && nearlyEqual(a.scaleY, b.scaleY)
        && a.flipX == b.flipX
        && a.flipY == b.flipY
        && a.noTouch == b.noTouch
        && a.passable == b.passable
        && a.groupDisabled == b.groupDisabled
        && a.enabled == b.enabled
        && a.indexable == b.indexable;
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
    primitive.playLayerObjectIndex = object.playLayerObjectIndex;
    primitive.sourceUniqueID = object.uniqueID;
    primitive.objectID = object.objectID;
    primitive.rawGameObjectType = object.rawGameObjectType;
    primitive.classification = object.type;
    primitive.support = object.v01Support;
    primitive.objectBounds = object.objectRect;
    primitive.broadphaseBounds = object.objectRect;
    primitive.gameplayBounds = object.objectRect;
    primitive.vertices = boundsVertices(object.objectRect);
    primitive.vertexCount = 4;
    primitive.x = object.x;
    primitive.y = object.y;
    primitive.nodeX = object.nodeX;
    primitive.nodeY = object.nodeY;
    primitive.rotation = object.rotation;
    primitive.rotationX = object.rotationX;
    primitive.rotationY = object.rotationY;
    primitive.scaleX = object.scaleX;
    primitive.scaleY = object.scaleY;
    primitive.anchorX = object.anchorX;
    primitive.anchorY = object.anchorY;
    primitive.contentWidth = object.contentWidth;
    primitive.contentHeight = object.contentHeight;
    primitive.flipX = object.flipX;
    primitive.flipY = object.flipY;
    primitive.noTouch = object.noTouch;
    primitive.passable = object.passable;
    primitive.groupDisabled = object.groupDisabled;
    primitive.enabled = object.enabled && !object.groupDisabled;
    primitive.slope = object.slope;
    primitive.groupCount = object.groupCount;
    primitive.potentiallyDynamic = object.groupCount > 0;

    // getObjectRect() remains an OBJECT BOUND / broad-phase bound only.
    if (object.type == GameplayObjectType::Solid) {
        primitive.shape = object.slope
            ? CollisionShapeKind::SlopeBounds
            : CollisionShapeKind::ObjectBounds;
        primitive.geometryVerification = GeometryVerification::NotVerified;
        primitive.participation = object.noTouch
            ? CollisionParticipation::ExcludedNoTouch
            : CollisionParticipation::CollisionSurface;
    } else if (object.type == GameplayObjectType::Hazard) {
        primitive.shape = CollisionShapeKind::HazardBounds;
        primitive.geometryVerification = GeometryVerification::NotVerified;
        primitive.participation = object.noTouch
            ? CollisionParticipation::ExcludedNoTouch
            : CollisionParticipation::CollisionSurface;
    } else {
        primitive.shape = CollisionShapeKind::ReferenceBounds;
        primitive.geometryVerification = GeometryVerification::NotSupported;
        primitive.participation = CollisionParticipation::GameplayReference;
    }

    if (object.v01Support == V01Support::NotSupported) {
        primitive.geometryVerification = GeometryVerification::NotSupported;
    }

    primitive.indexable = primitive.enabled
        && primitive.participation != CollisionParticipation::ExcludedNoTouch;

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
        if (object.noTouch) ++entry.noTouch;
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

std::vector<RawClassificationAuditEntry> CollisionWorld::buildRawClassificationAudit(StaticWorld const& source) {
    std::unordered_map<int, RawClassificationAuditEntry> byRaw;
    byRaw.reserve(32);

    for (std::size_t sourceIndex = 0; sourceIndex < source.objects.size(); ++sourceIndex) {
        auto const& object = source.objects[sourceIndex];
        auto& entry = byRaw[object.rawGameObjectType];
        entry.rawGameObjectType = object.rawGameObjectType;
        ++entry.finalCategoryCounts[categoryIndex(object.type)];
        ++entry.total;
        if (object.noTouch) ++entry.noTouch;
        if (entry.sampleCount < entry.sampleObjectIDs.size()) {
            entry.sampleObjectIDs[entry.sampleCount] = object.objectID;
            entry.sampleSourceIndices[entry.sampleCount] = sourceIndex;
            ++entry.sampleCount;
        }
    }

    std::vector<RawClassificationAuditEntry> result;
    result.reserve(byRaw.size());
    for (auto const& [raw, entry] : byRaw) {
        (void)raw;
        result.push_back(entry);
    }
    std::sort(result.begin(), result.end(), [](auto const& a, auto const& b) {
        return a.rawGameObjectType < b.rawGameObjectType;
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
        if (nearby.size() == static_cast<std::size_t>(-1)) return benchmark;
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
    m_sourceToPrimitive.clear();
    m_dynamicWatchPrimitiveIndices.clear();
    m_audit.clear();
    m_rawAudit.clear();
    m_metrics = {};
    m_metrics.parseTimeMs = parseTimeMs;
    m_metrics.sourceObjects = source.objects.size();

    if (!source.parsed) return false;

    const auto totalStart = Clock::now();
    const auto primitiveStart = Clock::now();

    m_audit = buildClassificationAudit(source);
    m_rawAudit = buildRawClassificationAudit(source);
    m_sourceToPrimitive.assign(source.objects.size(), kInvalidPrimitiveIndex);
    m_primitives.reserve(source.solids + source.hazards + source.orbs + source.pads + source.portals);

    for (std::size_t sourceIndex = 0; sourceIndex < source.objects.size(); ++sourceIndex) {
        auto const& object = source.objects[sourceIndex];
        auto& consistency = m_metrics.consistency[categoryIndex(object.type)];
        ++consistency.parsed;
        if (object.v01Support == V01Support::Supported) ++consistency.supported;

        if (!gameplayRelevant(object.type)) {
            ++consistency.intentionallySkipped;
            continue;
        }

        auto primitive = makePrimitive(object, sourceIndex);
        const auto primitiveIndex = m_primitives.size();
        m_sourceToPrimitive[sourceIndex] = primitiveIndex;
        m_primitives.push_back(primitive);
        ++consistency.colliderCreated;

        if (primitive.potentiallyDynamic) {
            m_dynamicWatchPrimitiveIndices.push_back(primitiveIndex);
        }
        if (!primitive.indexable) {
            ++consistency.intentionallySkipped;
        }

        if (collisionSurface(primitive.classification) && primitive.indexable) {
            ++m_metrics.collisionCandidates;
        }

        if (primitive.geometryVerification == GeometryVerification::NotVerified) {
            ++m_metrics.notVerifiedShapes;
        } else if (primitive.geometryVerification == GeometryVerification::NotSupported) {
            ++m_metrics.notSupportedShapes;
        }
    }

    const auto primitiveEnd = Clock::now();
    m_metrics.primitiveBuildTimeMs = elapsedMs(primitiveStart, primitiveEnd);

    const auto hashStart = Clock::now();
    m_spatialHash.build(m_primitives);
    const auto hashEnd = Clock::now();
    m_metrics.spatialHashBuildTimeMs = elapsedMs(hashStart, hashEnd);

    for (std::size_t primitiveIndex = 0; primitiveIndex < m_primitives.size(); ++primitiveIndex) {
        auto const& primitive = m_primitives[primitiveIndex];
        auto& consistency = m_metrics.consistency[categoryIndex(primitive.classification)];
        if (!primitive.indexable) continue;
        if (m_spatialHash.contains(primitiveIndex)) {
            ++consistency.indexed;
        } else {
            ++consistency.silentlyLost;
            ++m_metrics.silentlyLost;
        }
    }

    m_metrics.indexedObjects = 0;
    for (std::size_t i = 0; i < m_primitives.size(); ++i) {
        if (m_spatialHash.contains(i)) ++m_metrics.indexedObjects;
    }
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
    result.primitiveIndices = m_spatialHash.queryRegionDetailed(region, m_primitives, result.debug);
    const auto end = Clock::now();
    result.queryMs = elapsedMs(start, end);

    for (auto index : result.primitiveIndices) {
        if (index >= m_primitives.size()) {
            ++result.debug.invalidIndices;
            continue;
        }
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

bool CollisionWorld::updatePrimitiveFromWorldObject(
    std::size_t sourceIndex,
    WorldObject const& liveObject,
    bool& reindexed
) {
    reindexed = false;
    if (!m_ready || sourceIndex >= m_sourceToPrimitive.size()) return false;

    const auto primitiveIndex = m_sourceToPrimitive[sourceIndex];
    if (primitiveIndex == kInvalidPrimitiveIndex || primitiveIndex >= m_primitives.size()) {
        return false;
    }

    const bool wasIndexed = m_spatialHash.contains(primitiveIndex);
    auto const previous = m_primitives[primitiveIndex];
    auto updated = makePrimitive(liveObject, sourceIndex);
    updated.observedDynamic = previous.observedDynamic;

    const bool changed = !samePrimitiveState(previous, updated);
    if (!changed) return false;

    const bool broadphaseChanged = !sameRect(previous.broadphaseBounds, updated.broadphaseBounds)
        || previous.indexable != updated.indexable
        || previous.enabled != updated.enabled;

    updated.observedDynamic = true;
    m_primitives[primitiveIndex] = updated;
    if (broadphaseChanged) {
        m_spatialHash.refreshPrimitive(primitiveIndex, m_primitives[primitiveIndex]);
        reindexed = true;
        const bool isIndexed = m_spatialHash.contains(primitiveIndex);
        if (!wasIndexed && isIndexed) ++m_metrics.indexedObjects;
        else if (wasIndexed && !isIndexed && m_metrics.indexedObjects > 0) --m_metrics.indexedObjects;
        m_metrics.spatialCells = m_spatialHash.cellCount();
    }
    return true;
}

std::size_t CollisionWorld::primitiveForSource(std::size_t sourceIndex) const {
    if (sourceIndex >= m_sourceToPrimitive.size()) return kInvalidPrimitiveIndex;
    return m_sourceToPrimitive[sourceIndex];
}

bool CollisionWorld::primitiveIndexed(std::size_t primitiveIndex) const {
    return m_spatialHash.contains(primitiveIndex);
}

std::vector<SpatialCell> const& CollisionWorld::cellsForPrimitive(std::size_t primitiveIndex) const {
    return m_spatialHash.cellsFor(primitiveIndex);
}

} // namespace autobot::world
