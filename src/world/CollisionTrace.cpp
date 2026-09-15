#include "autobot/world/CollisionTrace.hpp"

#include <algorithm>
#include <cstdint>
#include <unordered_set>

namespace autobot::world {
namespace {

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

struct Extents {
    float minX;
    float minY;
    float maxX;
    float maxY;
};

Extents extents(WorldRect const& rect) {
    const float x2 = rect.x + rect.width;
    const float y2 = rect.y + rect.height;
    return {
        std::min(rect.x, x2),
        std::min(rect.y, y2),
        std::max(rect.x, x2),
        std::max(rect.y, y2),
    };
}

bool intersects(WorldRect const& a, WorldRect const& b) {
    const auto ea = extents(a);
    const auto eb = extents(b);
    return ea.maxX >= eb.minX
        && ea.minX <= eb.maxX
        && ea.maxY >= eb.minY
        && ea.minY <= eb.maxY;
}

bool diagnosticRelevant(WorldObject const& object) {
    // Known decoration is intentionally excluded. Everything else is retained
    // for trace mode so unsupported/unknown gameplay candidates cannot vanish
    // silently from diagnostics.
    return object.type != GameplayObjectType::Decoration
        || object.v01Support != V01Support::NonGameplay;
}

void fnvMix(std::uint64_t& hash, std::uint64_t value) {
    constexpr std::uint64_t kPrime = 1099511628211ull;
    for (int i = 0; i < 8; ++i) {
        hash ^= static_cast<std::uint8_t>((value >> (i * 8)) & 0xffu);
        hash *= kPrime;
    }
}

} // namespace

void CollisionTrace::clear() {
    m_ready = false;
    m_tracePrimitives.clear();
    m_traceHash.clear();
}

bool CollisionTrace::build(StaticWorld const& source, CollisionWorld const& collisionWorld) {
    clear();
    if (!source.parsed || !collisionWorld.ready()) return false;

    m_tracePrimitives.reserve(source.solids + source.hazards + source.orbs
        + source.pads + source.portals + source.unknown);

    for (std::size_t sourceIndex = 0; sourceIndex < source.objects.size(); ++sourceIndex) {
        auto const& object = source.objects[sourceIndex];
        if (!object.enabled || !diagnosticRelevant(object)) continue;

        CollisionPrimitive proxy{};
        proxy.sourceIndex = sourceIndex;
        proxy.sourceUniqueID = object.uniqueID;
        proxy.objectID = object.objectID;
        proxy.rawGameObjectType = object.rawGameObjectType;
        proxy.classification = object.type;
        proxy.support = object.v01Support;
        proxy.shape = CollisionShapeKind::ReferenceBounds;
        proxy.geometryVerification = GeometryVerification::NotSupported;
        proxy.participation = CollisionParticipation::DiagnosticOnly;
        proxy.objectBounds = object.objectRect;
        proxy.broadphaseBounds = object.objectRect;
        proxy.gameplayBounds = object.objectRect;
        proxy.x = object.x;
        proxy.y = object.y;
        proxy.nodeX = object.nodeX;
        proxy.nodeY = object.nodeY;
        proxy.rotation = object.rotation;
        proxy.rotationX = object.rotationX;
        proxy.rotationY = object.rotationY;
        proxy.scaleX = object.scaleX;
        proxy.scaleY = object.scaleY;
        proxy.anchorX = object.anchorX;
        proxy.anchorY = object.anchorY;
        proxy.contentWidth = object.contentWidth;
        proxy.contentHeight = object.contentHeight;
        proxy.flipX = object.flipX;
        proxy.flipY = object.flipY;
        proxy.noTouch = object.noTouch;
        proxy.passable = object.passable;
        proxy.groupDisabled = object.groupDisabled;
        proxy.enabled = object.enabled;
        proxy.slope = object.slope;
        proxy.indexable = true;
        m_tracePrimitives.push_back(proxy);
    }

    m_traceHash.build(m_tracePrimitives);
    m_ready = true;
    return true;
}

CollisionTraceQuery CollisionTrace::query(
    WorldRect const& region,
    StaticWorld const& source,
    CollisionWorld const& collisionWorld,
    CollisionQueryResult const& mainQuery,
    bool runFullInvariantScan
) const {
    CollisionTraceQuery result{};
    result.region = region;
    if (!m_ready) return result;

    const auto traceIndices = m_traceHash.queryRegionDetailed(
        region, m_tracePrimitives, result.diagnosticHashDebug
    );

    std::unordered_set<std::size_t> mainReturned;
    mainReturned.reserve(mainQuery.primitiveIndices.size() * 2 + 1);
    for (auto primitiveIndex : mainQuery.primitiveIndices) {
        if (primitiveIndex >= collisionWorld.primitives().size()) {
            ++result.invalidMainIndices;
            continue;
        }
        if (!mainReturned.insert(primitiveIndex).second) {
            ++result.duplicateMainResults;
        }
    }
    result.mainReturned = mainReturned.size();

    result.records.reserve(traceIndices.size());
    for (auto traceIndex : traceIndices) {
        if (traceIndex >= m_tracePrimitives.size()) continue;
        auto const& proxy = m_tracePrimitives[traceIndex];
        if (proxy.sourceIndex >= source.objects.size()) continue;
        auto const& object = source.objects[proxy.sourceIndex];

        CollisionTraceRecord record{};
        record.sourceIndex = proxy.sourceIndex;
        record.uniqueID = object.uniqueID;
        record.objectID = object.objectID;
        record.rawGameObjectType = object.rawGameObjectType;
        record.classification = object.type;
        record.support = object.v01Support;
        record.x = object.x;
        record.y = object.y;
        record.nodeX = object.nodeX;
        record.nodeY = object.nodeY;
        record.rotation = object.rotation;
        record.rotationX = object.rotationX;
        record.rotationY = object.rotationY;
        record.scaleX = object.scaleX;
        record.scaleY = object.scaleY;
        record.anchorX = object.anchorX;
        record.anchorY = object.anchorY;
        record.contentWidth = object.contentWidth;
        record.contentHeight = object.contentHeight;
        record.flipX = object.flipX;
        record.flipY = object.flipY;
        record.noTouch = object.noTouch;
        record.passable = object.passable;
        record.groupDisabled = object.groupDisabled;
        record.enabled = object.enabled;
        record.objectBounds = object.objectRect;
        record.intersectsCurrentQuery = intersects(object.objectRect, region);

        const auto primitiveIndex = collisionWorld.primitiveForSource(proxy.sourceIndex);
        if (primitiveIndex != kInvalidPrimitiveIndex
            && primitiveIndex < collisionWorld.primitives().size()) {
            record.primitiveCreated = true;
            record.primitiveIndex = primitiveIndex;
            auto const& primitive = collisionWorld.primitives()[primitiveIndex];
            record.primitiveType = primitive.shape;
            record.insertedIntoSpatialHash = collisionWorld.primitiveIndexed(primitiveIndex);
            record.spatialCells = collisionWorld.cellsForPrimitive(primitiveIndex);
            record.returnedByQuery = mainReturned.contains(primitiveIndex);

            if (!primitive.indexable) {
                if (primitive.noTouch) {
                    record.rejectionReason = "NO_TOUCH: INTENTIONALLY NOT INDEXED";
                } else {
                    record.rejectionReason = "PRIMITIVE NOT INDEXABLE";
                }
            } else if (!record.insertedIntoSpatialHash) {
                record.rejectionReason = "INDEXABLE PRIMITIVE MISSING FROM SPATIAL HASH";
            } else if (record.intersectsCurrentQuery && !record.returnedByQuery) {
                record.rejectionReason = "SPATIAL HASH QUERY DROPPED INTERSECTING OBJECT";
            }
        } else {
            if (object.type == GameplayObjectType::Unknown) {
                record.rejectionReason = "NO COLLISION PRIMITIVE: UNKNOWN / NOT SUPPORTED";
            } else if (object.type == GameplayObjectType::Decoration) {
                record.rejectionReason = "NO COLLISION PRIMITIVE: NON GAMEPLAY DECORATION";
            } else {
                record.rejectionReason = "NO COLLISION PRIMITIVE";
            }
        }
        result.records.push_back(std::move(record));
    }

    if (runFullInvariantScan) {
        std::unordered_set<std::size_t> expected;
        auto const& primitives = collisionWorld.primitives();
        expected.reserve(primitives.size() / 4 + 1);
        for (std::size_t primitiveIndex = 0; primitiveIndex < primitives.size(); ++primitiveIndex) {
            auto const& primitive = primitives[primitiveIndex];
            if (!primitive.enabled || !primitive.indexable) continue;
            if (intersects(primitive.broadphaseBounds, region)) {
                expected.insert(primitiveIndex);
            }
        }
        result.mainExpectedByFullScan = expected.size();
        for (auto primitiveIndex : expected) {
            if (!mainReturned.contains(primitiveIndex)) ++result.mainMissing;
        }
        for (auto primitiveIndex : mainReturned) {
            if (!expected.contains(primitiveIndex)) ++result.mainUnexpected;
        }
    }

    return result;
}

WorldStructureFingerprint CollisionTrace::fingerprint(StaticWorld const& source) {
    WorldStructureFingerprint result{};
    result.totalObjects = source.objects.size();
    result.structuralHash = 1469598103934665603ull;

    for (std::size_t sourceIndex = 0; sourceIndex < source.objects.size(); ++sourceIndex) {
        auto const& object = source.objects[sourceIndex];
        ++result.categoryCounts[categoryIndex(object.type)];

        // Structural fingerprint deliberately excludes mutable positions so
        // attempt movement does not look like parser nondeterminism.
        fnvMix(result.structuralHash, sourceIndex);
        fnvMix(result.structuralHash, static_cast<std::uint32_t>(object.objectID));
        fnvMix(result.structuralHash, static_cast<std::uint32_t>(object.rawGameObjectType));
        fnvMix(result.structuralHash, static_cast<std::uint32_t>(categoryIndex(object.type)));
        fnvMix(result.structuralHash, static_cast<std::uint32_t>(object.v01Support));
        fnvMix(result.structuralHash, object.enabled ? 1u : 0u);
        fnvMix(result.structuralHash, object.noTouch ? 1u : 0u);
    }
    return result;
}

} // namespace autobot::world
