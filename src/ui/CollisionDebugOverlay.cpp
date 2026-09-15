#include "autobot/ui/CollisionDebugOverlay.hpp"

#include <Geode/Geode.hpp>
#include <fmt/format.h>

#include <algorithm>
#include <unordered_set>

using namespace geode::prelude;

namespace autobot::ui {
namespace {

ccColor4F solidColor() { return {0.15f, 1.0f, 0.25f, 0.90f}; }
ccColor4F hazardColor() { return {1.0f, 0.15f, 0.15f, 0.90f}; }
ccColor4F slopeColor() { return {1.0f, 0.75f, 0.10f, 0.95f}; }
ccColor4F unsupportedColor() { return {0.85f, 0.25f, 1.0f, 0.80f}; }
ccColor4F noTouchColor() { return {0.65f, 0.65f, 0.65f, 0.75f}; }
ccColor4F queryColor() { return {0.15f, 0.75f, 1.0f, 0.65f}; }

world::CollisionPoint rectCenter(world::WorldRect const& rect) {
    return {rect.x + rect.width * 0.5f, rect.y + rect.height * 0.5f};
}

} // namespace

bool CollisionDebugOverlay::attach(PlayLayer* playLayer) {
    if (attached()) return true;
    if (!playLayer || !playLayer->m_objectLayer) return false;
    m_playLayer = playLayer;

    if (!m_drawNode) {
        m_drawNode = CCDrawNode::create();
        if (!m_drawNode) return false;
        m_drawNode->setID("collision-debug-draw"_spr);
        m_drawNode->setZOrder(100000);
        playLayer->m_objectLayer->addChild(m_drawNode, 100000);
    }

    if (!m_label) {
        m_label = CCLabelBMFont::create("COLLISION WORLD: WAITING", "chatFont.fnt");
        if (!m_label) return false;

        const auto winSize = CCDirector::get()->getWinSize();
        m_label->setAnchorPoint({1.0f, 1.0f});
        m_label->setPosition({winSize.width - 8.0f, winSize.height - 8.0f});
        m_label->setScale(0.32f);
        m_label->setZOrder(1000000);
        m_label->setOpacity(235);
        m_label->setID("collision-debug-hud"_spr);
        playLayer->addChild(m_label, 1000000);
    }

    setEnabled(m_enabled);
    return true;
}

void CollisionDebugOverlay::setEnabled(bool enabled) {
    m_enabled = enabled;
    if (m_drawNode) {
        m_drawNode->setVisible(enabled);
        if (!enabled) m_drawNode->clear();
    }
    if (m_label) m_label->setVisible(enabled);
    if (!enabled) hideObjectLabels();
}

void CollisionDebugOverlay::setObjectLabelsEnabled(bool enabled) {
    m_objectLabelsEnabled = enabled;
    if (!enabled) hideObjectLabels();
}

void CollisionDebugOverlay::drawBounds(
    world::WorldRect const& rect,
    ccColor4F const& color,
    float thickness
) {
    if (!m_drawNode) return;

    const float x2 = rect.x + rect.width;
    const float y2 = rect.y + rect.height;
    const float left = std::min(rect.x, x2);
    const float right = std::max(rect.x, x2);
    const float bottom = std::min(rect.y, y2);
    const float top = std::max(rect.y, y2);

    m_drawNode->drawSegment({left, bottom}, {right, bottom}, thickness, color);
    m_drawNode->drawSegment({right, bottom}, {right, top}, thickness, color);
    m_drawNode->drawSegment({right, top}, {left, top}, thickness, color);
    m_drawNode->drawSegment({left, top}, {left, bottom}, thickness, color);
}

void CollisionDebugOverlay::drawCross(float x, float y, ccColor4F const& color, float radius) {
    if (!m_drawNode) return;
    m_drawNode->drawSegment({x - radius, y}, {x + radius, y}, 0.65f, color);
    m_drawNode->drawSegment({x, y - radius}, {x, y + radius}, 0.65f, color);
}

void CollisionDebugOverlay::drawPrimitive(world::CollisionPrimitive const& primitive) {
    if (!m_drawNode || !primitive.enabled) return;

    ccColor4F color = unsupportedColor();
    if (primitive.noTouch) color = noTouchColor();
    else if (primitive.slope) color = slopeColor();
    else if (primitive.classification == world::GameplayObjectType::Solid) color = solidColor();
    else if (primitive.classification == world::GameplayObjectType::Hazard) color = hazardColor();

    // IMPORTANT: only raw OBJECT BOUNDS are drawn in this gate. We no longer
    // rotate getObjectRect() again or fabricate slope triangles.
    drawBounds(primitive.objectBounds, color, 0.75f);
    drawCross(primitive.x, primitive.y, color, 2.25f);
}

void CollisionDebugOverlay::drawTraceRecord(world::CollisionTraceRecord& record) {
    record.rendererReceived = true;

    ccColor4F color = unsupportedColor();
    if (record.noTouch) color = noTouchColor();
    else if (record.classification == world::GameplayObjectType::Solid) {
        color = record.primitiveType == world::CollisionShapeKind::SlopeBounds
            ? slopeColor() : solidColor();
    } else if (record.classification == world::GameplayObjectType::Hazard) {
        color = hazardColor();
    }

    drawBounds(record.objectBounds, color, 0.45f);
    drawCross(record.x, record.y, color, 1.75f);
    record.rendererDrew = true;
}

CCLabelBMFont* CollisionDebugOverlay::ensureObjectLabel(std::size_t index) {
    if (!m_playLayer || !m_playLayer->m_objectLayer) return nullptr;
    while (m_objectLabels.size() <= index) {
        auto* label = CCLabelBMFont::create("", "chatFont.fnt");
        if (!label) return nullptr;
        label->setAnchorPoint({0.5f, 0.0f});
        label->setScale(0.22f);
        label->setOpacity(240);
        label->setZOrder(100001);
        m_playLayer->m_objectLayer->addChild(label, 100001);
        m_objectLabels.push_back(label);
    }
    return m_objectLabels[index];
}

void CollisionDebugOverlay::hideObjectLabels() {
    for (auto* label : m_objectLabels) {
        if (label) label->setVisible(false);
    }
}

void CollisionDebugOverlay::update(
    core::GameSnapshot const& snapshot,
    world::CollisionWorld const& collisionWorld,
    world::CollisionQueryResult const& query,
    world::CollisionTraceQuery* trace
) {
    if (!m_enabled || !attached()) return;

    m_drawNode->clear();
    hideObjectLabels();

    if (!collisionWorld.ready() || !snapshot.valid) {
        m_label->setString("COLLISION WORLD: WAITING");
        return;
    }

    drawBounds(query.region, queryColor(), 0.45f);

    std::unordered_set<std::size_t> mainDrawnSources;
    mainDrawnSources.reserve(query.primitiveIndices.size() * 2 + 1);
    for (auto primitiveIndex : query.primitiveIndices) {
        if (primitiveIndex >= collisionWorld.primitives().size()) continue;
        auto const& primitive = collisionWorld.primitives()[primitiveIndex];
        mainDrawnSources.insert(primitive.sourceIndex);

        if (primitive.classification == world::GameplayObjectType::Solid
            || primitive.classification == world::GameplayObjectType::Hazard) {
            drawPrimitive(primitive);
        }
    }

    constexpr std::size_t kMaxObjectLabels = 24;
    std::size_t labelsUsed = 0;
    if (trace) {
        for (auto& record : trace->records) {
            record.rendererReceived = true;

            // If the main collision renderer did not draw this source, trace
            // mode still shows its object bounds. This makes MISSING distinct
            // from intentionally unsupported/no-touch.
            if (!mainDrawnSources.contains(record.sourceIndex)) {
                drawTraceRecord(record);
            } else {
                record.rendererDrew = true;
            }

            if (m_objectLabelsEnabled && labelsUsed < kMaxObjectLabels) {
                auto* label = ensureObjectLabel(labelsUsed);
                if (label) {
                    const auto center = rectCenter(record.objectBounds);
                    const auto text = fmt::format(
                        "IDX {} ID {} U {}\nRAW {} CLASS {}\n{}{}",
                        record.sourceIndex,
                        record.objectID,
                        record.uniqueID,
                        record.rawGameObjectType,
                        world::toString(record.classification),
                        world::toString(record.support),
                        record.noTouch ? " NO_TOUCH" : ""
                    );
                    label->setString(text.c_str());
                    label->setPosition({center.x, center.y + 3.0f});
                    label->setVisible(true);
                    ++labelsUsed;
                }
            }
        }
    }

    auto const& metrics = collisionWorld.metrics();
    auto const& benchmark = metrics.queryBenchmark;
    const auto text = trace
        ? fmt::format(
            "COLLISION WORLD: READY\n"
            "INDEXED OBJECTS: {}  SILENTLY LOST: {}\n"
            "NEARBY S:{} H:{} SLOPE:{} UNSUP:{}\n"
            "QUERY RECT x:{:.1f} y:{:.1f} w:{:.1f} h:{:.1f}\n"
            "MAIN HASH cells:{} inCells:{} dedup:{} intersect:{} returned:{}\n"
            "MAIN dup:{} invalid:{} invExpected:{} missing:{} unexpected:{}\n"
            "TRACE HASH cells:{} inCells:{} dedup:{} intersect:{} records:{}\n"
            "QUERY: {:.4f} ms  PARSE:{:.3f} CW:{:.3f} HASH:{:.3f} ms\n"
            "BENCH n={} avg:{:.4f} p95:{:.4f} p99:{:.4f} ms\n"
            "GAMEPLAY HITBOXES: NOT VERIFIED",
            metrics.indexedObjects,
            metrics.silentlyLost,
            query.solids,
            query.hazards,
            query.slopes,
            query.unsupported,
            query.region.x,
            query.region.y,
            query.region.width,
            query.region.height,
            query.debug.cellsVisited,
            query.debug.objectsInCells,
            query.debug.objectsAfterDedup,
            query.debug.objectsAfterIntersection,
            query.primitiveIndices.size(),
            trace->duplicateMainResults,
            trace->invalidMainIndices,
            trace->mainExpectedByFullScan,
            trace->mainMissing,
            trace->mainUnexpected,
            trace->diagnosticHashDebug.cellsVisited,
            trace->diagnosticHashDebug.objectsInCells,
            trace->diagnosticHashDebug.objectsAfterDedup,
            trace->diagnosticHashDebug.objectsAfterIntersection,
            trace->records.size(),
            query.queryMs,
            metrics.parseTimeMs,
            metrics.collisionWorldBuildTimeMs,
            metrics.spatialHashBuildTimeMs,
            benchmark.samples,
            benchmark.averageMs,
            benchmark.p95Ms,
            benchmark.p99Ms
        )
        : fmt::format(
            "COLLISION WORLD: READY\n"
            "INDEXED OBJECTS: {}  SILENTLY LOST: {}\n"
            "NEARBY SOLIDS: {}\n"
            "NEARBY HAZARDS: {}\n"
            "NEARBY SLOPES: {}\n"
            "NEARBY UNSUPPORTED: {}\n"
            "QUERY TIME: {:.4f} ms\n"
            "PARSE: {:.3f} ms\n"
            "CW BUILD: {:.3f} ms\n"
            "HASH BUILD: {:.3f} ms\n"
            "QUERY BENCH n={} avg:{:.4f} p95:{:.4f} p99:{:.4f} ms\n"
            "GAMEPLAY HITBOXES: NOT VERIFIED",
            metrics.indexedObjects,
            metrics.silentlyLost,
            query.solids,
            query.hazards,
            query.slopes,
            query.unsupported,
            query.queryMs,
            metrics.parseTimeMs,
            metrics.collisionWorldBuildTimeMs,
            metrics.spatialHashBuildTimeMs,
            benchmark.samples,
            benchmark.averageMs,
            benchmark.p95Ms,
            benchmark.p99Ms
        );
    m_label->setString(text.c_str());
}

} // namespace autobot::ui
