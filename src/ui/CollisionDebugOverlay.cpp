#include "autobot/ui/CollisionDebugOverlay.hpp"

#include <Geode/Geode.hpp>
#include <fmt/format.h>

#include <algorithm>

using namespace geode::prelude;

namespace autobot::ui {
namespace {

ccColor4F solidColor() {
    return {0.15f, 1.0f, 0.25f, 0.90f};
}

ccColor4F hazardColor() {
    return {1.0f, 0.15f, 0.15f, 0.90f};
}

ccColor4F slopeColor() {
    return {1.0f, 0.75f, 0.10f, 0.95f};
}

ccColor4F unsupportedColor() {
    return {0.85f, 0.25f, 1.0f, 0.75f};
}

ccColor4F queryColor() {
    return {0.15f, 0.75f, 1.0f, 0.65f};
}

CCPoint toPoint(world::CollisionPoint const& point) {
    return {point.x, point.y};
}

} // namespace

bool CollisionDebugOverlay::attach(PlayLayer* playLayer) {
    if (attached()) return true;
    if (!playLayer) return false;

    auto* objectLayer = playLayer->m_objectLayer;
    if (!objectLayer) return false;

    if (!m_drawNode) {
        m_drawNode = CCDrawNode::create();
        if (!m_drawNode) return false;
        m_drawNode->setID("collision-debug-draw"_spr);
        m_drawNode->setZOrder(100000);
        objectLayer->addChild(m_drawNode, 100000);
    }

    if (!m_label) {
        m_label = CCLabelBMFont::create("COLLISION WORLD: WAITING", "chatFont.fnt");
        if (!m_label) return false;

        const auto winSize = CCDirector::get()->getWinSize();
        m_label->setAnchorPoint({1.0f, 1.0f});
        m_label->setPosition({winSize.width - 8.0f, winSize.height - 8.0f});
        m_label->setScale(0.38f);
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

void CollisionDebugOverlay::drawPrimitive(world::CollisionPrimitive const& primitive) {
    if (!m_drawNode || !primitive.enabled) return;

    ccColor4F color = unsupportedColor();
    if (primitive.geometryVerification != world::GeometryVerification::NotSupported) {
        if (primitive.slope) color = slopeColor();
        else if (primitive.classification == world::GameplayObjectType::Solid) color = solidColor();
        else if (primitive.classification == world::GameplayObjectType::Hazard) color = hazardColor();
    }

    if (primitive.vertexCount >= 2) {
        for (std::size_t index = 0; index < primitive.vertexCount; ++index) {
            const auto next = (index + 1) % primitive.vertexCount;
            m_drawNode->drawSegment(
                toPoint(primitive.vertices[index]),
                toPoint(primitive.vertices[next]),
                0.75f,
                color
            );
        }
    } else {
        drawBounds(primitive.objectBounds, color, 0.75f);
    }
}

void CollisionDebugOverlay::update(
    core::GameSnapshot const& snapshot,
    world::CollisionWorld const& collisionWorld,
    world::CollisionQueryResult const& query
) {
    if (!m_enabled || !attached()) return;

    m_drawNode->clear();

    if (!collisionWorld.ready() || !snapshot.valid) {
        m_label->setString("COLLISION WORLD: WAITING");
        return;
    }

    drawBounds(query.region, queryColor(), 0.45f);

    for (auto primitiveIndex : query.primitiveIndices) {
        if (primitiveIndex >= collisionWorld.primitives().size()) continue;
        auto const& primitive = collisionWorld.primitives()[primitiveIndex];

        // Collision overlay only: references such as portals/orbs/pads are
        // indexed for locality but are not drawn as collision surfaces.
        if (primitive.classification != world::GameplayObjectType::Solid
            && primitive.classification != world::GameplayObjectType::Hazard) {
            continue;
        }
        drawPrimitive(primitive);
    }

    auto const& metrics = collisionWorld.metrics();
    auto const& benchmark = metrics.queryBenchmark;
    const auto text = fmt::format(
        "COLLISION WORLD: READY\n"
        "INDEXED OBJECTS: {}\n"
        "COLLISION CANDIDATES: {}\n"
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
        metrics.collisionCandidates,
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
