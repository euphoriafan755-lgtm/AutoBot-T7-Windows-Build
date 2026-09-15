#include "autobot/world/ModeTransitionTrace.hpp"
#include "autobot/world/CollisionQueryAudit.hpp"

#include <Geode/Geode.hpp>

#include <unordered_set>

using namespace geode::prelude;

namespace autobot::world {

void ModeTransitionTrace::audit(ModeTransitionSample& sample, CollisionWorld const& collisionWorld) {
    auto fullScan = bruteForceQueryRegion(collisionWorld, sample.queryRect);
    sample.fullScanExpected = fullScan.primitiveIndices.size();

    std::unordered_set<std::size_t> expected(fullScan.primitiveIndices.begin(), fullScan.primitiveIndices.end());
    std::unordered_set<std::size_t> returned;
    returned.reserve(sample.returnedIndices.size() * 2 + 1);
    for (auto index : sample.returnedIndices) {
        if (index >= collisionWorld.primitives().size()) {
            ++sample.invalidIndices;
            continue;
        }
        if (!returned.insert(index).second) ++sample.duplicates;
    }
    for (auto index : expected) if (!returned.contains(index)) ++sample.missing;
    for (auto index : returned) if (!expected.contains(index)) ++sample.unexpected;
}

void ModeTransitionTrace::logSample(
    std::uint64_t transitionID,
    char const* phase,
    ModeTransitionSample const& s
) {
    log::info(
        "MODE_TRANSITION_TRACE id={} phase={} tick={} levelTime={:.6f} mode={} "
        "player=({:.3f},{:.3f}) node=({:.3f},{:.3f}) vel=({:.5f},{:.5f}) "
        "camera=({:.3f},{:.3f}) camera2=({:.3f},{:.3f}) objectLayer=({:.3f},{:.3f}) "
        "scale=({:.4f},{:.4f}) rot={:.3f} query=({:.3f},{:.3f},{:.3f},{:.3f}) "
        "cells={} inCells={} dedup={} intersect={} returned={} fullExpected={} missing={} "
        "unexpected={} duplicates={} invalid={} rendererReceived={} rendererDrawn={} generation={}/{}",
        transitionID, phase, s.sampleTick, s.levelTime, core::toString(s.mode),
        s.playerX, s.playerY, s.playerNodeX, s.playerNodeY, s.velocityX, s.velocityY,
        s.cameraX, s.cameraY, s.camera2X, s.camera2Y, s.objectLayerX, s.objectLayerY,
        s.objectLayerScaleX, s.objectLayerScaleY, s.objectLayerRotation,
        s.queryRect.x, s.queryRect.y, s.queryRect.width, s.queryRect.height,
        s.queryDebug.cellsVisited, s.queryDebug.objectsInCells, s.queryDebug.objectsAfterDedup,
        s.queryDebug.objectsAfterIntersection, s.returnedIndices.size(), s.fullScanExpected,
        s.missing, s.unexpected, s.duplicates, s.invalidIndices,
        s.rendererReceived, s.rendererDrawn, s.collisionWorldGeneration, s.spatialHashGeneration
    );
}

void ModeTransitionTrace::observe(
    PlayLayer* playLayer,
    core::GameSnapshot const& snapshot,
    CollisionWorld const& collisionWorld,
    CollisionQueryResult const& query,
    ui::CollisionRenderStats const& renderStats,
    std::uint64_t worldGeneration
) {
    if (!playLayer || !snapshot.valid || !collisionWorld.ready()) return;

    ModeTransitionSample sample{};
    sample.sampleTick = snapshot.gameTick;
    sample.levelTime = snapshot.levelTime;
    sample.mode = snapshot.player.mode;
    sample.playerX = snapshot.player.x;
    sample.playerY = snapshot.player.y;
    sample.velocityX = snapshot.player.velocityX;
    sample.velocityY = snapshot.player.velocityY;
    if (playLayer->m_player1) {
        auto node = playLayer->m_player1->getPosition();
        sample.playerNodeX = node.x;
        sample.playerNodeY = node.y;
    }
    sample.cameraX = playLayer->m_gameState.m_cameraPosition.x;
    sample.cameraY = playLayer->m_gameState.m_cameraPosition.y;
    sample.camera2X = playLayer->m_gameState.m_cameraPosition2.x;
    sample.camera2Y = playLayer->m_gameState.m_cameraPosition2.y;
    if (playLayer->m_objectLayer) {
        auto layerPos = playLayer->m_objectLayer->getPosition();
        sample.objectLayerX = layerPos.x;
        sample.objectLayerY = layerPos.y;
        sample.objectLayerScaleX = playLayer->m_objectLayer->getScaleX();
        sample.objectLayerScaleY = playLayer->m_objectLayer->getScaleY();
        sample.objectLayerRotation = playLayer->m_objectLayer->getRotation();
    }
    sample.queryRect = query.region;
    sample.queryDebug = query.debug;
    sample.returnedIndices = query.primitiveIndices;
    sample.rendererReceived = renderStats.queryReceived;
    sample.rendererDrawn = renderStats.queryDrawn;
    sample.collisionWorldGeneration = worldGeneration;
    sample.spatialHashGeneration = collisionWorld.spatialHashGeneration();

    const bool changed = m_previousModeValid && snapshot.player.mode != m_previousMode;
    if (changed) {
        ++m_transitionID;
        log::info(
            "MODE_TRANSITION_DETECTED id={} tick={} old={} new={} player=({:.3f},{:.3f})",
            m_transitionID, snapshot.gameTick, core::toString(m_previousMode),
            core::toString(snapshot.player.mode), snapshot.player.x, snapshot.player.y
        );
        for (auto prior : m_history) {
            audit(prior, collisionWorld);
            logSample(m_transitionID, "PRE", prior);
        }
        audit(sample, collisionWorld);
        logSample(m_transitionID, "CHANGE", sample);
        m_afterRemaining = 60;
    } else if (m_afterRemaining > 0) {
        audit(sample, collisionWorld);
        logSample(m_transitionID, "POST", sample);
        --m_afterRemaining;
        if (m_afterRemaining == 0) {
            log::info("TRANSITION_TRACE_CAPTURED id={} postSamples=60", m_transitionID);
        }
    }

    m_history.push_back(std::move(sample));
    while (m_history.size() > 10) m_history.pop_front();
    m_previousMode = snapshot.player.mode;
    m_previousModeValid = true;
}

} // namespace autobot::world
