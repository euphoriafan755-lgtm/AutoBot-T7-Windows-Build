#include <Geode/Geode.hpp>
#include <Geode/modify/PlayLayer.hpp>
#include <fmt/format.h>

#include "autobot/core/GameStateReader.hpp"
#include "autobot/ui/CollisionDebugOverlay.hpp"
#include "autobot/ui/DiagnosticHUD.hpp"
#include "autobot/world/CollisionTrace.hpp"
#include "autobot/world/CollisionWorld.hpp"
#include "autobot/world/LevelParser.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <optional>
#include <string>

using namespace geode::prelude;

class $modify(AutoBotT7GameLayerHook, PlayLayer) {
    struct Fields {
        std::uint64_t tick = 0;
        std::uint64_t parseTick = 0;
        bool parserAttempted = false;
        bool traceBuilt = false;
        bool stability30Done = false;
        bool stability120Done = false;
        std::size_t restartStabilitySamples = 0;
        double lastLevelTime = -1.0;
        autobot::world::WorldStructureFingerprint baselineFingerprint{};
        bool baselineFingerprintValid = false;
        autobot::world::StaticWorld world{};
        autobot::world::CollisionWorld collisionWorld{};
        autobot::world::CollisionTrace collisionTrace{};
        autobot::ui::DiagnosticHUD hud{};
        autobot::ui::CollisionDebugOverlay collisionDebug{};
    };

    void postUpdate(float dt) {
        PlayLayer::postUpdate(dt);

        auto* playLayer = PlayLayer::get();
        if (!playLayer || playLayer != this) return;

        ++m_fields->tick;

        if (!m_fields->hud.attached()) {
            if (!m_fields->hud.attach(playLayer)) {
                log::warn("AutoBot T7: diagnostic HUD could not be attached yet");
            }
        }

        if (!m_fields->collisionDebug.attached()) {
            if (!m_fields->collisionDebug.attach(playLayer)) {
                log::warn("AutoBot T7: collision debug overlay could not be attached yet");
            }
        }

        const bool collisionDebugEnabled =
            Mod::get()->getSettingValue<bool>("collision-debug-overlay");
        const bool traceDebugEnabled =
            Mod::get()->getSettingValue<bool>("collision-trace-debug");
        const bool objectLabelsEnabled =
            Mod::get()->getSettingValue<bool>("collision-object-labels");

        m_fields->collisionDebug.setEnabled(collisionDebugEnabled);
        m_fields->collisionDebug.setObjectLabelsEnabled(
            collisionDebugEnabled && traceDebugEnabled && objectLabelsEnabled
        );

        if (!m_fields->parserAttempted && playLayer->m_objects && playLayer->m_objects->count() > 0) {
            using Clock = std::chrono::steady_clock;
            const auto parseStart = Clock::now();
            m_fields->world = autobot::world::LevelParser::parse(playLayer);
            const auto parseEnd = Clock::now();
            const double parseTimeMs =
                std::chrono::duration<double, std::milli>(parseEnd - parseStart).count();

            m_fields->parserAttempted = true;
            m_fields->parseTick = m_fields->tick;

            if (m_fields->world.parsed) {
                log::info(
                    "AutoBot T7: parsed {} objects in {:.3f} ms "
                    "(solid={}, hazard={}, orb={}, pad={}, portal={}, decoration={}, unknown={})",
                    m_fields->world.objects.size(),
                    parseTimeMs,
                    m_fields->world.solids,
                    m_fields->world.hazards,
                    m_fields->world.orbs,
                    m_fields->world.pads,
                    m_fields->world.portals,
                    m_fields->world.decorations,
                    m_fields->world.unknown
                );

                m_fields->baselineFingerprint =
                    autobot::world::CollisionTrace::fingerprint(m_fields->world);
                m_fields->baselineFingerprintValid = true;
                log::info(
                    "PARSE_BASELINE total={} structuralHash={}",
                    m_fields->baselineFingerprint.totalObjects,
                    m_fields->baselineFingerprint.structuralHash
                );

                if (m_fields->collisionWorld.build(m_fields->world, parseTimeMs)) {
                    auto const& metrics = m_fields->collisionWorld.metrics();
                    auto const& benchmark = metrics.queryBenchmark;
                    log::info(
                        "AutoBot T7 CollisionWorld: READY indexed={} collisionCandidates={} "
                        "cells={} primitiveBuild={:.3f}ms hashBuild={:.3f}ms total={:.3f}ms silentlyLost={}",
                        metrics.indexedObjects,
                        metrics.collisionCandidates,
                        metrics.spatialCells,
                        metrics.primitiveBuildTimeMs,
                        metrics.spatialHashBuildTimeMs,
                        metrics.collisionWorldBuildTimeMs,
                        metrics.silentlyLost
                    );
                    log::info(
                        "AutoBot T7 SpatialHash benchmark: n={} avg={:.4f}ms p95={:.4f}ms p99={:.4f}ms",
                        benchmark.samples,
                        benchmark.averageMs,
                        benchmark.p95Ms,
                        benchmark.p99Ms
                    );
                    log::info(
                        "AutoBot T7 geometry status: NOT_VERIFIED={} NOT_SUPPORTED={} "
                        "(OBJECT BOUNDS only; exact gameplay hitboxes NOT VERIFIED)",
                        metrics.notVerifiedShapes,
                        metrics.notSupportedShapes
                    );

                    constexpr std::array<autobot::world::GameplayObjectType, 7> categories{{
                        autobot::world::GameplayObjectType::Solid,
                        autobot::world::GameplayObjectType::Hazard,
                        autobot::world::GameplayObjectType::Orb,
                        autobot::world::GameplayObjectType::Pad,
                        autobot::world::GameplayObjectType::Portal,
                        autobot::world::GameplayObjectType::Decoration,
                        autobot::world::GameplayObjectType::Unknown,
                    }};
                    for (std::size_t i = 0; i < categories.size(); ++i) {
                        auto const& c = metrics.consistency[i];
                        log::info(
                            "CONSISTENCY {} parsed={} supported={} colliderCreated={} indexed={} "
                            "intentionallySkipped={} silentlyLost={}",
                            autobot::world::toString(categories[i]),
                            c.parsed,
                            c.supported,
                            c.colliderCreated,
                            c.indexed,
                            c.intentionallySkipped,
                            c.silentlyLost
                        );
                    }

                    for (auto const& entry : m_fields->collisionWorld.rawClassificationAudit()) {
                        log::info(
                            "RAW_AUDIT raw={} total={} -> SOLID={} HAZ={} ORB={} PAD={} PORTAL={} DECO={} UNKNOWN={} "
                            "noTouch={} samples=[{}@{},{}@{},{}@{}]",
                            entry.rawGameObjectType,
                            entry.total,
                            entry.finalCategoryCounts[0],
                            entry.finalCategoryCounts[1],
                            entry.finalCategoryCounts[2],
                            entry.finalCategoryCounts[3],
                            entry.finalCategoryCounts[4],
                            entry.finalCategoryCounts[5],
                            entry.finalCategoryCounts[6],
                            entry.noTouch,
                            entry.sampleObjectIDs[0], entry.sampleSourceIndices[0],
                            entry.sampleObjectIDs[1], entry.sampleSourceIndices[1],
                            entry.sampleObjectIDs[2], entry.sampleSourceIndices[2]
                        );
                    }
                } else {
                    log::error("AutoBot T7: CollisionWorld build failed");
                }
            } else {
                log::error("AutoBot T7: LevelParser failed: {}", m_fields->world.error);
            }
        }

        if (traceDebugEnabled && m_fields->collisionWorld.ready() && !m_fields->traceBuilt) {
            m_fields->traceBuilt = m_fields->collisionTrace.build(
                m_fields->world,
                m_fields->collisionWorld
            );
            log::info(
                "CollisionTrace build: {}",
                m_fields->traceBuilt ? "READY" : "FAILED"
            );
        } else if (!traceDebugEnabled && m_fields->traceBuilt) {
            m_fields->collisionTrace.clear();
            m_fields->traceBuilt = false;
        }

        const auto snapshot = autobot::core::GameStateReader::capture(playLayer, m_fields->tick);
        m_fields->hud.update(snapshot, m_fields->world);

        auto logStabilitySample = [&](char const* label) {
            if (!traceDebugEnabled || !m_fields->baselineFingerprintValid) return;
            auto sampleWorld = autobot::world::LevelParser::parse(playLayer);
            if (!sampleWorld.parsed) {
                log::error("PARSE_STABILITY {} parse failed: {}", label, sampleWorld.error);
                return;
            }
            const auto sample = autobot::world::CollisionTrace::fingerprint(sampleWorld);
            const bool same = sample == m_fields->baselineFingerprint;
            log::info(
                "PARSE_STABILITY {} same={} baselineTotal={} sampleTotal={} baselineHash={} sampleHash={} "
                "S={} H={} O={} P={} PORT={} DECO={} U={}",
                label,
                same,
                m_fields->baselineFingerprint.totalObjects,
                sample.totalObjects,
                m_fields->baselineFingerprint.structuralHash,
                sample.structuralHash,
                sample.categoryCounts[0],
                sample.categoryCounts[1],
                sample.categoryCounts[2],
                sample.categoryCounts[3],
                sample.categoryCounts[4],
                sample.categoryCounts[5],
                sample.categoryCounts[6]
            );
        };

        if (traceDebugEnabled && m_fields->parserAttempted) {
            const auto age = m_fields->tick - m_fields->parseTick;
            if (!m_fields->stability30Done && age >= 30) {
                m_fields->stability30Done = true;
                logStabilitySample("TICK+30");
            }
            if (!m_fields->stability120Done && age >= 120) {
                m_fields->stability120Done = true;
                logStabilitySample("TICK+120");
            }
            if (snapshot.valid && m_fields->lastLevelTime > 0.75
                && snapshot.levelTime + 0.25 < m_fields->lastLevelTime
                && m_fields->restartStabilitySamples < 5) {
                ++m_fields->restartStabilitySamples;
                logStabilitySample("ATTEMPT_RESTART");
            }
        }
        if (snapshot.valid) m_fields->lastLevelTime = snapshot.levelTime;

        if (m_fields->collisionWorld.ready() && snapshot.valid) {
            auto query = m_fields->collisionWorld.queryAhead(
                static_cast<float>(snapshot.player.x),
                static_cast<float>(snapshot.player.y)
            );

            std::optional<autobot::world::CollisionTraceQuery> traceQuery;
            const bool fullInvariantScan = traceDebugEnabled && (m_fields->tick % 30u == 0u);
            if (traceDebugEnabled && m_fields->collisionTrace.ready()) {
                traceQuery = m_fields->collisionTrace.query(
                    query.region,
                    m_fields->world,
                    m_fields->collisionWorld,
                    query,
                    fullInvariantScan
                );
            }

            m_fields->collisionDebug.update(
                snapshot,
                m_fields->collisionWorld,
                query,
                traceQuery ? &*traceQuery : nullptr
            );

            if (traceQuery && fullInvariantScan) {
                log::info(
                    "QUERY_AUDIT cells={} inCells={} dedup={} intersection={} returned={} "
                    "expected={} missing={} unexpected={} duplicates={} invalid={}",
                    query.debug.cellsVisited,
                    query.debug.objectsInCells,
                    query.debug.objectsAfterDedup,
                    query.debug.objectsAfterIntersection,
                    query.primitiveIndices.size(),
                    traceQuery->mainExpectedByFullScan,
                    traceQuery->mainMissing,
                    traceQuery->mainUnexpected,
                    traceQuery->duplicateMainResults,
                    traceQuery->invalidMainIndices
                );

                std::size_t logged = 0;
                for (auto const& record : traceQuery->records) {
                    if (record.rejectionReason.empty()) continue;
                    std::string cellPreview = "[";
                    const auto cellCount = std::min<std::size_t>(record.spatialCells.size(), 8);
                    for (std::size_t cellIndex = 0; cellIndex < cellCount; ++cellIndex) {
                        if (cellIndex != 0) cellPreview += ",";
                        auto const& cell = record.spatialCells[cellIndex];
                        cellPreview += fmt::format("({},{})", cell.x, cell.y);
                    }
                    if (record.spatialCells.size() > cellCount) cellPreview += ",...";
                    cellPreview += "]";

                    log::info(
                        "TRACE idx={} unique={} objectID={} raw={} class={} support={} "
                        "pos=({:.2f},{:.2f}) node=({:.2f},{:.2f}) rot={:.2f}/{:.2f}/{:.2f} "
                        "scale=({:.3f},{:.3f}) anchor=({:.2f},{:.2f}) size=({:.1f},{:.1f}) flip=({},{}) noTouch={} passable={} groupDisabled={} "
                        "bounds=({:.2f},{:.2f},{:.2f},{:.2f}) primitive={} primIndex={} hash={} "
                        "cells={} cellList={} intersects={} returned={} rendererReceived={} rendererDrew={} reason={}",
                        record.sourceIndex,
                        record.uniqueID,
                        record.objectID,
                        record.rawGameObjectType,
                        autobot::world::toString(record.classification),
                        autobot::world::toString(record.support),
                        record.x,
                        record.y,
                        record.nodeX,
                        record.nodeY,
                        record.rotation,
                        record.rotationX,
                        record.rotationY,
                        record.scaleX,
                        record.scaleY,
                        record.anchorX,
                        record.anchorY,
                        record.contentWidth,
                        record.contentHeight,
                        record.flipX,
                        record.flipY,
                        record.noTouch,
                        record.passable,
                        record.groupDisabled,
                        record.objectBounds.x,
                        record.objectBounds.y,
                        record.objectBounds.width,
                        record.objectBounds.height,
                        record.primitiveCreated,
                        record.primitiveIndex,
                        record.insertedIntoSpatialHash,
                        record.spatialCells.size(),
                        cellPreview,
                        record.intersectsCurrentQuery,
                        record.returnedByQuery,
                        record.rendererReceived,
                        record.rendererDrew,
                        record.rejectionReason
                    );
                    if (++logged >= 16) break;
                }
            }
        }
    }
};
