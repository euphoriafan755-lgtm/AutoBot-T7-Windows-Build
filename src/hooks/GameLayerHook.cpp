#include <Geode/Geode.hpp>
#include <Geode/modify/PlayLayer.hpp>

#include "autobot/core/GameStateReader.hpp"
#include "autobot/ui/CollisionDebugOverlay.hpp"
#include "autobot/ui/DiagnosticHUD.hpp"
#include "autobot/world/CollisionWorld.hpp"
#include "autobot/world/LevelParser.hpp"

#include <algorithm>
#include <chrono>

using namespace geode::prelude;

class $modify(AutoBotT7GameLayerHook, PlayLayer) {
    struct Fields {
        std::uint64_t tick = 0;
        bool parserAttempted = false;
        autobot::world::StaticWorld world{};
        autobot::world::CollisionWorld collisionWorld{};
        autobot::ui::DiagnosticHUD hud{};
        autobot::ui::CollisionDebugOverlay collisionDebug{};
    };

    void postUpdate(float dt) {
        PlayLayer::postUpdate(dt);

        auto* playLayer = PlayLayer::get();
        if (!playLayer || playLayer != this) {
            return;
        }

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
        m_fields->collisionDebug.setEnabled(collisionDebugEnabled);

        if (!m_fields->parserAttempted && playLayer->m_objects && playLayer->m_objects->count() > 0) {
            using Clock = std::chrono::steady_clock;
            const auto parseStart = Clock::now();
            m_fields->world = autobot::world::LevelParser::parse(playLayer);
            const auto parseEnd = Clock::now();
            const double parseTimeMs =
                std::chrono::duration<double, std::milli>(parseEnd - parseStart).count();

            m_fields->parserAttempted = true;

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

                if (m_fields->collisionWorld.build(m_fields->world, parseTimeMs)) {
                    auto const& metrics = m_fields->collisionWorld.metrics();
                    auto const& benchmark = metrics.queryBenchmark;
                    log::info(
                        "AutoBot T7 CollisionWorld: READY indexed={} collisionCandidates={} "
                        "cells={} primitiveBuild={:.3f}ms hashBuild={:.3f}ms total={:.3f}ms",
                        metrics.indexedObjects,
                        metrics.collisionCandidates,
                        metrics.spatialCells,
                        metrics.primitiveBuildTimeMs,
                        metrics.spatialHashBuildTimeMs,
                        metrics.collisionWorldBuildTimeMs
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
                        "(object bounds are broad-phase only; gameplay hitboxes are not assumed)",
                        metrics.notVerifiedShapes,
                        metrics.notSupportedShapes
                    );

                    auto const& audit = m_fields->collisionWorld.classificationAudit();
                    const std::size_t examples = std::min<std::size_t>(12, audit.size());
                    log::info(
                        "AutoBot T7 classification audit: {} distinct object IDs; "
                        "showing {} most frequent IDs without auto-correction",
                        audit.size(),
                        examples
                    );
                    for (std::size_t index = 0; index < examples; ++index) {
                        auto const& entry = audit[index];
                        log::info(
                            "AUDIT objectID={} total={} "
                            "SOLID={} HAZ={} ORB={} PAD={} PORTAL={} DECO={} UNKNOWN={} "
                            "supported={} notSupported={} nonGameplay={} enabled={} disabled={}",
                            entry.objectID,
                            entry.total(),
                            entry.categoryCounts[0],
                            entry.categoryCounts[1],
                            entry.categoryCounts[2],
                            entry.categoryCounts[3],
                            entry.categoryCounts[4],
                            entry.categoryCounts[5],
                            entry.categoryCounts[6],
                            entry.supported,
                            entry.notSupported,
                            entry.nonGameplay,
                            entry.enabled,
                            entry.disabled
                        );
                    }
                } else {
                    log::error("AutoBot T7: CollisionWorld build failed");
                }
            } else {
                log::error("AutoBot T7: LevelParser failed: {}", m_fields->world.error);
            }
        }

        const auto snapshot = autobot::core::GameStateReader::capture(playLayer, m_fields->tick);
        m_fields->hud.update(snapshot, m_fields->world);

        if (m_fields->collisionWorld.ready() && snapshot.valid) {
            const auto query = m_fields->collisionWorld.queryAhead(
                static_cast<float>(snapshot.player.x),
                static_cast<float>(snapshot.player.y)
            );
            m_fields->collisionDebug.update(snapshot, m_fields->collisionWorld, query);
        }
    }
};
