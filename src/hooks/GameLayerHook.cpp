#include <Geode/Geode.hpp>
#include <Geode/modify/PlayLayer.hpp>

#include "autobot/control/AutonomousTestDriver.hpp"
#include "autobot/control/InputController.hpp"
#include "autobot/core/GameStateReader.hpp"
#include "autobot/ui/AutonomousHUD.hpp"
#include "autobot/ui/CollisionDebugOverlay.hpp"
#include "autobot/ui/DiagnosticHUD.hpp"
#include "autobot/world/CollisionTrace.hpp"
#include "autobot/world/CollisionWorld.hpp"
#include "autobot/world/DynamicColliderSync.hpp"
#include "autobot/world/LevelParser.hpp"
#include "autobot/world/ModeTransitionTrace.hpp"

#include <array>
#include <chrono>
#include <optional>

using namespace geode::prelude;

class $modify(AutoBotT7GameLayerHook, PlayLayer) {
    struct Fields {
        std::uint64_t tick = 0;
        std::uint64_t parseTick = 0;
        std::uint64_t worldGeneration = 0;
        bool parserAttempted = false;
        bool traceBuilt = false;
        bool stability1Done = false;
        bool stability5Done = false;
        bool stability30Done = false;
        bool stability120Done = false;
        std::size_t restartStabilitySamples = 0;
        double lastLevelTime = -1.0;

        autobot::world::WorldStructureFingerprint baselineFingerprint{};
        bool baselineFingerprintValid = false;

        autobot::world::StaticWorld world{};
        autobot::world::CollisionWorld collisionWorld{};
        autobot::world::CollisionTrace collisionTrace{};
        autobot::world::DynamicColliderSync dynamicSync{};
        autobot::world::ModeTransitionTrace modeTransitionTrace{};

        autobot::ui::DiagnosticHUD hud{};
        autobot::ui::CollisionDebugOverlay collisionDebug{};
        autobot::ui::AutonomousHUD autonomousHUD{};

        autobot::control::InputController inputController{};
        autobot::control::AutonomousTestDriver autonomousDriver{};
    };

    bool buildWorldFromGameplayLifecycle() {
        if (m_fields->parserAttempted) return m_fields->collisionWorld.ready();
        if (!m_started || !m_objects || m_objects->count() == 0) {
            log::error(
                "WORLD_READY_LIFECYCLE failed: started={} objects={}",
                m_started,
                m_objects ? m_objects->count() : 0
            );
            return false;
        }

        using Clock = std::chrono::steady_clock;
        const auto parseStart = Clock::now();
        m_fields->world = autobot::world::LevelParser::parse(this);
        const auto parseEnd = Clock::now();
        const double parseTimeMs = std::chrono::duration<double, std::milli>(
            parseEnd - parseStart
        ).count();

        m_fields->parserAttempted = true;
        m_fields->parseTick = m_fields->tick;

        if (!m_fields->world.parsed) {
            log::error(
                "AutoBot T7: LevelParser failed at PlayLayer::startGame lifecycle: {}",
                m_fields->world.error
            );
            return false;
        }

        m_fields->baselineFingerprint = autobot::world::CollisionTrace::fingerprint(m_fields->world);
        m_fields->baselineFingerprintValid = true;

        ++m_fields->worldGeneration;
        if (!m_fields->collisionWorld.build(m_fields->world, parseTimeMs)) {
            log::error("AutoBot T7: CollisionWorld build failed at PlayLayer::startGame lifecycle");
            return false;
        }

        auto const& metrics = m_fields->collisionWorld.metrics();
        auto const& benchmark = metrics.queryBenchmark;
        log::info(
            "WORLD_READY_LIFECYCLE source=startGame started={} tick={} objects={} structuralHash={} parseMs={:.3f}",
            m_started,
            m_fields->tick,
            m_fields->baselineFingerprint.totalObjects,
            m_fields->baselineFingerprint.structuralHash,
            parseTimeMs
        );
        log::info(
            "AutoBot T7 CollisionWorld: READY indexed={} collisionCandidates={} cells={} "
            "primitiveBuild={:.3f}ms hashBuild={:.3f}ms total={:.3f}ms silentlyLost={} worldGeneration={} hashGeneration={}",
            metrics.indexedObjects,
            metrics.collisionCandidates,
            metrics.spatialCells,
            metrics.primitiveBuildTimeMs,
            metrics.spatialHashBuildTimeMs,
            metrics.collisionWorldBuildTimeMs,
            metrics.silentlyLost,
            m_fields->worldGeneration,
            m_fields->collisionWorld.spatialHashGeneration()
        );
        log::info(
            "AutoBot T7 SpatialHash benchmark: n={} avg={:.4f}ms p95={:.4f}ms p99={:.4f}ms",
            benchmark.samples,
            benchmark.averageMs,
            benchmark.p95Ms,
            benchmark.p99Ms
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
                "CONSISTENCY {} parsed={} supported={} colliderCreated={} indexed={} intentionallySkipped={} silentlyLost={}",
                autobot::world::toString(categories[i]),
                c.parsed,
                c.supported,
                c.colliderCreated,
                c.indexed,
                c.intentionallySkipped,
                c.silentlyLost
            );
        }
        return true;
    }

    void startGame() {
        PlayLayer::startGame();
        buildWorldFromGameplayLifecycle();
    }

    void postUpdate(float dt) {
        PlayLayer::postUpdate(dt);

        auto* playLayer = PlayLayer::get();
        if (!playLayer || playLayer != this) return;
        ++m_fields->tick;

        if (!m_fields->hud.attached() && !m_fields->hud.attach(playLayer)) {
            log::warn("AutoBot T7: diagnostic HUD could not be attached yet");
        }
        if (!m_fields->collisionDebug.attached() && !m_fields->collisionDebug.attach(playLayer)) {
            log::warn("AutoBot T7: collision debug overlay could not be attached yet");
        }
        if (!m_fields->autonomousHUD.attached() && !m_fields->autonomousHUD.attach(playLayer)) {
            log::warn("AutoBot T7: autonomous HUD could not be attached yet");
        }

        const bool collisionDebugEnabled = Mod::get()->getSettingValue<bool>("collision-debug-overlay");
        const bool traceDebugEnabled = Mod::get()->getSettingValue<bool>("collision-trace-debug");
        const bool objectLabelsEnabled = Mod::get()->getSettingValue<bool>("collision-object-labels");
        const bool fullWorldDebug = Mod::get()->getSettingValue<bool>("collision-debug-full-world");
        const bool autobotEnabled = Mod::get()->getSettingValue<bool>("autobot-enabled");

        m_fields->collisionDebug.setEnabled(collisionDebugEnabled);
        m_fields->collisionDebug.setObjectLabelsEnabled(
            collisionDebugEnabled && traceDebugEnabled && objectLabelsEnabled
        );
        m_fields->collisionDebug.setFullWorldEnabled(collisionDebugEnabled && fullWorldDebug);

        const auto snapshot = autobot::core::GameStateReader::capture(playLayer, m_fields->tick);
        m_fields->hud.update(snapshot, m_fields->world);

        autobot::world::DynamicSyncStats syncStats{};
        if (m_fields->collisionWorld.ready()) {
            syncStats = m_fields->dynamicSync.sync(
                playLayer,
                m_fields->world,
                m_fields->collisionWorld
            );
            if ((syncStats.changed > 0 || syncStats.identityMismatches > 0 || syncStats.readFailures > 0)
                && (traceDebugEnabled || m_fields->tick % 60u == 0u)) {
                log::info(
                    "DYNAMIC_COLLIDER_SYNC tick={} watched={} changed={} reindexed={} readFailures={} "
                    "identityMismatches={} syncMs={:.4f} hashGeneration={}",
                    m_fields->tick,
                    syncStats.watched,
                    syncStats.changed,
                    syncStats.reindexed,
                    syncStats.readFailures,
                    syncStats.identityMismatches,
                    syncStats.syncMs,
                    m_fields->collisionWorld.spatialHashGeneration()
                );
            }
        }

        if (traceDebugEnabled && m_fields->collisionWorld.ready()) {
            if (!m_fields->traceBuilt || syncStats.changed > 0) {
                m_fields->traceBuilt = m_fields->collisionTrace.build(
                    m_fields->world,
                    m_fields->collisionWorld
                );
                log::info(
                    "CollisionTrace build/rebuild: {} dynamicChanged={}",
                    m_fields->traceBuilt ? "READY" : "FAILED",
                    syncStats.changed
                );
            }
        } else if (!traceDebugEnabled && m_fields->traceBuilt) {
            m_fields->collisionTrace.clear();
            m_fields->traceBuilt = false;
        }

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
            if (!m_fields->stability1Done && age >= 1) {
                m_fields->stability1Done = true;
                logStabilitySample("TICK+1");
            }
            if (!m_fields->stability5Done && age >= 5) {
                m_fields->stability5Done = true;
                logStabilitySample("TICK+5");
            }
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

        autobot::world::CollisionQueryResult query{};
        std::optional<autobot::world::CollisionTraceQuery> traceQuery;
        if (m_fields->collisionWorld.ready() && snapshot.valid) {
            query = m_fields->collisionWorld.queryAhead(
                static_cast<float>(snapshot.player.x),
                static_cast<float>(snapshot.player.y)
            );

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
        }

        m_fields->collisionDebug.update(
            snapshot,
            m_fields->collisionWorld,
            query,
            traceQuery ? &*traceQuery : nullptr
        );

        if (traceQuery && (m_fields->tick % 30u == 0u)) {
            log::info(
                "QUERY_AUDIT cells={} inCells={} dedup={} intersection={} returned={} expected={} "
                "missing={} unexpected={} duplicates={} invalid={}",
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
        }

        if (snapshot.valid && m_fields->collisionWorld.ready()) {
            m_fields->modeTransitionTrace.observe(
                playLayer,
                snapshot,
                m_fields->collisionWorld,
                query,
                m_fields->collisionDebug.lastStats(),
                m_fields->worldGeneration
            );
        }

        const double inputTimestamp = snapshot.valid
            ? snapshot.levelTime
            : m_gameState.m_levelTime;
        m_fields->inputController.setBotControl(autobotEnabled, playLayer, inputTimestamp);
        auto decision = m_fields->autonomousDriver.decide(
            snapshot,
            m_fields->collisionWorld,
            query,
            autobotEnabled,
            m_fields->inputController.botHolding()
        );

        if (autobotEnabled) {
            if (decision.ownership == autobot::control::InputOwnership::Bot) {
                m_fields->inputController.setBotControl(true, playLayer, inputTimestamp);
            }
            m_fields->inputController.apply(decision.action, playLayer, inputTimestamp);
            decision.ownership = m_fields->inputController.ownership();
            decision.active = decision.ownership == autobot::control::InputOwnership::Bot;
        }
        m_fields->autonomousHUD.update(snapshot, decision);
    }
};
