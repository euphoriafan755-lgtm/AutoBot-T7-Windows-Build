#include <Geode/Geode.hpp>
#include <Geode/modify/GJBaseGameLayer.hpp>
#include <Geode/modify/PlayLayer.hpp>

#include "autobot/control/AutonomousTestDriver.hpp"
#include "autobot/control/InputController.hpp"
#include "autobot/core/GameStateReader.hpp"
#include "autobot/core/PerformanceMetrics.hpp"
#include "autobot/presolve/UniversalRuntimeSession.hpp"
#include "autobot/presolve/UniversalSearchRuntimeLiveness.hpp"
#include "autobot/ui/AutonomousHUD.hpp"
#include "autobot/ui/CollisionDebugOverlay.hpp"
#include "autobot/ui/DiagnosticHUD.hpp"
#include "autobot/ui/ShowTrajectoryOverlay.hpp"
#include "autobot/world/CollisionTrace.hpp"
#include "autobot/world/CollisionWorld.hpp"
#include "autobot/world/DynamicColliderSync.hpp"
#include "autobot/world/LevelParser.hpp"
#include "autobot/world/ModeTransitionTrace.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <string_view>

using namespace geode::prelude;

namespace {
using PerfClock = std::chrono::steady_clock;

double elapsedMs(PerfClock::time_point start) {
    return std::chrono::duration<double, std::milli>(PerfClock::now() - start).count();
}

std::string_view gameObjectTypeName(int rawType) {
    switch (static_cast<GameObjectType>(rawType)) {
        case GameObjectType::Solid: return "Solid";
        case GameObjectType::Hazard: return "Hazard";
        case GameObjectType::InverseGravityPortal: return "InverseGravityPortal";
        case GameObjectType::NormalGravityPortal: return "NormalGravityPortal";
        case GameObjectType::ShipPortal: return "ShipPortal";
        case GameObjectType::CubePortal: return "CubePortal";
        case GameObjectType::Decoration: return "Decoration";
        case GameObjectType::YellowJumpPad: return "YellowJumpPad";
        case GameObjectType::PinkJumpPad: return "PinkJumpPad";
        case GameObjectType::GravityPad: return "GravityPad";
        case GameObjectType::YellowJumpRing: return "YellowJumpRing";
        case GameObjectType::PinkJumpRing: return "PinkJumpRing";
        case GameObjectType::GravityRing: return "GravityRing";
        case GameObjectType::InverseMirrorPortal: return "InverseMirrorPortal";
        case GameObjectType::NormalMirrorPortal: return "NormalMirrorPortal";
        case GameObjectType::BallPortal: return "BallPortal";
        case GameObjectType::RegularSizePortal: return "RegularSizePortal";
        case GameObjectType::MiniSizePortal: return "MiniSizePortal";
        case GameObjectType::UfoPortal: return "UfoPortal";
        case GameObjectType::Modifier: return "Modifier";
        case GameObjectType::Breakable: return "Breakable";
        case GameObjectType::SecretCoin: return "SecretCoin";
        case GameObjectType::DualPortal: return "DualPortal";
        case GameObjectType::SoloPortal: return "SoloPortal";
        case GameObjectType::Slope: return "Slope";
        case GameObjectType::WavePortal: return "WavePortal";
        case GameObjectType::RobotPortal: return "RobotPortal";
        case GameObjectType::TeleportPortal: return "TeleportPortal";
        case GameObjectType::GreenRing: return "GreenRing";
        case GameObjectType::Collectible: return "Collectible";
        case GameObjectType::UserCoin: return "UserCoin";
        case GameObjectType::DropRing: return "DropRing";
        case GameObjectType::SpiderPortal: return "SpiderPortal";
        case GameObjectType::RedJumpPad: return "RedJumpPad";
        case GameObjectType::RedJumpRing: return "RedJumpRing";
        case GameObjectType::CustomRing: return "CustomRing";
        case GameObjectType::DashRing: return "DashRing";
        case GameObjectType::GravityDashRing: return "GravityDashRing";
        case GameObjectType::CollisionObject: return "CollisionObject";
        case GameObjectType::Special: return "Special";
        case GameObjectType::SwingPortal: return "SwingPortal";
        case GameObjectType::GravityTogglePortal: return "GravityTogglePortal";
        case GameObjectType::SpiderOrb: return "SpiderOrb";
        case GameObjectType::SpiderPad: return "SpiderPad";
        case GameObjectType::EnterEffectObject: return "EnterEffectObject";
        case GameObjectType::TeleportOrb: return "TeleportOrb";
        case GameObjectType::AnimatedHazard: return "AnimatedHazard";
    }
    return "UnknownRawType";
}

struct AuthoritativeFreezeProbe {
    PlayLayer* owner = nullptr;
    bool active = false;
    bool passLogged = false;
    bool failLogged = false;
    std::uint64_t gjBaseUpdateCalls = 0;
    std::uint64_t playLayerPostUpdateCalls = 0;
    std::uint64_t deathTransitions = 0;
    bool lastDead = false;
    autobot::presolve::FreezeInvariantSnapshot anchor{};
    PerfClock::time_point started = PerfClock::now();
};

AuthoritativeFreezeProbe g_authoritativeFreeze{};
bool g_universalInternalStep = false;
autobot::presolve::UniversalRuntimeSession g_universalRuntime{};
autobot::presolve::UniversalSearchRuntimeLiveness g_universalSearchLiveness{};
PerfClock::time_point g_universalSearchLivenessStarted = PerfClock::now();
std::size_t g_universalRuntimeCallDepth = 0;
std::string_view g_universalRuntimeCallOperation = "none";

char const* runtimeStageName(autobot::presolve::UniversalRuntimeStage stage) {
    switch (stage) {
        case autobot::presolve::UniversalRuntimeStage::Idle: return "Idle";
        case autobot::presolve::UniversalRuntimeStage::Searching: return "Searching";
        case autobot::presolve::UniversalRuntimeStage::Ready: return "Ready";
        case autobot::presolve::UniversalRuntimeStage::Playing: return "Playing";
        case autobot::presolve::UniversalRuntimeStage::Complete: return "Complete";
        case autobot::presolve::UniversalRuntimeStage::Error: return "Error";
    }
    return "Unknown";
}

struct UniversalRuntimeCallScope {
    std::string_view previousOperation;

    explicit UniversalRuntimeCallScope(std::string_view operation)
      : previousOperation(g_universalRuntimeCallOperation) {
        ++g_universalRuntimeCallDepth;
        g_universalRuntimeCallOperation = operation;
    }

    ~UniversalRuntimeCallScope() {
        if (g_universalRuntimeCallDepth > 0) --g_universalRuntimeCallDepth;
        g_universalRuntimeCallOperation = previousOperation;
    }
};
bool g_universalSearchLivenessPassLogged = false;
bool g_universalSearchLivenessFailLogged = false;
std::size_t g_universalSearchSliceBudget = 8;
double g_universalSearchLastSliceMs = 0.0;
std::uint64_t g_liveSimulationSteps = 0;
std::uint64_t g_liveRealUpdates = 0;
bool g_liveRoundTripLogged = false;
autobot::ui::AutonomousHUD* g_universalHUD = nullptr;

autobot::presolve::FreezeInvariantSnapshot captureAuthoritativeFreezeInvariant(PlayLayer* layer) {
    autobot::presolve::FreezeInvariantSnapshot snapshot{};
    if (!layer) return snapshot;
    if (layer->m_player1) snapshot.playerX = static_cast<double>(layer->m_player1->getRealPosition().x);
    snapshot.progress = static_cast<double>(layer->getCurrentPercent());
    snapshot.levelTime = static_cast<double>(layer->m_gameState.m_levelTime);
    snapshot.attempts = layer->m_attempts;
    snapshot.dead = layer->m_player1 ? layer->m_player1->m_isDead : false;
    return snapshot;
}

autobot::presolve::SearchVisibleState captureSearchVisibleState(PlayLayer* layer) {
    autobot::presolve::SearchVisibleState state{};
    if (!layer) return state;
    if (layer->m_player1) state.playerX = static_cast<double>(layer->m_player1->getRealPosition().x);
    state.progress = static_cast<double>(layer->getCurrentPercent());
    state.levelTime = static_cast<double>(layer->m_gameState.m_levelTime);
    state.attempts = layer->m_attempts;
    state.dead = layer->m_player1 ? layer->m_player1->m_isDead : false;
    return state;
}

void activateAuthoritativeFreeze(PlayLayer* layer) {
    g_authoritativeFreeze = {};
    g_authoritativeFreeze.owner = layer;
    g_authoritativeFreeze.active = layer != nullptr;
    g_authoritativeFreeze.anchor = captureAuthoritativeFreezeInvariant(layer);
    g_authoritativeFreeze.lastDead = g_authoritativeFreeze.anchor.dead;
    g_authoritativeFreeze.started = PerfClock::now();
    if (layer) {
        log::info(
            "PRE_RUN_FREEZE_DRIVER authoritative=GJBaseGameLayer::update postUpdate=PlayLayer::postUpdate "
            "playerX={:.6f} progress={:.6f} levelTime={:.6f} attempts={}",
            g_authoritativeFreeze.anchor.playerX,
            g_authoritativeFreeze.anchor.progress,
            g_authoritativeFreeze.anchor.levelTime,
            g_authoritativeFreeze.anchor.attempts
        );
    }
}

void releaseAuthoritativeFreeze(PlayLayer* layer) {
    if (g_authoritativeFreeze.owner != layer) return;
    g_authoritativeFreeze.active = false;
}

bool authoritativeFreezeOwns(GJBaseGameLayer* layer) {
    return g_authoritativeFreeze.active
        && g_authoritativeFreeze.owner
        && static_cast<GJBaseGameLayer*>(g_authoritativeFreeze.owner) == layer;
}

void verifyAuthoritativeFreezeRuntime(PlayLayer* layer) {
    if (!layer || !g_authoritativeFreeze.active || g_authoritativeFreeze.owner != layer) return;
    const auto current = captureAuthoritativeFreezeInvariant(layer);
    if (current.dead && !g_authoritativeFreeze.lastDead) ++g_authoritativeFreeze.deathTransitions;
    g_authoritativeFreeze.lastDead = current.dead;

    const double playerXDelta = current.playerX - g_authoritativeFreeze.anchor.playerX;
    const double progressDelta = current.progress - g_authoritativeFreeze.anchor.progress;
    const double levelTimeDelta = current.levelTime - g_authoritativeFreeze.anchor.levelTime;
    const int attemptsDelta = current.attempts - g_authoritativeFreeze.anchor.attempts;
    const bool holds = std::abs(playerXDelta) <= 0.0001
        && std::abs(progressDelta) <= 0.0001
        && std::abs(levelTimeDelta) <= 0.0001
        && attemptsDelta == 0
        && g_authoritativeFreeze.deathTransitions == 0
        && !current.dead;

    if (!holds && !g_authoritativeFreeze.failLogged) {
        g_authoritativeFreeze.failLogged = true;
        log::error(
            "PRE_RUN_FREEZE_RUNTIME=FAIL playerXDelta={:.6f} levelTimeDelta={:.6f} progressDelta={:.6f} "
            "deaths={} attemptsDelta={} GJBaseGameLayerUpdateCalls={} PlayLayerPostUpdateCalls={}",
            playerXDelta,
            levelTimeDelta,
            progressDelta,
            g_authoritativeFreeze.deathTransitions,
            attemptsDelta,
            g_authoritativeFreeze.gjBaseUpdateCalls,
            g_authoritativeFreeze.playLayerPostUpdateCalls
        );
    }

    if (holds && !g_authoritativeFreeze.passLogged && elapsedMs(g_authoritativeFreeze.started) >= 5000.0) {
        g_authoritativeFreeze.passLogged = true;
        log::info(
            "PRE_RUN_FREEZE_RUNTIME=PASS playerXDelta=0 levelTimeDelta=0 progressDelta=0 deaths=0 attemptsDelta=0 "
            "GJBaseGameLayerUpdateCalls={} PlayLayerPostUpdateCalls={}",
            g_authoritativeFreeze.gjBaseUpdateCalls,
            g_authoritativeFreeze.playLayerPostUpdateCalls
        );
    }
}

void logUniversalRuntimeValidation() {
    const auto v = g_universalRuntime.validation();
    log::info(
        "RUNTIME_INTEGRATION_TEST_MODE=ACTIVE roundTripChecks={} recoveryCount={}",
        v.roundTripChecks,
        g_universalRuntime.recoveryCount()
    );
    log::info(
        "CHECKPOINT_ROUNDTRIP_TEST={} checks={}",
        (v.roundTripChecks > 0 && v.roundTripPass) ? "PASS" : "FAIL",
        v.roundTripChecks
    );
    log::info(
        "RNG_RESTORE_TEST={} checks={} transitionsObserved={}",
        !v.rngPass ? "FAIL" : (v.rngTransitionsObserved > 0 ? "PASS" : "NOT_EXERCISED"),
        v.rngChecks,
        v.rngTransitionsObserved
    );
    log::info(
        "EFFECT_STATE_RESTORE_TEST={} checks={} transitionsObserved={}",
        !v.effectStatePass ? "FAIL" : (v.effectTransitionsObserved > 0 ? "PASS" : "NOT_EXERCISED"),
        v.effectStateChecks,
        v.effectTransitionsObserved
    );
    log::info(
        "DYNAMIC_WORLD_RESTORE_TEST={} checks={} transitionsObserved={}",
        !v.dynamicWorldPass ? "FAIL" : (v.dynamicTransitionsObserved > 0 ? "PASS" : "NOT_EXERCISED"),
        v.dynamicWorldChecks,
        v.dynamicTransitionsObserved
    );
    log::info(
        "REAL_INPUT_QUEUE_TEST={} checks={}",
        v.inputQueuePass ? "PASS" : "FAIL",
        v.inputQueueChecks
    );
}

} // namespace

class $modify(AutoBotT7BaseGameLayerHook, GJBaseGameLayer) {
    void update(float dt) {
        if (g_universalInternalStep) {
            GJBaseGameLayer::update(dt);
            return;
        }

        auto* owner = g_universalRuntime.owner();
        const bool universalOwns = owner
            && static_cast<GJBaseGameLayer*>(owner) == static_cast<GJBaseGameLayer*>(this);
        if (universalOwns) {
            g_universalRuntime.setStepCallback([this, owner](float innerDt) {
                // Simulation steps run only inside a captured oracle state and
                // are restored before the single visible update below.
                owner->m_isPaused = false;
                g_universalInternalStep = true;
                ++g_liveSimulationSteps;
                GJBaseGameLayer::update(innerDt);
                g_universalInternalStep = false;
            });

            if (g_universalRuntime.searching()) {
                owner->m_isPaused = false;
                const auto searchSliceStart = PerfClock::now();
                {
                    UniversalRuntimeCallScope runtimeScope("liveSearchSlice");
                    g_universalRuntime.searchSlice(g_universalSearchSliceBudget);
                }
                g_universalSearchLastSliceMs = elapsedMs(searchSliceStart);

                constexpr double kTargetSliceMs = 4.0;
                constexpr std::size_t kMinSliceBudget = 1;
                constexpr std::size_t kMaxSliceBudget = 64;
                if (g_universalSearchLastSliceMs < kTargetSliceMs * 0.50
                    && g_universalSearchSliceBudget < kMaxSliceBudget) {
                    g_universalSearchSliceBudget = std::min(
                        kMaxSliceBudget,
                        g_universalSearchSliceBudget * 2U
                    );
                } else if (g_universalSearchLastSliceMs > kTargetSliceMs * 1.75
                    && g_universalSearchSliceBudget > kMinSliceBudget) {
                    g_universalSearchSliceBudget = std::max(
                        kMinSliceBudget,
                        g_universalSearchSliceBudget / 2U
                    );
                }

                const auto stats = g_universalRuntime.stats();
                const auto validation = g_universalRuntime.validation();
                if (!g_liveRoundTripLogged && validation.liveRoundTripChecks > 0) {
                    g_liveRoundTripLogged = true;
                    log::info(
                        "LIVE_SEARCH_ROUNDTRIP_RUNTIME={} checks={} rng={} effects={} dynamic={} "
                        "queued={} attempts={} pause={}",
                        validation.roundTripPass ? "PASS" : "FAIL",
                        validation.liveRoundTripChecks,
                        validation.rngPass ? "PASS" : "FAIL",
                        validation.effectStatePass ? "PASS" : "FAIL",
                        validation.dynamicWorldPass ? "PASS" : "FAIL",
                        validation.queuedStatePass ? "PASS" : "FAIL",
                        validation.attemptStatePass ? "PASS" : "FAIL",
                        validation.pauseStatePass ? "PASS" : "FAIL"
                    );
                }

                if (g_universalHUD) {
                    g_universalHUD->updatePreRun(
                        autobot::presolve::PreRunStage::Searching,
                        fmt::format(
                            "LIVE SEARCH t={:.1f}s eps={:.0f} exp={} steps={} frontier={} best={:.2f}% "
                            "reroots={} budget={} slice={:.2f}ms",
                            stats.elapsedSeconds,
                            stats.expansionsPerSecond,
                            stats.totalExpansions,
                            stats.totalEngineSteps,
                            stats.frontierSize,
                            stats.bestProgress,
                            g_universalRuntime.liveRerootCount(),
                            g_universalSearchSliceBudget,
                            g_universalSearchLastSliceMs
                        ),
                        g_universalRuntime.fullPolicyAvailable(),
                        stats.bestProgress
                    );
                }
            }

            if (g_universalRuntime.error()) {
                log::error(
                    "LIVE_GLOBAL_SEARCH=ERROR generation={} reason={} action=CONTINUE_WITH_LOCAL_MPC",
                    g_universalRuntime.generation(),
                    g_universalRuntime.reason()
                );
            }

            // Exactly one visible gameplay update per outer scheduler callback.
            // The search may have executed many reversible simulation steps,
            // but they were restored before this point.
            owner->m_isPaused = false;
            ++g_liveRealUpdates;
            if (g_liveRealUpdates <= 8 || (g_liveRealUpdates % 120U) == 0U) {
                const auto stats = g_universalRuntime.stats();
                log::info(
                    "LIVE_SOLVER_FRAME generation={} realUpdates={} simulationSteps={} levelTime={:.6f} "
                    "progress={:.3f}% searchElapsed={:.3f}s expansions={} engineSteps={} frontier={}",
                    g_universalRuntime.generation(),
                    g_liveRealUpdates,
                    g_liveSimulationSteps,
                    owner->m_gameState.m_levelTime,
                    owner->getCurrentPercent(),
                    stats.elapsedSeconds,
                    stats.totalExpansions,
                    stats.totalEngineSteps,
                    stats.frontierSize
                );
            }
            GJBaseGameLayer::update(dt);
            return;
        }

        GJBaseGameLayer::update(dt);
    }
};

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
        bool pendingPressResponse = false;
        bool runtimeAutoplayActiveLogged = false;
        bool firstRequiredActionLogged = false;
        bool preRunFreezeActive = false;
        bool freezePassLogged = false;
        bool freezeFailLogged = false;
        autobot::presolve::FreezeInvariantSnapshot freezeAnchor{};
        PerfClock::time_point freezeStarted = PerfClock::now();
        std::uint64_t pendingPressSample = 0;
        double pendingPressY = 0.0;
        double pendingPressVy = 0.0;

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
        autobot::ui::ShowTrajectoryOverlay showTrajectory{};

        autobot::control::InputController inputController{};
        autobot::control::AutonomousTestDriver autonomousDriver{};
        autobot::core::SolverPerformanceMetrics performance{};
    };

    autobot::presolve::FreezeInvariantSnapshot captureFreezeInvariant() {
        autobot::presolve::FreezeInvariantSnapshot snapshot{};
        if (m_player1) snapshot.playerX = static_cast<double>(m_player1->getRealPosition().x);
        snapshot.progress = static_cast<double>(getCurrentPercent());
        snapshot.levelTime = static_cast<double>(m_gameState.m_levelTime);
        snapshot.attempts = m_attempts;
        snapshot.dead = m_player1 ? m_player1->m_isDead : false;
        return snapshot;
    }

    void beginPreRunFreeze() {
        m_fields->preRunFreezeActive = true;
        m_fields->freezePassLogged = false;
        m_fields->freezeFailLogged = false;
        m_fields->freezeAnchor = captureFreezeInvariant();
        m_fields->freezeStarted = PerfClock::now();
        activateAuthoritativeFreeze(this);
        m_isPaused = true;
    }

    void verifyPreRunFreezeInvariant() {
        const auto current = captureFreezeInvariant();
        const bool holds = autobot::presolve::freezeInvariantHolds(m_fields->freezeAnchor, current);
        if (!holds && !m_fields->freezeFailLogged) {
            m_fields->freezeFailLogged = true;
            log::error(
                "PRE_RUN_FREEZE_TEST=FAIL playerXDelta={:.6f} progressDelta={:.6f} levelTimeDelta={:.6f} "
                "deaths={} attemptRestart={}",
                current.playerX - m_fields->freezeAnchor.playerX,
                current.progress - m_fields->freezeAnchor.progress,
                current.levelTime - m_fields->freezeAnchor.levelTime,
                current.dead ? 1 : 0,
                current.attempts - m_fields->freezeAnchor.attempts
            );
        }
        if (holds && !m_fields->freezePassLogged && elapsedMs(m_fields->freezeStarted) >= 1000.0) {
            m_fields->freezePassLogged = true;
            log::info(
                "PRE_RUN_FREEZE_TEST=PASS playerXDelta=0 progressDelta=0 levelTimeDelta=0 deaths=0 attemptRestart=0"
            );
        }
    }

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
        const bool collisionReady = m_fields->collisionWorld.build(m_fields->world, parseTimeMs);
        if (!collisionReady) {
            log::warn(
                "AutoBot T7: diagnostic CollisionWorld build failed; universal engine-oracle remains available"
            );
        } else if (!m_fields->autonomousDriver.configureWorld(m_fields->world, m_fields->collisionWorld)) {
            log::warn("AutoBot T7: diagnostic TriggerWorldModel could not be built");
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
        const auto generationBefore = g_universalRuntime.generation();
        const auto runtimeStageBefore = g_universalRuntime.stage();
        const bool runtimeOwnsLayer = g_universalRuntime.owner() == this;

        log::info(
            "PLAYLAYER_STARTGAME generation={} runtimeStage={} runtimeOwnsLayer={} "
            "runtimeCallDepth={} runtimeOperation={}",
            generationBefore,
            runtimeStageName(runtimeStageBefore),
            runtimeOwnsLayer ? "YES" : "NO",
            g_universalRuntimeCallDepth,
            g_universalRuntimeCallOperation
        );

        if (g_universalRuntimeCallDepth > 0 && runtimeOwnsLayer) {
            log::error(
                "REENTRANT_LIFECYCLE startGame called while universal runtime call is active "
                "generation={} runtimeStage={} operation={} depth={} action=RUN_GD_ORIGINAL_ONLY",
                generationBefore,
                runtimeStageName(runtimeStageBefore),
                g_universalRuntimeCallOperation,
                g_universalRuntimeCallDepth
            );

            // This startGame came from inside begin/search/restore/engine stepping.
            // GD's original lifecycle is allowed to run, but AutoBot must not
            // recursively reset/re-begin the same session while its core is on stack.
            PlayLayer::startGame();

            log::error(
                "REENTRANT_LIFECYCLE_RETURN generation={} runtimeStage={} operation={} "
                "runtimeCoreFrontier={} runtimeCoreExpansions={}",
                g_universalRuntime.generation(),
                runtimeStageName(g_universalRuntime.stage()),
                g_universalRuntimeCallOperation,
                g_universalRuntime.stats().frontierSize,
                g_universalRuntime.stats().totalExpansions
            );
            return;
        }

        m_fields->preRunFreezeActive = false;
        PlayLayer::startGame();
        releaseAuthoritativeFreeze(this);

        m_fields->runtimeAutoplayActiveLogged = false;
        m_fields->firstRequiredActionLogged = false;
        if (!m_fields->autonomousHUD.attached()) {
            m_fields->autonomousHUD.attach(this);
        }
        m_fields->autonomousHUD.updatePreRun(
            autobot::presolve::PreRunStage::ParsingLevel,
            "BUILDING FULL LEVEL MODEL",
            false,
            0.0
        );

        if (!buildWorldFromGameplayLifecycle()) {
            log::error("NOT READY: full level world could not be built");
            m_fields->autonomousHUD.updatePreRun(
                autobot::presolve::PreRunStage::NotReady,
                "FULL LEVEL WORLD COULD NOT BE BUILT",
                false,
                0.0
            );
            return;
        }

        log::info(
            "REAL_LEVEL_OBJECTS={} UNKNOWN={} UNSUPPORTED_GAMEPLAY={} PORTALS={}",
            m_fields->world.objects.size(),
            m_fields->world.unknown,
            m_fields->world.unsupportedGameplay,
            m_fields->world.portals
        );
        double lastColliderX = 0.0;
        bool hasCollider = false;
        for (auto const& primitive : m_fields->collisionWorld.primitives()) {
            if (!primitive.enabled) continue;
            lastColliderX = hasCollider
                ? std::max(lastColliderX, static_cast<double>(primitive.broadphaseBounds.x + primitive.broadphaseBounds.width))
                : static_cast<double>(primitive.broadphaseBounds.x + primitive.broadphaseBounds.width);
            hasCollider = true;
        }
        log::info(
            "LEVEL_BOUNDARY ACTUAL_LEVEL_COMPLETION_BOUNDARY={:.3f} GD_LEVEL_LENGTH={:.3f} "
            "WORLD_LAST_COLLIDER={:.3f} boundaryValid={} sourcesConsistent={}",
            m_fields->world.completionBoundaryX,
            m_fields->world.gdLevelLength,
            hasCollider ? lastColliderX : 0.0,
            m_fields->world.completionBoundaryValid,
            m_fields->world.completionSourcesConsistent
        );
        for (auto const& audit : m_fields->world.unsupportedAudit) {
            log::warn(
                "UNSUPPORTED_OBJECT_AUDIT objectID={} rawType={} rawTypeName={} x={:.3f} y={:.3f} classification={} "
                "whyUnsupported={} gameplayRelevant={}",
                audit.objectID,
                audit.rawGameObjectType,
                gameObjectTypeName(audit.rawGameObjectType),
                audit.x,
                audit.y,
                autobot::world::toString(audit.classification),
                audit.reason,
                audit.gameplayRelevant ? "YES" : "NO"
            );
        }

        log::info(
            "UNIVERSAL_PARSE_GATE objects={} runtimeRequiredUnknown={} manualUnsupported={} unknownClassified={}",
            m_fields->world.objects.size(),
            m_fields->world.runtimeRequiredUnknown,
            m_fields->world.unsupportedGameplay,
            m_fields->world.unknown
        );
        if (m_fields->world.runtimeRequiredUnknown != 0) {
            const auto reason = fmt::format(
                "{} RAW RUNTIME TYPES UNKNOWN TO GD 2.2081 BINDINGS",
                m_fields->world.runtimeRequiredUnknown
            );
            log::error("UNIVERSAL_RUNTIME_ORACLE=BLOCKED reason={}", reason);
            m_fields->autonomousHUD.updatePreRun(
                autobot::presolve::PreRunStage::NotReady, reason, false, 0.0
            );
            return;
        }

        g_universalRuntime.reset("PlayLayer::startGame normal lifecycle");
        bool runtimeBeginOk = false;
        {
            UniversalRuntimeCallScope runtimeScope("begin");
            runtimeBeginOk = g_universalRuntime.begin(this);
        }
        if (!runtimeBeginOk) {
            m_fields->preRunFreezeActive = false;
            m_isPaused = false;
            log::error("UNIVERSAL_RUNTIME_ORACLE=FAIL reason={} action=CONTINUE_WITH_LOCAL_MPC", g_universalRuntime.reason());
            m_fields->autonomousHUD.updatePreRun(
                autobot::presolve::PreRunStage::NotReady,
                g_universalRuntime.reason(),
                false,
                0.0
            );
            return;
        }
        const auto bootstrapStats = g_universalRuntime.stats();
        log::info(
            "RUNTIME_BOOTSTRAP_READY generation={} runtimeStage={} coreStage={} started={} "
            "frontier={} elapsed={:.6f}s expansions={} engineSteps={} resetSerial={}",
            g_universalRuntime.generation(),
            runtimeStageName(g_universalRuntime.stage()),
            static_cast<int>(bootstrapStats.stage),
            bootstrapStats.started ? "YES" : "NO",
            bootstrapStats.frontierSize,
            bootstrapStats.elapsedSeconds,
            bootstrapStats.totalExpansions,
            bootstrapStats.totalEngineSteps,
            bootstrapStats.resetSerial
        );

        g_universalHUD = &m_fields->autonomousHUD;
        m_fields->preRunFreezeActive = false;
        releaseAuthoritativeFreeze(this);
        g_universalSearchSliceBudget = 8;
        g_universalSearchLastSliceMs = 0.0;
        g_liveSimulationSteps = 0;
        g_liveRealUpdates = 0;
        g_liveRoundTripLogged = false;
        m_isPaused = false;
        log::info(
            "LIVE_SOLVER=ACTIVE stepDt={:.9f} requiredUnknown=0 manualUnsupported={} "
            "globalSearch=ROLLING localControl=MPC visibleFreeze=NO",
            g_universalRuntime.stepDt(),
            m_fields->world.unsupportedGameplay
        );
        m_fields->autonomousHUD.updatePreRun(
            autobot::presolve::PreRunStage::Searching,
            "LIVE GLOBAL SEARCH + LOCAL MPC",
            false,
            0.0
        );
    }

    void postUpdate(float dt) {
        if (g_universalInternalStep) {
            PlayLayer::postUpdate(dt);
            return;
        }
        // Live runtime never blocks postUpdate waiting for a full policy.
        // Simulation callbacks return above; this path belongs to the one real
        // gameplay update and feeds the realtime MPC for the next control step.
        m_fields->preRunFreezeActive = false;
        m_isPaused = false;
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
        if (!m_fields->showTrajectory.attached() && !m_fields->showTrajectory.attach(playLayer)) {
            log::warn("AutoBot T7: Show Trajectory overlay could not be attached yet");
        }

        const bool collisionDebugEnabled = Mod::get()->getSettingValue<bool>("collision-debug-overlay");
        const bool traceDebugEnabled = Mod::get()->getSettingValue<bool>("collision-trace-debug");
        const bool objectLabelsEnabled = Mod::get()->getSettingValue<bool>("collision-object-labels");
        const bool fullWorldDebug = Mod::get()->getSettingValue<bool>("collision-debug-full-world");
        const bool autobotEnabled = Mod::get()->getSettingValue<bool>("autobot-enabled");
        const bool showTrajectoryEnabled = Mod::get()->getSettingValue<bool>("show-trajectory");
        const bool solverDebugEnabled = Mod::get()->getSettingValue<bool>("solver-debug");

        m_fields->collisionDebug.setEnabled(collisionDebugEnabled);
        m_fields->collisionDebug.setObjectLabelsEnabled(
            collisionDebugEnabled && traceDebugEnabled && objectLabelsEnabled
        );
        m_fields->collisionDebug.setFullWorldEnabled(collisionDebugEnabled && fullWorldDebug);
        m_fields->showTrajectory.setEnabled(showTrajectoryEnabled);

        const auto stateReadStart = PerfClock::now();
        const auto snapshot = autobot::core::GameStateReader::capture(playLayer, m_fields->tick);
        m_fields->performance.stateRead.add(elapsedMs(stateReadStart));
        m_fields->hud.update(snapshot, m_fields->world);

        if (m_fields->pendingPressResponse && snapshot.valid) {
            const bool playerResponded = std::abs(snapshot.player.velocityY - m_fields->pendingPressVy) > 0.05
                || std::abs(snapshot.player.y - m_fields->pendingPressY) > 0.05;
            log::info(
                "INPUT EXECUTION RESPONSE selectedSample={} observedSample={} holding={} "
                "playerResponse={} yBefore={:.3f} yNow={:.3f} vyBefore={:.3f} vyNow={:.3f}",
                m_fields->pendingPressSample,
                snapshot.solverSampleID,
                m_fields->inputController.botHolding(),
                playerResponded,
                m_fields->pendingPressY,
                snapshot.player.y,
                m_fields->pendingPressVy,
                snapshot.player.velocityY
            );
            m_fields->pendingPressResponse = false;
        }

        autobot::world::DynamicSyncStats syncStats{};
        if (m_fields->collisionWorld.ready()) {
            const auto worldSyncStart = PerfClock::now();
            syncStats = m_fields->dynamicSync.sync(
                playLayer,
                m_fields->world,
                m_fields->collisionWorld
            );
            m_fields->performance.worldSync.add(elapsedMs(worldSyncStart));
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
        const auto queryStart = PerfClock::now();
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
        m_fields->performance.query.add(elapsedMs(queryStart));

        double debugOverlayMs = 0.0;
        const auto collisionDebugStart = PerfClock::now();
        m_fields->collisionDebug.update(
            snapshot,
            m_fields->collisionWorld,
            query,
            traceQuery ? &*traceQuery : nullptr
        );
        debugOverlayMs += elapsedMs(collisionDebugStart);

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

        auto decision = m_fields->autonomousDriver.decide(
            snapshot,
            m_fields->collisionWorld,
            query,
            autobotEnabled,
            m_fields->inputController.botHolding(),
            m_fields->inputController.botHoldingP2()
        );

        if (g_universalRuntime.owner() == this) {
            const auto liveSearch = g_universalRuntime.stats();
            decision.liveSearchElapsed = liveSearch.elapsedSeconds;
            decision.liveSearchExpansionsPerSecond = liveSearch.expansionsPerSecond;
            decision.liveSearchExpansions = liveSearch.totalExpansions;
            decision.liveSearchEngineSteps = liveSearch.totalEngineSteps;
            decision.liveSearchFrontier = liveSearch.frontierSize;
            decision.liveSearchReroots = g_universalRuntime.liveRerootCount();
            decision.liveSearchBestProgress = liveSearch.bestProgress;
            decision.liveGlobalPolicyAvailable = g_universalRuntime.fullPolicyAvailable();
        }

        if (decision.countdown.active || decision.countdown.fired) {
            log::info(
                "ACTION_COUNTDOWN sample={} label={} dueIn={} targetSample={} fired={} action={}",
                snapshot.solverSampleID,
                decision.countdown.label,
                decision.countdown.dueIn,
                decision.countdown.targetSampleID,
                decision.countdown.fired,
                autobot::control::toString(decision.action)
            );
        }

        m_fields->performance.candidateGeneration.add(decision.plan.candidateGenerationMs);
        m_fields->performance.physicsSimulation.add(decision.plan.physicsSimulationMs);
        m_fields->performance.trajectoryScoring.add(decision.plan.trajectoryScoringMs);
        m_fields->performance.plannerTotal.add(decision.plan.plannerDurationMs);

        if (solverDebugEnabled
            && decision.plan.modelTruth.nearestHazardPrimitive != autobot::world::kInvalidPrimitiveIndex
            && (m_fields->tick % 10u == 0u)) {
            auto const& truth = decision.plan.modelTruth;
            log::info(
                "MODEL_TRUTH_TRACE sample={} realX={:.3f} realY={:.3f} rawVx={:.5f} obsWorldVx={:.3f} "
                "realDt={:.6f}s simDt={:.4f} ratio={:.3f} hazardPrim={} hazardDist={:.3f} "
                "timeToHazard={:.4f}s samplesToHazard={:.2f} horizonTicks={} requiredDistance={:.2f}",
                snapshot.solverSampleID,
                snapshot.player.x,
                snapshot.player.y,
                truth.rawVelocityX,
                truth.observedWorldVelocityX,
                truth.realDtSeconds,
                truth.simDtNormalized,
                truth.physicsTicksPerSecond,
                truth.nearestHazardPrimitive,
                truth.nearestHazardDistance,
                truth.timeToHazardSeconds,
                truth.samplesToHazard,
                truth.horizonTicks,
                truth.requiredForwardDistance
            );
            log::info(
                "HAZARD_PIPELINE prim={} source={} objectID={} uniqueID={} classHazard={} geometry={} "
                "enabled={} indexable={} noTouch={} hashIndexed={} hashCells={} insideQuery={} "
                "localContains={} bruteForceContains={}",
                truth.nearestHazardPrimitive,
                truth.nearestHazardSourceIndex,
                truth.nearestHazardObjectID,
                truth.nearestHazardUniqueID,
                truth.hazardClassifiedHazard,
                autobot::world::toString(truth.nearestHazardGeometry),
                truth.hazardEnabled,
                truth.hazardIndexable,
                truth.hazardNoTouch,
                truth.hazardHashIndexed,
                truth.hazardHashCellCount,
                truth.hazardInsideLocalQueryRect,
                truth.hazardLocalWorldContains,
                truth.hazardBruteForceContains
            );
            for (auto const& trajectory : decision.plan.trajectories) {
                const auto collisionTick = trajectory.collisionTick == std::numeric_limits<std::size_t>::max()
                    ? -1LL
                    : static_cast<long long>(trajectory.collisionTick);
                log::info(
                    "CANDIDATE_TRACE sample={} label={} class={} fatal={} hazardCollision={} collisionTick={} "
                    "collisionPrimitive={} minClearance={:.3f} progress={:.3f} finalX={:.3f} finalY={:.3f} "
                    "horizonConclusive={} receivesNearestHazard={} confidence={:.3f} score={:.3f}",
                    snapshot.solverSampleID,
                    trajectory.candidate.label,
                    autobot::solver::toString(trajectory.classification),
                    trajectory.fatalCollision,
                    trajectory.hazardCollision,
                    collisionTick,
                    trajectory.collisionPrimitive,
                    trajectory.minimumClearance,
                    trajectory.progress,
                    trajectory.predictedFinalX,
                    trajectory.predictedFinalY,
                    trajectory.horizonConclusive,
                    trajectory.nearestHazardSeen,
                    trajectory.confidence,
                    trajectory.score
                );

                if (trajectory.candidate.label == "NO PRESS" || trajectory.candidate.label == "PRESS NOW") {
                    for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
                        auto const& point = trajectory.points[i];
                        if (!point.nearestHazardIntersects
                            && point.nearestHazardDistance > 60.0
                            && !point.collision) {
                            continue;
                        }
                        log::info(
                            "TRAJECTORY_COLLISION_TRACE sample={} candidate={} tick={} x={:.3f} y={:.3f} "
                            "playerBounds=({:.2f},{:.2f},{:.2f},{:.2f}) hazardBounds=({:.2f},{:.2f},{:.2f},{:.2f}) "
                            "distance={:.3f} intersects={} collision={}",
                            snapshot.solverSampleID,
                            trajectory.candidate.label,
                            i,
                            point.x,
                            point.y,
                            point.playerBounds.x,
                            point.playerBounds.y,
                            point.playerBounds.width,
                            point.playerBounds.height,
                            point.nearestHazardBounds.x,
                            point.nearestHazardBounds.y,
                            point.nearestHazardBounds.width,
                            point.nearestHazardBounds.height,
                            point.nearestHazardDistance,
                            point.nearestHazardIntersects,
                            point.collision
                        );
                    }
                }
            }
        }

        if (decision.deathSnapshot.valid) {
            auto const& death = decision.deathSnapshot;
            log::error(
                "DEATH CAUSAL SNAPSHOT deathSample={} lastSelected={} input={} class={} predictedDeath={} "
                "realDeath=YES FALSE_SAFE={} FALSE_SAFE_TOTAL={}",
                death.deathSampleID,
                death.lastSelectedLabel,
                autobot::control::toString(death.lastSelectedInput),
                autobot::solver::toString(death.lastSelectedClass),
                death.predictedDeath,
                death.falseSafe,
                decision.falseSafeTotal
            );
            if (death.falseSafe) {
                log::error("RUNTIME MODEL INVARIANT FAILURE: FALSE SAFE");
            }
            for (auto const& sample : death.history) {
                auto const& truth = sample.diagnostics;
                log::error(
                    "DEATH_HISTORY sample={} t={:.5f} x={:.3f} y={:.3f} rawVx={:.4f} rawVy={:.4f} "
                    "grounded={} gravity={:.5f} mode={} mini={} holding={} query=({:.1f},{:.1f},{:.1f},{:.1f}) "
                    "hazardPrim={} hazardDist={:.2f} selected={} input={} predictedDeath={}",
                    sample.solverSampleID,
                    sample.levelTime,
                    sample.player.x,
                    sample.player.y,
                    sample.player.velocityX,
                    sample.player.velocityY,
                    sample.player.grounded,
                    sample.player.gravity,
                    autobot::core::toString(sample.player.mode),
                    sample.player.mini,
                    sample.player.holding,
                    truth.localQueryRect.x,
                    truth.localQueryRect.y,
                    truth.localQueryRect.width,
                    truth.localQueryRect.height,
                    truth.nearestHazardPrimitive,
                    truth.nearestHazardDistance,
                    sample.selectedLabel,
                    autobot::control::toString(sample.selectedInput),
                    sample.predictedDeath
                );
                for (auto const& candidate : sample.candidates) {
                    const auto collisionTick = candidate.collisionTick == std::numeric_limits<std::size_t>::max()
                        ? -1LL
                        : static_cast<long long>(candidate.collisionTick);
                    log::error(
                        "DEATH_HISTORY_CANDIDATE sample={} label={} class={} fatal={} hazard={} collisionPrim={} "
                        "collisionTick={} clearance={:.3f} progress={:.3f} confidence={:.3f} score={:.3f} "
                        "finalX={:.3f} finalY={:.3f} conclusive={}",
                        sample.solverSampleID,
                        candidate.label,
                        autobot::solver::toString(candidate.classification),
                        candidate.fatalCollision,
                        candidate.hazardCollision,
                        candidate.collisionPrimitive,
                        collisionTick,
                        candidate.minimumClearance,
                        candidate.progress,
                        candidate.confidence,
                        candidate.score,
                        candidate.finalX,
                        candidate.finalY,
                        candidate.horizonConclusive
                    );
                }
            }
        }

        constexpr double kPlannerBudgetMs = 8.0;
        if (decision.plan.plannerDurationMs > kPlannerBudgetMs) {
            log::warn(
                "PLANNER DEADLINE MISSED tick={} durationMs={:.3f} budgetMs={:.3f} mode={}",
                m_fields->tick,
                decision.plan.plannerDurationMs,
                kPlannerBudgetMs,
                snapshot.valid ? autobot::core::toString(snapshot.player.mode) : "UNKNOWN"
            );
        }

        auto const& modelError = decision.plan.lastModelError;
        const bool modelDiverged = modelError.magnitude() > 25.0
            || !modelError.modeMatched
            || !modelError.landingMatched
            || !modelError.collisionMatched;
        if (modelDiverged && m_fields->autonomousDriver.validation().stats().samples > 0) {
            log::warn(
                "MODEL DIVERGENCE tick={} dx={:.3f} dy={:.3f} dvx={:.3f} dvy={:.3f} "
                "modeMatched={} groundedMatched={} collisionMatched={}",
                m_fields->tick,
                modelError.dx,
                modelError.dy,
                modelError.dvx,
                modelError.dvy,
                modelError.modeMatched,
                modelError.landingMatched,
                modelError.collisionMatched
            );
        }

        const bool readyForInput = autobotEnabled
            && decision.active
            && decision.plan.gameStateReady
            && decision.plan.worldReady
            && decision.plan.physicsReady
            && decision.plan.plannerReady;

        const auto inputStart = PerfClock::now();
        m_fields->inputController.setBotControl(readyForInput, playLayer, inputTimestamp);
        bool inputApplied = false;
        if (readyForInput) {
            if (decision.action == autobot::control::InputAction::Press) {
                log::info("PLANNER SELECTED PRESS sample={}", snapshot.solverSampleID);
            }
            inputApplied = snapshot.dualMode
                ? m_fields->inputController.applyJoint(
                    decision.action, decision.p2Action, playLayer, inputTimestamp
                )
                : m_fields->inputController.apply(
                    decision.action, playLayer, inputTimestamp
                );
            auto const& transition = m_fields->inputController.lastTransition();
            if (m_fields->inputController.lastQueueInvoked()) {
                log::info(
                    "INPUT EXECUTION TRACE sample={} requested={} effective={} queueButtonInvoked=YES push={} "
                    "success={} holdingAfter={}",
                    snapshot.solverSampleID,
                    autobot::control::toString(decision.action),
                    autobot::control::toString(transition.effectiveAction),
                    m_fields->inputController.lastQueuePush(),
                    m_fields->inputController.lastQueueSucceeded(),
                    m_fields->inputController.botHolding()
                );
            }
            if (snapshot.dualMode && m_fields->inputController.lastQueueInvokedP2()) {
                auto const& p2Transition = m_fields->inputController.lastTransitionP2();
                log::info(
                    "INPUT EXECUTION TRACE P2 sample={} requested={} effective={} queueButtonInvoked=YES push={} "
                    "success={} holdingAfter={}",
                    snapshot.solverSampleID,
                    autobot::control::toString(decision.p2Action),
                    autobot::control::toString(p2Transition.effectiveAction),
                    m_fields->inputController.lastQueuePushP2(),
                    m_fields->inputController.lastQueueSucceededP2(),
                    m_fields->inputController.botHoldingP2()
                );
            }
            if (transition.emitPress && inputApplied) {
                m_fields->pendingPressResponse = true;
                m_fields->pendingPressSample = snapshot.solverSampleID;
                m_fields->pendingPressY = snapshot.player.y;
                m_fields->pendingPressVy = snapshot.player.velocityY;
            }
        }
        m_fields->performance.input.add(elapsedMs(inputStart));

        if (readyForInput) {
            decision.ownership = m_fields->inputController.ownership();
            decision.active = decision.ownership == autobot::control::InputOwnership::Bot;
            if (decision.active && !m_fields->runtimeAutoplayActiveLogged) {
                m_fields->runtimeAutoplayActiveLogged = true;
                log::info("AUTOPLAY_CONTROL=ACTIVE sample={}", snapshot.solverSampleID);
            }
            auto const& transition = m_fields->inputController.lastTransition();
            if (inputApplied
                && (transition.emitPress || transition.emitRelease)
                && !m_fields->firstRequiredActionLogged) {
                m_fields->firstRequiredActionLogged = true;
                log::info(
                    "FIRST_REQUIRED_ACTION=EXECUTED sample={} action={} queueSuccess={} holdingAfter={}",
                    snapshot.solverSampleID,
                    autobot::control::toString(transition.effectiveAction),
                    m_fields->inputController.lastQueueSucceeded(),
                    m_fields->inputController.botHolding()
                );
            }
        } else {
            decision.ownership = autobotEnabled
                ? autobot::control::InputOwnership::None
                : autobot::control::InputOwnership::User;
            decision.active = false;
            if (autobotEnabled && decision.reason.empty()) {
                decision.reason = "AUTOBOT WAITING FOR READY";
            }
        }

        const auto finalOverlayStart = PerfClock::now();
        m_fields->showTrajectory.update(decision.plan);
        m_fields->autonomousHUD.update(snapshot, decision, solverDebugEnabled);
        debugOverlayMs += elapsedMs(finalOverlayStart);
        m_fields->performance.debugOverlay.add(debugOverlayMs);

        if (m_fields->tick % 120u == 0u) {
            auto emitPerf = [&](char const* name, autobot::core::RollingTiming const& timing) {
                const auto summary = timing.summary();
                if (summary.samples == 0) return;
                log::info(
                    "SOLVER_PERF {} n={} avg={:.4f}ms p95={:.4f}ms p99={:.4f}ms",
                    name,
                    summary.samples,
                    summary.averageMs,
                    summary.p95Ms,
                    summary.p99Ms
                );
            };
            emitPerf("state_read", m_fields->performance.stateRead);
            emitPerf("world_sync", m_fields->performance.worldSync);
            emitPerf("query", m_fields->performance.query);
            emitPerf("candidate_generation", m_fields->performance.candidateGeneration);
            emitPerf("physics_simulation", m_fields->performance.physicsSimulation);
            emitPerf("trajectory_scoring", m_fields->performance.trajectoryScoring);
            emitPerf("planner_total", m_fields->performance.plannerTotal);
            emitPerf("input", m_fields->performance.input);
            emitPerf("debug_overlay", m_fields->performance.debugOverlay);
        }
    }
};
