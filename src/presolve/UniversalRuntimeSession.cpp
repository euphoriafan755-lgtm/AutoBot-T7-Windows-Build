#include "autobot/presolve/UniversalRuntimeSession.hpp"

#include <Geode/Geode.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

using namespace geode::prelude;

namespace autobot::presolve {
namespace {

char const* runtimeStageName(UniversalRuntimeStage stage) {
    switch (stage) {
        case UniversalRuntimeStage::Idle: return "Idle";
        case UniversalRuntimeStage::Searching: return "Searching";
        case UniversalRuntimeStage::Ready: return "Ready";
        case UniversalRuntimeStage::Playing: return "Playing";
        case UniversalRuntimeStage::Complete: return "Complete";
        case UniversalRuntimeStage::Error: return "Error";
    }
    return "Unknown";
}

char const* searchStageName(UniversalSearchStage stage) {
    switch (stage) {
        case UniversalSearchStage::Idle: return "Idle";
        case UniversalSearchStage::Searching: return "Searching";
        case UniversalSearchStage::Replaying: return "Replaying";
        case UniversalSearchStage::Ready: return "Ready";
        case UniversalSearchStage::Exhausted: return "Exhausted";
        case UniversalSearchStage::Error: return "Error";
    }
    return "Unknown";
}

} // namespace

void UniversalRuntimeSession::recordLifecycle(std::string message) {
    constexpr std::size_t kMaxLifecycleEvents = 24;
    if (m_lifecycleTrace.size() >= kMaxLifecycleEvents) m_lifecycleTrace.pop_front();
    m_lifecycleTrace.push_back(std::move(message));
}

void UniversalRuntimeSession::dumpLifecycle(std::string_view context) const {
    log::error(
        "UNIVERSAL_LIFECYCLE_DUMP generation={} context={} entries={}",
        m_generation,
        context,
        m_lifecycleTrace.size()
    );
    for (std::size_t i = 0; i < m_lifecycleTrace.size(); ++i) {
        log::error("UNIVERSAL_LIFECYCLE_DUMP[{}] {}", i, m_lifecycleTrace[i]);
    }
}

bool UniversalRuntimeSession::validateSearchCoreInvariant(std::string_view context) {
    if (m_stage != UniversalRuntimeStage::Searching) return true;

    const auto core = m_search.stats();
    const bool stageOk = core.stage == UniversalSearchStage::Searching
        || core.stage == UniversalSearchStage::Replaying
        || core.stage == UniversalSearchStage::Ready;
    const bool frontierOk = core.stage != UniversalSearchStage::Searching || core.frontierSize > 0;
    const bool startedOk = core.started;

    if (stageOk && frontierOk && startedOk) return true;

    const auto reason = fmt::format(
        "A) SEARCH CORE LOST AFTER BEGIN generation={} context={} runtimeStage={} coreStage={} "
        "started={} frontier={} elapsed={:.6f}s expansions={} engineSteps={} resetSerial={}",
        m_generation,
        context,
        runtimeStageName(m_stage),
        searchStageName(core.stage),
        core.started ? "YES" : "NO",
        core.frontierSize,
        core.elapsedSeconds,
        core.totalExpansions,
        core.totalEngineSteps,
        core.resetSerial
    );
    log::error("{}", reason);
    recordLifecycle(reason);
    dumpLifecycle(context);
    enterError(reason);
    return false;
}

bool UniversalRuntimeSession::begin(PlayLayer* layer) {
    if (!layer) {
        enterError("A) SEARCH DRIVER NO CORRE: PLAYLAYER NULL");
        return false;
    }

    const auto previousRuntimeStage = m_stage;
    ++m_generation;
    m_lifecycleTrace.clear();
    m_searchSliceCount = 0;
    m_liveRerootCount = 0;
    m_liveAccumulatedExpansions = 0;
    m_liveAccumulatedEngineSteps = 0;
    m_liveBestProgress = 0.0;
    m_fullPolicyAvailable = false;
    m_liveStartedAt = std::chrono::steady_clock::now();
    m_owner = layer;

    m_search.setLifecycleGeneration(m_generation);
    m_search.setLifecycleCallback([this](
        std::uint64_t generation,
        UniversalSearchStage previousStage,
        UniversalSearchStage newStage,
        std::string_view reason,
        std::size_t resetSerial
    ) {
        const auto event = fmt::format(
            "SEARCH_CORE_RESET generation={} reason={} previousStage={} newStage={} resetSerial={}",
            generation,
            reason,
            searchStageName(previousStage),
            searchStageName(newStage),
            resetSerial
        );
        log::warn("{}", event);
        recordLifecycle(event);
    });

    const auto runtimeBegin = fmt::format(
        "RUNTIME_BEGIN generation={} previousStage={} newStage=Initializing",
        m_generation,
        runtimeStageName(previousRuntimeStage)
    );
    log::info("{}", runtimeBegin);
    recordLifecycle(runtimeBegin);

    // Start coarse enough for real use, then tighten only when search evidence
    // says the current resolution cannot close the route.
    m_oracle = std::make_unique<GeometryDashRuntimeOracle>(layer, 1.0 / 120.0);

    auto root = m_oracle->capture();
    if (!root) {
        enterError("C) RESTORE INCORRECTO: VISIBLE ROOT SNAPSHOT FAILED: " + m_oracle->lastError());
        return false;
    }
    m_visibleRoot = *root;

    if (!m_search.begin(*m_oracle)) {
        enterError("A) SEARCH DRIVER NO CORRE: UNIVERSAL SEARCH BEGIN FAILED: " + m_oracle->lastError());
        return false;
    }

    const auto bootstrap = m_search.stats();
    const auto searchBeginEvent = fmt::format(
        "SEARCH_BEGIN generation={} frontier={} coreStage={} started={} elapsed={:.6f}s resetSerial={}",
        m_generation,
        bootstrap.frontierSize,
        searchStageName(bootstrap.stage),
        bootstrap.started ? "YES" : "NO",
        bootstrap.elapsedSeconds,
        bootstrap.resetSerial
    );
    log::info("{}", searchBeginEvent);
    recordLifecycle(searchBeginEvent);

    m_stage = UniversalRuntimeStage::Searching;
    if (!validateSearchCoreInvariant("immediately-after-begin")) return false;
    m_reason = "ENGINE RUNTIME ORACLE SEARCH";
    layer->m_isPaused = false;
    return true;
}

void UniversalRuntimeSession::reset(std::string_view reason) {
    const auto previousRuntimeStage = m_stage;
    const auto coreBefore = m_search.stats();

    log::warn(
        "RUNTIME_RESET generation={} reason={} previousStage={} newStage=Idle "
        "coreStage={} frontier={} expansions={} engineSteps={} resetSerial={}",
        m_generation,
        reason,
        runtimeStageName(previousRuntimeStage),
        searchStageName(coreBefore.stage),
        coreBefore.frontierSize,
        coreBefore.totalExpansions,
        coreBefore.totalEngineSteps,
        coreBefore.resetSerial
    );
    recordLifecycle(fmt::format(
        "RUNTIME_RESET generation={} reason={} previousStage={} newStage=Idle coreStage={} resetSerial={}",
        m_generation,
        reason,
        runtimeStageName(previousRuntimeStage),
        searchStageName(coreBefore.stage),
        coreBefore.resetSerial
    ));

    m_search.reset(m_oracle.get(), reason);
    if (m_oracle) {
        if (m_lastSafePlayback != kInvalidUniversalToken) m_oracle->discard(m_lastSafePlayback);
        if (m_visibleRoot != kInvalidUniversalToken) m_oracle->discard(m_visibleRoot);
    }

    m_owner = nullptr;
    m_oracle.reset();
    m_visibleRoot = kInvalidUniversalToken;
    m_lastSafePlayback = kInvalidUniversalToken;
    m_policy.clear();
    m_expectedStates.clear();
    m_playbackCursor = 0;
    m_refinement = 0;
    m_recoveryCount = 0;
    m_stallRecoveryCount = 0;
    m_searchSliceCount = 0;
    m_liveRerootCount = 0;
    m_liveAccumulatedExpansions = 0;
    m_liveAccumulatedEngineSteps = 0;
    m_liveBestProgress = 0.0;
    m_fullPolicyAvailable = false;
    m_liveStartedAt = {};
    m_accumulator = 0.0;
    m_stage = UniversalRuntimeStage::Idle;
    m_reason = "IDLE";
}

void UniversalRuntimeSession::setStepCallback(StepCallback callback) {
    if (m_oracle) m_oracle->setStepCallback(std::move(callback));
}

void UniversalRuntimeSession::enterError(std::string reason) {
    const auto previousStage = m_stage;
    m_stage = UniversalRuntimeStage::Error;
    m_reason = std::move(reason);
    recordLifecycle(fmt::format(
        "RUNTIME_ERROR generation={} previousStage={} newStage=Error reason={}",
        m_generation,
        runtimeStageName(previousStage),
        m_reason
    ));
    if (m_owner) m_owner->m_isPaused = false;
}

bool UniversalRuntimeSession::restartAtFinerResolution() {
    if (!m_oracle || m_visibleRoot == kInvalidUniversalToken) return false;

    constexpr double kMinStep = 1.0 / 960.0;
    constexpr std::size_t kMaxRefinements = 4;
    if (m_refinement >= kMaxRefinements || m_oracle->stepDt() <= kMinStep + 1e-12) {
        return false;
    }

    if (!m_oracle->restore(m_visibleRoot)) return false;
    accumulateCurrentSearchEpoch();
    m_search.reset(m_oracle.get(), "restartAtFinerResolution");

    ++m_refinement;
    const auto nextDt = std::max(kMinStep, m_oracle->stepDt() * 0.5);
    m_oracle->setStepDt(nextDt);

    if (!m_search.begin(*m_oracle)) return false;

    m_reason = "COARSE-TO-FINE SEARCH REFINEMENT " + std::to_string(m_refinement)
        + " DT=" + std::to_string(m_oracle->stepDt());
    return true;
}

bool UniversalRuntimeSession::recoverFromStall(std::string reason) {
    if (!m_oracle || m_stage != UniversalRuntimeStage::Searching) return false;
    ++m_stallRecoveryCount;

    // First change only search ordering/granularity. UniversalSearchCore keeps
    // every live state, so this does not sacrifice a valid solution.
    if (m_search.refineStrategy()) {
        m_reason = "E) SEARCH ES DEMASIADO LENTO: STALL -> STRATEGY TIER "
            + std::to_string(m_search.stats().strategyTier)
            + " (" + reason + ")";
        return true;
    }

    // If all macro tiers have been tried, re-run from the immutable visible
    // root with a finer engine step.
    if (restartAtFinerResolution()) {
        m_reason = "E) SEARCH ES DEMASIADO LENTO: STALL -> FINER ENGINE RESOLUTION ("
            + reason + ")";
        return true;
    }

    return false;
}

bool UniversalRuntimeSession::beginRecoveryFromToken(UniversalToken token, std::string reason) {
    if (!m_oracle || token == kInvalidUniversalToken) return false;
    if (!m_oracle->restore(token)) return false;

    m_search.reset(m_oracle.get(), "beginRecoveryFromToken");

    if (m_visibleRoot != kInvalidUniversalToken && m_visibleRoot != token) {
        m_oracle->discard(m_visibleRoot);
    }
    if (m_lastSafePlayback != kInvalidUniversalToken && m_lastSafePlayback != token) {
        m_oracle->discard(m_lastSafePlayback);
    }

    m_lastSafePlayback = kInvalidUniversalToken;
    m_visibleRoot = token;
    m_policy.clear();
    m_expectedStates.clear();
    m_playbackCursor = 0;
    m_accumulator = 0.0;
    ++m_recoveryCount;

    if (!m_search.begin(*m_oracle)) return false;

    m_stage = UniversalRuntimeStage::Searching;
    m_reason = "H) PLAYBACK DIVERGE: RECOVERY REPLAN #"
        + std::to_string(m_recoveryCount) + ": " + reason;
    if (m_owner) m_owner->m_isPaused = false;
    return true;
}

void UniversalRuntimeSession::fail(std::string reason) {
    enterError(std::move(reason));
}

bool UniversalRuntimeSession::canonicalMatchesExpected(std::size_t index, UniversalToken token) const {
    if (!m_oracle || index >= m_expectedStates.size()) return false;
    auto current = m_oracle->canonicalState(token);
    if (!current) return false;
    return current->words == m_expectedStates[index].words;
}


void UniversalRuntimeSession::accumulateCurrentSearchEpoch() {
    const auto current = m_search.stats();
    m_liveAccumulatedExpansions += current.totalExpansions;
    m_liveAccumulatedEngineSteps += current.totalEngineSteps;
    m_liveBestProgress = std::max(m_liveBestProgress, current.bestProgress);
}

bool UniversalRuntimeSession::syncLiveRoot() {
    if (!m_oracle || !m_owner) return false;

    auto liveToken = m_oracle->capture();
    if (!liveToken || *liveToken == kInvalidUniversalToken) {
        enterError("C) RESTORE INCORRECTO: LIVE ROOT CAPTURE FAILED: " + m_oracle->lastError());
        return false;
    }

    if (m_visibleRoot == kInvalidUniversalToken) {
        m_visibleRoot = *liveToken;
        return true;
    }

    auto previous = m_oracle->canonicalState(m_visibleRoot);
    auto current = m_oracle->canonicalState(*liveToken);
    const bool compatible = previous && current && previous->words == current->words;

    if (compatible) {
        m_oracle->discard(*liveToken);
        return true;
    }

    const auto before = m_search.stats();
    accumulateCurrentSearchEpoch();
    m_search.reset(m_oracle.get(), "live-reroot");

    if (m_visibleRoot != kInvalidUniversalToken) {
        m_oracle->discard(m_visibleRoot);
    }
    m_visibleRoot = *liveToken;
    ++m_liveRerootCount;

    if (!m_search.begin(*m_oracle)) {
        enterError("A) LIVE SEARCH REROOT BEGIN FAILED: " + m_oracle->lastError());
        return false;
    }

    const auto after = m_search.stats();
    log::info(
        "LIVE_SEARCH_REROOT generation={} reroot={} previousCoreStage={} previousFrontier={} "
        "newFrontier={} accumulatedExpansions={} accumulatedEngineSteps={}",
        m_generation,
        m_liveRerootCount,
        searchStageName(before.stage),
        before.frontierSize,
        after.frontierSize,
        m_liveAccumulatedExpansions,
        m_liveAccumulatedEngineSteps
    );
    return true;
}

void UniversalRuntimeSession::searchSlice(std::size_t expansionBudget) {
    if (m_stage != UniversalRuntimeStage::Searching) return;
    if (!m_oracle) {
        enterError(fmt::format(
            "A) SEARCH CORE LOST AFTER BEGIN generation={} context=live-search-no-oracle",
            m_generation
        ));
        dumpLifecycle("live-search-no-oracle");
        return;
    }

    if (!syncLiveRoot()) return;
    if (!validateSearchCoreInvariant("live-search-slice-enter")) return;

    ++m_searchSliceCount;
    const auto before = m_search.stats();
    const bool traceSlice = m_searchSliceCount <= 8 || (m_searchSliceCount % 120U) == 0U;
    if (traceSlice) {
        const auto cumulative = stats();
        log::info(
            "LIVE_SEARCH_SLICE_ENTER generation={} slice={} coreStage={} frontier={} elapsed={:.6f}s "
            "expansions={} engineSteps={} reroots={} budget={}",
            m_generation,
            m_searchSliceCount,
            searchStageName(before.stage),
            before.frontierSize,
            cumulative.elapsedSeconds,
            cumulative.totalExpansions,
            cumulative.totalEngineSteps,
            m_liveRerootCount,
            expansionBudget
        );
    }

    if (before.stage == UniversalSearchStage::Searching
        || before.stage == UniversalSearchStage::Replaying) {
        m_search.work(*m_oracle, std::max<std::size_t>(1, expansionBudget));
    }

    const auto coreAfter = m_search.stats();

    if (!m_oracle->restore(m_visibleRoot)) {
        enterError("C) LIVE SEARCH ROUNDTRIP RESTORE FAILED: " + m_oracle->lastError());
        return;
    }

    const auto validation = m_oracle->validation();
    if (!validation.roundTripPass
        || !validation.queuedStatePass
        || !validation.attemptStatePass
        || !validation.pauseStatePass) {
        enterError("C) LIVE SEARCH ROUNDTRIP MISMATCH: " + m_oracle->lastError());
        return;
    }

    if (m_owner) m_owner->m_isPaused = false;

    if (m_search.ready()) {
        m_policy = m_search.policy();
        m_expectedStates = m_search.replayStates();
        m_fullPolicyAvailable = m_expectedStates.size() == m_policy.size() && !m_policy.empty();
        m_reason = m_fullPolicyAvailable
            ? "LIVE GLOBAL SEARCH: FULL VERIFIED POLICY AVAILABLE; MPC REMAINS ACTIVE"
            : "LIVE GLOBAL SEARCH: PARTIAL/INVALID FULL POLICY; MPC ACTIVE";
    } else if (coreAfter.stage == UniversalSearchStage::Exhausted) {
        if (!restartAtFinerResolution()) {
            m_reason = "LIVE GLOBAL SEARCH EXHAUSTED AT CURRENT ROOT; MPC ACTIVE";
        }
    } else if (coreAfter.stage == UniversalSearchStage::Error) {
        enterError("B) LIVE GLOBAL SEARCH ORACLE ERROR: " + m_oracle->lastError());
        return;
    } else {
        m_reason = "LIVE GLOBAL SEARCH + LOCAL MPC";
    }

    if (traceSlice) {
        const auto cumulative = stats();
        log::info(
            "LIVE_SEARCH_SLICE_EXIT generation={} slice={} coreStage={} frontier={} elapsed={:.6f}s "
            "expansions={} engineSteps={} reroots={} fullPolicyAvailable={}",
            m_generation,
            m_searchSliceCount,
            searchStageName(coreAfter.stage),
            coreAfter.frontierSize,
            cumulative.elapsedSeconds,
            cumulative.totalExpansions,
            cumulative.totalEngineSteps,
            m_liveRerootCount,
            m_fullPolicyAvailable ? "YES" : "NO"
        );
    }
}

void UniversalRuntimeSession::startPlayback() {
    if (!m_oracle || m_stage != UniversalRuntimeStage::Ready) return;

    if (!m_oracle->restore(m_visibleRoot)) {
        enterError("C) RESTORE INCORRECTO: ROOT RESTORE BEFORE PLAYBACK FAILED: " + m_oracle->lastError());
        return;
    }

    m_search.reset(m_oracle.get(), "startPlayback");
    if (m_lastSafePlayback != kInvalidUniversalToken) {
        m_oracle->discard(m_lastSafePlayback);
        m_lastSafePlayback = kInvalidUniversalToken;
    }

    m_playbackCursor = 0;
    m_accumulator = 0.0;
    m_stage = UniversalRuntimeStage::Playing;
    m_reason = m_recoveryCount == 0
        ? "AUTOPLAY ENGINE-VERIFIED POLICY"
        : "AUTOPLAY RECOVERED ENGINE-VERIFIED POLICY";

    if (m_owner) m_owner->m_isPaused = false;
}

void UniversalRuntimeSession::playbackFrame(double realDt) {
    if (m_stage == UniversalRuntimeStage::Ready) startPlayback();
    if (!m_oracle || m_stage != UniversalRuntimeStage::Playing) return;

    if (!std::isfinite(realDt) || realDt <= 0.0) realDt = m_oracle->stepDt();
    m_accumulator += realDt;

    const double step = m_oracle->stepDt();
    while (m_accumulator + 1e-12 >= step && m_stage == UniversalRuntimeStage::Playing) {
        if (m_playbackCursor >= m_policy.size()) {
            const auto final = m_oracle->observe();
            if (final.complete) {
                m_stage = UniversalRuntimeStage::Complete;
                m_reason = "LEVEL COMPLETION REACHED";
                if (m_owner) m_owner->m_isPaused = false;
            } else {
                enterError("I) COMPLETION NO SE DETECTA: VERIFIED POLICY ENDED BEFORE RUNTIME COMPLETION");
            }
            break;
        }

        if (m_lastSafePlayback != kInvalidUniversalToken) {
            m_oracle->discard(m_lastSafePlayback);
        }

        auto safe = m_oracle->capture();
        if (!safe) {
            enterError("C) RESTORE INCORRECTO: PLAYBACK SAFE SNAPSHOT FAILED: " + m_oracle->lastError());
            break;
        }
        m_lastSafePlayback = *safe;

        const auto expectedIndex = m_playbackCursor;
        const auto observation = m_oracle->step(m_policy[m_playbackCursor++]);
        m_accumulator -= step;

        if (!observation.valid) {
            const auto validation = m_oracle->validation();
            if (!validation.inputQueuePass) {
                enterError("G) READY PERO INPUT NO SALE: " + m_oracle->lastError());
            } else {
                enterError("B) ORACLE STEP NO AVANZA: RUNTIME OBSERVATION INVALID: " + m_oracle->lastError());
            }
            break;
        }

        if (observation.complete) {
            m_stage = UniversalRuntimeStage::Complete;
            m_reason = "LEVEL COMPLETION REACHED";
            if (m_owner) m_owner->m_isPaused = false;
            break;
        }

        if (observation.dead) {
            const auto recoveryToken = m_lastSafePlayback;
            m_lastSafePlayback = kInvalidUniversalToken;
            if (!beginRecoveryFromToken(recoveryToken, "PLAYBACK DIED BEFORE EXPECTED STATE")) {
                enterError("H) PLAYBACK DIVERGE: DESYNC RECOVERY FAILED AFTER DEATH: " + m_oracle->lastError());
            }
            break;
        }

        auto current = m_oracle->capture();
        if (!current) {
            enterError("C) RESTORE INCORRECTO: PLAYBACK CANONICAL SNAPSHOT FAILED: " + m_oracle->lastError());
            break;
        }

        if (!canonicalMatchesExpected(expectedIndex, *current)) {
            const auto recoveryToken = *current;
            if (!beginRecoveryFromToken(recoveryToken, "EXPECTED CANONICAL STATE MISMATCH")) {
                m_oracle->discard(recoveryToken);
                enterError("H) PLAYBACK DIVERGE: DESYNC RECOVERY FAILED: " + m_oracle->lastError());
            }
            break;
        }

        m_oracle->discard(*current);
    }
}

UniversalSearchStats UniversalRuntimeSession::stats() const {
    auto result = m_search.stats();
    result.totalExpansions += m_liveAccumulatedExpansions;
    result.totalEngineSteps += m_liveAccumulatedEngineSteps;
    result.bestProgress = std::max(result.bestProgress, m_liveBestProgress);

    if (m_liveStartedAt != std::chrono::steady_clock::time_point{}) {
        result.elapsedSeconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - m_liveStartedAt
        ).count();
    }
    if (result.elapsedSeconds > 1e-9) {
        result.expansionsPerSecond = static_cast<double>(result.totalExpansions)
            / result.elapsedSeconds;
    }
    return result;
}

double UniversalRuntimeSession::stepDt() const {
    return m_oracle ? m_oracle->stepDt() : 0.0;
}

RuntimeOracleValidation UniversalRuntimeSession::validation() const {
    return m_oracle ? m_oracle->validation() : RuntimeOracleValidation{};
}

} // namespace autobot::presolve
