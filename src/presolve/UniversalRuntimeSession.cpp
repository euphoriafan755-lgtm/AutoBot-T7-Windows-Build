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

void UniversalRuntimeSession::enterError(std::string reason) {
    const auto previous = m_stage;
    m_stage = UniversalRuntimeStage::Error;
    m_reason = std::move(reason);
    recordLifecycle(fmt::format(
        "RUNTIME_ERROR generation={} previousStage={} newStage=Error reason={}",
        m_generation,
        runtimeStageName(previous),
        m_reason
    ));
}

bool UniversalRuntimeSession::validateSearchCoreInvariant(std::string_view context) {
    if (m_stage != UniversalRuntimeStage::Searching) return true;

    const auto core = m_search.stats();
    const bool stageOk = core.stage == UniversalSearchStage::Searching
        || core.stage == UniversalSearchStage::Replaying
        || core.stage == UniversalSearchStage::Ready;
    const bool frontierOk = core.stage != UniversalSearchStage::Searching
        || core.frontierSize > 0;
    const bool startedOk = core.started;

    if (stageOk && frontierOk && startedOk) return true;

    const auto reason = fmt::format(
        "A) SHADOW SEARCH CORE LOST generation={} context={} runtimeStage={} coreStage={} "
        "started={} frontier={} expansions={} engineSteps={} resetSerial={}",
        m_generation,
        context,
        runtimeStageName(m_stage),
        searchStageName(core.stage),
        core.started ? "YES" : "NO",
        core.frontierSize,
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

bool UniversalRuntimeSession::begin(
    PlayLayer* owner,
    core::GameSnapshot const& snapshot,
    world::CollisionWorld const* collisionWorld,
    solver::PhysicsValidationHarness const* validation,
    double completionBoundaryX,
    bool platformer,
    bool p1Holding
) {
    if (!owner || !snapshot.valid || !collisionWorld || !collisionWorld->ready()) {
        enterError("A) SHADOW SEARCH BEGIN INPUT INVALID");
        return false;
    }

    const auto previousStage = m_stage;
    ++m_generation;
    m_lifecycleTrace.clear();
    m_searchSliceCount = 0;
    m_liveRerootCount = 0;
    m_liveFramesObserved = 0;
    m_liveFramesSinceRoot = 0;
    m_liveAccumulatedExpansions = 0;
    m_liveAccumulatedEngineSteps = 0;
    m_liveBestProgress = static_cast<double>(snapshot.levelProgress);
    m_fullPolicyAvailable = false;
    m_liveStartedAt = std::chrono::steady_clock::now();
    m_owner = owner;

    m_search.setLifecycleGeneration(m_generation);
    m_search.setLifecycleCallback([this](
        std::uint64_t generation,
        UniversalSearchStage previous,
        UniversalSearchStage next,
        std::string_view reason,
        std::size_t resetSerial
    ) {
        const auto event = fmt::format(
            "SHADOW_SEARCH_CORE_RESET generation={} reason={} previousStage={} newStage={} resetSerial={}",
            generation,
            reason,
            searchStageName(previous),
            searchStageName(next),
            resetSerial
        );
        log::info("{}", event);
        recordLifecycle(event);
    });

    m_oracle = std::make_unique<ShadowUniversalOracle>();
    m_oracle->configure(
        collisionWorld,
        validation,
        completionBoundaryX,
        platformer
    );

    m_pendingLiveSnapshot = snapshot;
    m_rootLiveSnapshot = snapshot;
    m_lastLiveSnapshot = snapshot;
    m_pendingP1Holding = p1Holding;
    m_liveRootDirty = true;
    m_stage = UniversalRuntimeStage::Searching;
    m_reason = "SHADOW GLOBAL SEARCH + LOCAL MPC";

    log::info(
        "RUNTIME_BEGIN generation={} previousStage={} newStage=Searching backend=SHADOW",
        m_generation,
        runtimeStageName(previousStage)
    );

    if (!rerootShadow("initial-live-root")) {
        return false;
    }

    const auto bootstrap = m_search.stats();
    log::info(
        "SHADOW_SEARCH_BEGIN generation={} frontier={} coreStage={} started={} "
        "expansions={} engineSteps={} internalGdUpdates={} checkpointCalls={}",
        m_generation,
        bootstrap.frontierSize,
        searchStageName(bootstrap.stage),
        bootstrap.started ? "YES" : "NO",
        bootstrap.totalExpansions,
        bootstrap.totalEngineSteps,
        ShadowUniversalOracle::internalGdUpdateCalls(),
        ShadowUniversalOracle::checkpointCalls()
    );

    return validateSearchCoreInvariant("immediately-after-shadow-begin");
}

void UniversalRuntimeSession::reset(std::string_view reason) {
    const auto previous = m_stage;
    const auto before = m_search.stats();

    log::info(
        "RUNTIME_RESET generation={} reason={} previousStage={} newStage=Idle "
        "coreStage={} frontier={} expansions={} engineSteps={}",
        m_generation,
        reason,
        runtimeStageName(previous),
        searchStageName(before.stage),
        before.frontierSize,
        before.totalExpansions,
        before.totalEngineSteps
    );

    m_search.reset(m_oracle.get(), reason);
    m_owner = nullptr;
    m_oracle.reset();
    m_pendingLiveSnapshot = {};
    m_rootLiveSnapshot = {};
    m_lastLiveSnapshot = {};
    m_pendingP1Holding = false;
    m_liveRootDirty = false;
    m_liveFramesObserved = 0;
    m_liveFramesSinceRoot = 0;
    m_policy.clear();
    m_refinement = 0;
    m_stallRecoveryCount = 0;
    m_searchSliceCount = 0;
    m_liveRerootCount = 0;
    m_liveAccumulatedExpansions = 0;
    m_liveAccumulatedEngineSteps = 0;
    m_liveBestProgress = 0.0;
    m_fullPolicyAvailable = false;
    m_liveStartedAt = {};
    m_stage = UniversalRuntimeStage::Idle;
    m_reason = "IDLE";
}

void UniversalRuntimeSession::updateLiveRoot(
    core::GameSnapshot const& snapshot,
    bool p1Holding
) {
    if (m_stage != UniversalRuntimeStage::Searching || !snapshot.valid) return;

    const bool sameFrame = m_pendingLiveSnapshot.valid
        && snapshot.solverSampleID == m_pendingLiveSnapshot.solverSampleID
        && std::abs(snapshot.levelTime - m_pendingLiveSnapshot.levelTime) <= 1e-9;
    if (sameFrame) {
        m_pendingP1Holding = p1Holding;
        return;
    }

    if (!m_lastLiveSnapshot.valid) {
        m_lastLiveSnapshot = snapshot;
        m_pendingLiveSnapshot = snapshot;
        m_pendingP1Holding = p1Holding;
        m_liveRootDirty = true;
        return;
    }

    ++m_liveFramesObserved;
    ++m_liveFramesSinceRoot;

    const auto assessment = assessLiveRoot(
        m_rootLiveSnapshot,
        m_lastLiveSnapshot,
        snapshot,
        m_liveFramesSinceRoot,
        m_policy.size()
    );

    m_pendingLiveSnapshot = snapshot;
    m_pendingP1Holding = p1Holding;
    m_lastLiveSnapshot = snapshot;

    if (assessment.reroot) {
        m_liveRootDirty = true;
        log::info(
            "ROLLING_SEARCH_REROOT_REQUEST generation={} reason={} liveFrames={} framesSinceRoot={} "
            "policyTicks={} rerootCount={}",
            m_generation,
            assessment.reason,
            m_liveFramesObserved,
            m_liveFramesSinceRoot,
            m_policy.size(),
            m_liveRerootCount
        );
    }
}

void UniversalRuntimeSession::accumulateCurrentSearchEpoch() {
    const auto current = m_search.stats();
    m_liveAccumulatedExpansions += current.totalExpansions;
    m_liveAccumulatedEngineSteps += current.totalEngineSteps;
    m_liveBestProgress = std::max(m_liveBestProgress, current.bestProgress);
}

bool UniversalRuntimeSession::rerootShadow(std::string_view reason) {
    if (!m_oracle || !m_pendingLiveSnapshot.valid) {
        enterError("A) SHADOW REROOT HAS NO LIVE SNAPSHOT");
        return false;
    }

    if (m_search.stats().started) {
        accumulateCurrentSearchEpoch();
        m_search.reset(m_oracle.get(), reason);
        ++m_liveRerootCount;
    }

    if (!m_oracle->setLiveRoot(m_pendingLiveSnapshot, m_pendingP1Holding)) {
        enterError("A) SHADOW ROOT COPY FAILED: " + m_oracle->lastError());
        return false;
    }

    if (!m_search.begin(*m_oracle)) {
        enterError("A) SHADOW SEARCH BEGIN FAILED: " + m_oracle->lastError());
        return false;
    }

    m_liveRootDirty = false;
    m_fullPolicyAvailable = false;
    m_policy.clear();
    m_rootLiveSnapshot = m_pendingLiveSnapshot;
    m_lastLiveSnapshot = m_pendingLiveSnapshot;
    m_liveFramesSinceRoot = 0;

    const auto after = m_search.stats();
    log::info(
        "SHADOW_SEARCH_REROOT generation={} reroot={} sample={} levelTime={:.6f} "
        "frontier={} accumulatedExpansions={} accumulatedEngineSteps={}",
        m_generation,
        m_liveRerootCount,
        m_pendingLiveSnapshot.solverSampleID,
        m_pendingLiveSnapshot.levelTime,
        after.frontierSize,
        m_liveAccumulatedExpansions,
        m_liveAccumulatedEngineSteps
    );
    return true;
}

void UniversalRuntimeSession::searchSlice(std::size_t expansionBudget) {
    if (m_stage != UniversalRuntimeStage::Searching) return;
    if (!m_oracle) {
        enterError("A) SHADOW SEARCH ORACLE MISSING");
        return;
    }

    if (m_liveRootDirty && !rerootShadow("live-shadow-reroot")) return;
    if (!validateSearchCoreInvariant("shadow-search-slice-enter")) return;

    ++m_searchSliceCount;
    const auto before = m_search.stats();
    const bool trace = m_searchSliceCount <= 8 || (m_searchSliceCount % 120U) == 0U;

    if (trace) {
        const auto cumulative = stats();
        log::info(
            "SHADOW_SEARCH_SLICE_ENTER generation={} slice={} coreStage={} frontier={} "
            "elapsed={:.6f}s expansions={} engineSteps={} internalGdUpdates=0 budget={}",
            m_generation,
            m_searchSliceCount,
            searchStageName(before.stage),
            before.frontierSize,
            cumulative.elapsedSeconds,
            cumulative.totalExpansions,
            cumulative.totalEngineSteps,
            expansionBudget
        );
    }

    if (before.stage == UniversalSearchStage::Searching
        || before.stage == UniversalSearchStage::Replaying) {
        m_search.work(*m_oracle, std::max<std::size_t>(1, expansionBudget));
    }

    const auto after = m_search.stats();

    if (m_search.ready()) {
        m_policy = m_search.policy();
        m_fullPolicyAvailable = !m_policy.empty();
        m_reason = m_fullPolicyAvailable
            ? "SHADOW GLOBAL FULL POLICY AVAILABLE; LOCAL MPC ACTIVE"
            : "SHADOW GLOBAL SEARCH READY WITHOUT POLICY; LOCAL MPC ACTIVE";
    } else if (after.stage == UniversalSearchStage::Searching
        || after.stage == UniversalSearchStage::Replaying) {
        auto prefix = m_search.bestPrefix();
        if (!prefix.empty()) m_policy = std::move(prefix);
        m_fullPolicyAvailable = false;
        m_reason = "SHADOW GLOBAL PREFIX + LOCAL MPC";
    } else if (after.stage == UniversalSearchStage::Error) {
        m_reason = "SHADOW GLOBAL SEARCH ERROR; LOCAL MPC CONTINUES";
        m_liveRootDirty = true;
    } else if (after.stage == UniversalSearchStage::Exhausted) {
        m_reason = "SHADOW GLOBAL SEARCH EXHAUSTED; LOCAL MPC CONTINUES";
        m_liveRootDirty = true;
    } else {
        m_reason = "SHADOW GLOBAL SEARCH + LOCAL MPC";
    }

    if (trace) {
        const auto cumulative = stats();
        log::info(
            "SHADOW_SEARCH_SLICE_EXIT generation={} slice={} coreStage={} frontier={} "
            "elapsed={:.6f}s expansions={} engineSteps={} internalGdUpdates=0 fullPolicyAvailable={}",
            m_generation,
            m_searchSliceCount,
            searchStageName(after.stage),
            after.frontierSize,
            cumulative.elapsedSeconds,
            cumulative.totalExpansions,
            cumulative.totalEngineSteps,
            m_fullPolicyAvailable ? "YES" : "NO"
        );
    }
}

bool UniversalRuntimeSession::recoverFromStall(std::string reason) {
    if (!m_oracle || m_stage != UniversalRuntimeStage::Searching) return false;
    ++m_stallRecoveryCount;
    if (m_search.refineStrategy()) {
        ++m_refinement;
        m_reason = "SHADOW SEARCH STALL -> FINER STRATEGY: " + reason;
        return true;
    }
    m_liveRootDirty = true;
    m_reason = "SHADOW SEARCH STALL -> LIVE REROOT: " + reason;
    return true;
}

void UniversalRuntimeSession::fail(std::string reason) {
    enterError(std::move(reason));
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

} // namespace autobot::presolve
