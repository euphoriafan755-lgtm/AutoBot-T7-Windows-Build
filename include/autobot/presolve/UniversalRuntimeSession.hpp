#pragma once

#include "autobot/core/GameSnapshot.hpp"
#include "autobot/presolve/ShadowUniversalOracle.hpp"
#include "autobot/presolve/UniversalSearchCore.hpp"
#include "autobot/solver/PhysicsValidationHarness.hpp"
#include "autobot/world/CollisionWorld.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

class PlayLayer;

namespace autobot::presolve {

struct LiveRootAssessment {
    bool reroot = false;
    std::string_view reason = "root-reusable";
};

inline LiveRootAssessment assessLiveRoot(
    core::GameSnapshot const& root,
    core::GameSnapshot const& previous,
    core::GameSnapshot const& current,
    std::size_t framesSinceRoot,
    std::size_t reusablePolicyTicks
) {
    constexpr std::size_t kRollingRootWindowFrames = 24;

    if (!root.valid || !previous.valid || !current.valid) {
        return {true, "invalid-live-state"};
    }
    if (current.player.dead) {
        return {true, "death"};
    }
    if (current.levelTime + 0.02 < previous.levelTime
        || current.levelProgress + 0.5f < previous.levelProgress) {
        return {true, "attempt-restart"};
    }
    if (current.player.mode != previous.player.mode
        || current.player.mini != previous.player.mini
        || current.player.upsideDown != previous.player.upsideDown
        || current.dualMode != previous.dualMode) {
        return {true, "mode-or-portal-transition"};
    }

    const double dx = std::abs(current.player.x - previous.player.x);
    const double dy = std::abs(current.player.y - previous.player.y);
    const double maxDx = std::max(120.0, std::abs(previous.player.velocityX) * 8.0 + 30.0);
    const double maxDy = std::max(180.0, std::abs(previous.player.velocityY) * 8.0 + 60.0);
    if (dx > maxDx || dy > maxDy) {
        return {true, "state-discontinuity"};
    }

    if (reusablePolicyTicks > 0 && framesSinceRoot >= reusablePolicyTicks) {
        return {true, "policy-prefix-consumed"};
    }
    if (framesSinceRoot >= kRollingRootWindowFrames) {
        return {true, "rolling-window-advance"};
    }
    return {};
}

enum class UniversalRuntimeStage {
    Idle,
    Searching,
    Ready,
    Playing,
    Complete,
    Error,
};

class UniversalRuntimeSession final {
public:
    bool begin(
        PlayLayer* owner,
        core::GameSnapshot const& snapshot,
        world::CollisionWorld const* collisionWorld,
        solver::PhysicsValidationHarness const* validation,
        double completionBoundaryX,
        bool platformer,
        bool p1Holding
    );

    void updateLiveRoot(core::GameSnapshot const& snapshot, bool p1Holding);
    void reset(std::string_view reason = "explicit");
    void searchSlice(std::size_t expansionBudget);
    void fail(std::string reason);

    bool recoverFromStall(std::string reason);

    [[nodiscard]] PlayLayer* owner() const { return m_owner; }
    [[nodiscard]] UniversalRuntimeStage stage() const { return m_stage; }
    [[nodiscard]] bool searching() const { return m_stage == UniversalRuntimeStage::Searching; }
    [[nodiscard]] bool ready() const { return m_stage == UniversalRuntimeStage::Ready; }
    [[nodiscard]] bool playing() const { return m_stage == UniversalRuntimeStage::Playing; }
    [[nodiscard]] bool complete() const { return m_stage == UniversalRuntimeStage::Complete; }
    [[nodiscard]] bool error() const { return m_stage == UniversalRuntimeStage::Error; }
    [[nodiscard]] bool owns(PlayLayer* layer) const {
        return m_owner == layer && m_stage != UniversalRuntimeStage::Idle;
    }

    [[nodiscard]] std::string const& reason() const { return m_reason; }
    [[nodiscard]] UniversalSearchStats stats() const;
    [[nodiscard]] std::size_t policySize() const { return m_policy.size(); }
    [[nodiscard]] double stepDt() const;
    [[nodiscard]] std::size_t refinement() const { return m_refinement; }
    [[nodiscard]] std::size_t recoveryCount() const { return 0; }
    [[nodiscard]] std::size_t stallRecoveryCount() const { return m_stallRecoveryCount; }
    [[nodiscard]] std::uint64_t generation() const { return m_generation; }
    [[nodiscard]] std::size_t liveRerootCount() const { return m_liveRerootCount; }
    [[nodiscard]] bool fullPolicyAvailable() const { return m_fullPolicyAvailable; }
    [[nodiscard]] std::optional<bool> recommendedP1Hold() const {
        if (m_policy.empty() || m_liveFramesSinceRoot >= m_policy.size()) return std::nullopt;
        return m_policy[m_liveFramesSinceRoot].p1Hold;
    }
    [[nodiscard]] std::size_t liveFramesObserved() const { return m_liveFramesObserved; }
    [[nodiscard]] std::size_t liveFramesSinceRoot() const { return m_liveFramesSinceRoot; }
    [[nodiscard]] std::size_t internalGdUpdateCalls() const {
        return m_oracle ? m_oracle->internalGdUpdateCalls() : 0;
    }

private:
    bool rerootShadow(std::string_view reason);
    void accumulateCurrentSearchEpoch();
    bool validateSearchCoreInvariant(std::string_view context);
    void recordLifecycle(std::string message);
    void dumpLifecycle(std::string_view context) const;
    void enterError(std::string reason);

    PlayLayer* m_owner = nullptr;
    UniversalRuntimeStage m_stage = UniversalRuntimeStage::Idle;
    std::unique_ptr<ShadowUniversalOracle> m_oracle;
    UniversalSearchCore m_search;

    core::GameSnapshot m_pendingLiveSnapshot{};
    core::GameSnapshot m_rootLiveSnapshot{};
    core::GameSnapshot m_lastLiveSnapshot{};
    bool m_pendingP1Holding = false;
    bool m_liveRootDirty = false;
    std::size_t m_liveFramesObserved = 0;
    std::size_t m_liveFramesSinceRoot = 0;

    std::vector<UniversalAction> m_policy;
    std::size_t m_refinement = 0;
    std::size_t m_stallRecoveryCount = 0;
    std::size_t m_searchSliceCount = 0;
    std::size_t m_liveRerootCount = 0;
    std::size_t m_liveAccumulatedExpansions = 0;
    std::size_t m_liveAccumulatedEngineSteps = 0;
    double m_liveBestProgress = 0.0;
    bool m_fullPolicyAvailable = false;
    std::chrono::steady_clock::time_point m_liveStartedAt{};
    std::uint64_t m_generation = 0;
    std::string m_reason = "IDLE";
    std::deque<std::string> m_lifecycleTrace;
};

} // namespace autobot::presolve
