#pragma once

#include "autobot/core/GameSnapshot.hpp"
#include "autobot/presolve/ShadowUniversalOracle.hpp"
#include "autobot/presolve/UniversalSearchCore.hpp"
#include "autobot/solver/PhysicsValidationHarness.hpp"
#include "autobot/world/CollisionWorld.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

class PlayLayer;

namespace autobot::presolve {

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
    bool m_pendingP1Holding = false;
    bool m_liveRootDirty = false;

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
