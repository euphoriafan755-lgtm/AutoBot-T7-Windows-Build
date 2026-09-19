#pragma once

#include "autobot/presolve/GeometryDashRuntimeOracle.hpp"
#include "autobot/presolve/UniversalSearchCore.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
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
    using StepCallback = GeometryDashRuntimeOracle::StepCallback;

    bool begin(PlayLayer* layer);
    void reset(std::string_view reason = "explicit");
    void setStepCallback(StepCallback callback);
    void searchSlice(std::size_t expansionBudget);
    void playbackFrame(double realDt);
    void fail(std::string reason);

    // A stall is recoverable. First change search granularity without dropping
    // states; only then restart from the visible root at a finer engine step.
    bool recoverFromStall(std::string reason);

    [[nodiscard]] PlayLayer* owner() const { return m_owner; }
    [[nodiscard]] UniversalRuntimeStage stage() const { return m_stage; }
    [[nodiscard]] bool searching() const { return m_stage == UniversalRuntimeStage::Searching; }
    [[nodiscard]] bool ready() const { return m_stage == UniversalRuntimeStage::Ready; }
    [[nodiscard]] bool playing() const { return m_stage == UniversalRuntimeStage::Playing; }
    [[nodiscard]] bool complete() const { return m_stage == UniversalRuntimeStage::Complete; }
    [[nodiscard]] bool error() const { return m_stage == UniversalRuntimeStage::Error; }
    [[nodiscard]] bool owns(PlayLayer* layer) const { return m_owner == layer && m_stage != UniversalRuntimeStage::Idle; }
    [[nodiscard]] std::string const& reason() const { return m_reason; }
    [[nodiscard]] UniversalSearchStats stats() const { return m_search.stats(); }
    [[nodiscard]] std::size_t policySize() const { return m_policy.size(); }
    [[nodiscard]] std::size_t playbackCursor() const { return m_playbackCursor; }
    [[nodiscard]] double stepDt() const;
    [[nodiscard]] std::size_t refinement() const { return m_refinement; }
    [[nodiscard]] std::size_t recoveryCount() const { return m_recoveryCount; }
    [[nodiscard]] std::size_t stallRecoveryCount() const { return m_stallRecoveryCount; }
    [[nodiscard]] std::uint64_t generation() const { return m_generation; }
    [[nodiscard]] RuntimeOracleValidation validation() const;

private:
    bool restartAtFinerResolution();
    bool beginRecoveryFromToken(UniversalToken token, std::string reason);
    bool canonicalMatchesExpected(std::size_t index, UniversalToken token) const;
    bool validateSearchCoreInvariant(std::string_view context);
    void recordLifecycle(std::string message);
    void dumpLifecycle(std::string_view context) const;
    void enterError(std::string reason);
    void startPlayback();

    PlayLayer* m_owner = nullptr;
    UniversalRuntimeStage m_stage = UniversalRuntimeStage::Idle;
    std::unique_ptr<GeometryDashRuntimeOracle> m_oracle;
    UniversalSearchCore m_search;
    UniversalToken m_visibleRoot = kInvalidUniversalToken;
    UniversalToken m_lastSafePlayback = kInvalidUniversalToken;
    std::vector<UniversalAction> m_policy;
    std::vector<UniversalCanonicalState> m_expectedStates;
    std::size_t m_playbackCursor = 0;
    std::size_t m_refinement = 0;
    std::size_t m_recoveryCount = 0;
    std::size_t m_stallRecoveryCount = 0;
    std::size_t m_searchSliceCount = 0;
    std::uint64_t m_generation = 0;
    double m_accumulator = 0.0;
    std::string m_reason = "IDLE";
    std::deque<std::string> m_lifecycleTrace;
};

} // namespace autobot::presolve
