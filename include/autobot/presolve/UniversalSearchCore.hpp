#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autobot::presolve {

struct UniversalAction {
    bool p1Hold = false;
    bool p1Left = false;
    bool p1Right = false;
    bool p2Hold = false;
    bool p2Left = false;
    bool p2Right = false;

    friend bool operator==(UniversalAction const&, UniversalAction const&) = default;
};

struct UniversalFingerprint {
    std::uint64_t lo = 0;
    std::uint64_t hi = 0;

    friend bool operator==(UniversalFingerprint const&, UniversalFingerprint const&) = default;
};

struct UniversalFingerprintHash {
    std::size_t operator()(UniversalFingerprint const& value) const noexcept {
        auto mix = value.lo ^ (value.hi + 0x9e3779b97f4a7c15ULL + (value.lo << 6U) + (value.lo >> 2U));
        return static_cast<std::size_t>(mix ^ (mix >> 32U));
    }
};

struct UniversalCanonicalState {
    UniversalFingerprint hash{};
    std::vector<std::uint64_t> words;
    bool completeRepresentation = false;

    bool temporalDominanceEligible = false;
    UniversalFingerprint temporalHash{};
    std::vector<std::uint64_t> temporalWords;

    [[nodiscard]] bool exactEquals(UniversalCanonicalState const& other) const {
        return words == other.words;
    }

    [[nodiscard]] bool temporalEquals(UniversalCanonicalState const& other) const {
        return temporalWords == other.temporalWords;
    }
};

struct UniversalObservation {
    bool valid = false;
    bool dead = false;
    bool complete = false;
    bool dual = false;
    bool platformer = false;
    double progress = 0.0;
    UniversalFingerprint fingerprint{};
};

using UniversalToken = std::uint64_t;
inline constexpr UniversalToken kInvalidUniversalToken = 0;

class IUniversalStateOracle {
public:
    virtual ~IUniversalStateOracle() = default;

    [[nodiscard]] virtual UniversalObservation observe() const = 0;
    [[nodiscard]] virtual std::optional<UniversalToken> capture() = 0;
    virtual bool restore(UniversalToken token) = 0;
    [[nodiscard]] virtual UniversalObservation step(UniversalAction action) = 0;
    virtual void discard(UniversalToken token) = 0;
    [[nodiscard]] virtual std::optional<UniversalCanonicalState> canonicalState(UniversalToken token) const = 0;

    // Changes only on discrete gameplay events that can alter which input is
    // correct (mode/gravity/grounded transitions, trigger activation, timers,
    // etc.). Continuous position/velocity changes must not increment it.
    // Macro stepping stops at an epoch change and returns control to search.
    [[nodiscard]] virtual std::uint64_t decisionEpoch() const { return 0; }
};

enum class UniversalSearchStage {
    Idle,
    Searching,
    Replaying,
    Ready,
    Exhausted,
    Error,
};

struct UniversalSearchStats {
    UniversalSearchStage stage = UniversalSearchStage::Idle;
    bool started = false;
    std::size_t resetSerial = 0;
    std::size_t totalExpansions = 0;
    std::size_t totalEngineSteps = 0;
    std::size_t frontierSize = 0;
    std::size_t currentDepth = 0;
    std::size_t uniqueStates = 0;
    std::size_t replayCursor = 0;
    std::size_t exactDedupHits = 0;
    std::size_t temporalDominanceHits = 0;
    std::size_t currentMacroTicks = 1;
    std::size_t strategyTier = 0;
    std::size_t stallRecoveries = 0;
    std::size_t estimatedMemoryBytes = 0;
    double bestProgress = 0.0;
    double elapsedSeconds = 0.0;
    double expansionsPerSecond = 0.0;
    double etaSeconds = -1.0;
};

class UniversalSearchCore final {
public:
    using LifecycleCallback = std::function<void(
        std::uint64_t generation,
        UniversalSearchStage previousStage,
        UniversalSearchStage newStage,
        std::string_view reason,
        std::size_t resetSerial
    )>;

    bool begin(IUniversalStateOracle& oracle);
    UniversalSearchStats work(IUniversalStateOracle& oracle, std::size_t expansionBudget);
    void reset(IUniversalStateOracle* oracle = nullptr, std::string_view reason = "explicit");
    void setLifecycleGeneration(std::uint64_t generation) { m_lifecycleGeneration = generation; }
    void setLifecycleCallback(LifecycleCallback callback) { m_lifecycleCallback = std::move(callback); }

    // Changes only search ordering/granularity. Existing states are kept, so a
    // stall recovery never throws away a potentially valid branch.
    bool refineStrategy();

    [[nodiscard]] bool ready() const { return m_stage == UniversalSearchStage::Ready; }
    [[nodiscard]] bool searching() const {
        return m_stage == UniversalSearchStage::Searching || m_stage == UniversalSearchStage::Replaying;
    }
    [[nodiscard]] UniversalSearchStage stage() const { return m_stage; }
    [[nodiscard]] std::vector<UniversalAction> const& policy() const { return m_policy; }
    [[nodiscard]] std::vector<UniversalAction> bestPrefix() const;
    [[nodiscard]] std::vector<UniversalCanonicalState> const& replayStates() const { return m_replayStates; }
    [[nodiscard]] UniversalSearchStats stats() const;

private:
    struct NodeMeta {
        std::size_t parent = std::numeric_limits<std::size_t>::max();
        UniversalAction action{};
        std::size_t depth = 0;
        std::size_t repeatTicks = 1;
    };

    struct FrontierEntry {
        UniversalToken token = kInvalidUniversalToken;
        std::size_t metaIndex = 0;
        UniversalObservation observation{};
        UniversalCanonicalState canonical{};
        std::size_t nextCandidate = 0;
        std::size_t expansionTier = std::numeric_limits<std::size_t>::max();
    };

    struct SeenState {
        std::vector<std::uint64_t> words;
        std::size_t depth = 0;
    };

    struct AdvanceResult {
        UniversalObservation observation{};
        std::size_t ticks = 0;
    };

    [[nodiscard]] static std::vector<UniversalAction> actionsFor(bool dual, bool platformer);
    [[nodiscard]] static std::vector<std::size_t> durationsFor(
        std::size_t strategyTier,
        bool dual,
        bool platformer
    );
    [[nodiscard]] static AdvanceResult advanceAction(
        IUniversalStateOracle& oracle,
        UniversalAction action,
        std::size_t ticks
    );

    void reconstructPolicy(std::size_t metaIndex);
    void clearFrontierTokens(IUniversalStateOracle& oracle);
    void prioritizeFrontier();
    bool beginReplay(IUniversalStateOracle& oracle, std::size_t terminalMetaIndex);
    UniversalSearchStats replayWork(IUniversalStateOracle& oracle, std::size_t budget);
    bool shouldPrune(UniversalCanonicalState const& canonical, std::size_t depth);
    void remember(UniversalCanonicalState const& canonical, std::size_t depth);

    UniversalSearchStage m_stage = UniversalSearchStage::Idle;
    UniversalToken m_rootToken = kInvalidUniversalToken;
    UniversalObservation m_rootObservation{};
    UniversalCanonicalState m_rootCanonical{};
    std::deque<FrontierEntry> m_frontier;
    std::deque<FrontierEntry> m_nextFrontier;
    std::vector<NodeMeta> m_meta;

    std::unordered_map<UniversalFingerprint, std::vector<SeenState>, UniversalFingerprintHash> m_seenBuckets;
    std::unordered_map<UniversalFingerprint, std::vector<SeenState>, UniversalFingerprintHash> m_temporalBuckets;

    std::vector<UniversalAction> m_policy;
    std::vector<UniversalCanonicalState> m_replayStates;
    std::size_t m_replayCursor = 0;
    UniversalToken m_replayToken = kInvalidUniversalToken;
    std::size_t m_bestMetaIndex = 0;
    std::size_t m_totalExpansions = 0;
    std::size_t m_totalEngineSteps = 0;
    std::size_t m_currentDepth = 0;
    std::size_t m_uniqueStates = 0;
    std::size_t m_exactDedupHits = 0;
    std::size_t m_temporalDominanceHits = 0;
    std::size_t m_currentMacroTicks = 1;
    std::size_t m_strategyTier = 0;
    std::size_t m_stallRecoveries = 0;
    std::size_t m_resetSerial = 0;
    std::uint64_t m_lifecycleGeneration = 0;
    LifecycleCallback m_lifecycleCallback;
    double m_bestProgress = 0.0;
    double m_rootProgress = 0.0;
    std::chrono::steady_clock::time_point m_startedAt{};
};

} // namespace autobot::presolve
