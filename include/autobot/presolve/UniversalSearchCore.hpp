#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
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
    std::size_t totalExpansions = 0;
    std::size_t frontierSize = 0;
    std::size_t currentDepth = 0;
    std::size_t uniqueStates = 0;
    std::size_t replayCursor = 0;
    double bestProgress = 0.0;
};

class UniversalSearchCore final {
public:
    bool begin(IUniversalStateOracle& oracle);
    UniversalSearchStats work(IUniversalStateOracle& oracle, std::size_t expansionBudget);
    void reset(IUniversalStateOracle* oracle = nullptr);

    [[nodiscard]] bool ready() const { return m_stage == UniversalSearchStage::Ready; }
    [[nodiscard]] bool searching() const {
        return m_stage == UniversalSearchStage::Searching || m_stage == UniversalSearchStage::Replaying;
    }
    [[nodiscard]] UniversalSearchStage stage() const { return m_stage; }
    [[nodiscard]] std::vector<UniversalAction> const& policy() const { return m_policy; }
    [[nodiscard]] UniversalSearchStats stats() const;

private:
    struct NodeMeta {
        std::size_t parent = std::numeric_limits<std::size_t>::max();
        UniversalAction action{};
        std::size_t depth = 0;
    };

    struct FrontierEntry {
        UniversalToken token = kInvalidUniversalToken;
        std::size_t metaIndex = 0;
        UniversalObservation observation{};
    };

    [[nodiscard]] static std::vector<UniversalAction> actionsFor(bool dual, bool platformer);
    void reconstructPolicy(std::size_t metaIndex);
    void clearFrontierTokens(IUniversalStateOracle& oracle);
    bool beginReplay(IUniversalStateOracle& oracle, std::size_t terminalMetaIndex);
    UniversalSearchStats replayWork(IUniversalStateOracle& oracle, std::size_t budget);

    UniversalSearchStage m_stage = UniversalSearchStage::Idle;
    UniversalToken m_rootToken = kInvalidUniversalToken;
    UniversalObservation m_rootObservation{};
    std::deque<FrontierEntry> m_frontier;
    std::deque<FrontierEntry> m_nextFrontier;
    std::vector<NodeMeta> m_meta;
    std::unordered_map<UniversalFingerprint, std::size_t, UniversalFingerprintHash> m_seenDepth;
    std::vector<UniversalAction> m_policy;
    std::size_t m_replayCursor = 0;
    std::size_t m_totalExpansions = 0;
    std::size_t m_currentDepth = 0;
    double m_bestProgress = 0.0;
};

} // namespace autobot::presolve
