#include "autobot/presolve/UniversalSearchCore.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace autobot::presolve {

std::vector<UniversalAction> UniversalSearchCore::actionsFor(bool dual, bool platformer) {
    if (!platformer) {
        if (dual) {
            return {
                {false, false, false, false, false, false},
                {true,  false, false, false, false, false},
                {false, false, false, true,  false, false},
                {true,  false, false, true,  false, false},
            };
        }
        return {
            {false, false, false, false, false, false},
            {true,  false, false, false, false, false},
        };
    }

    struct PlayerAction { bool jump; bool left; bool right; };
    constexpr std::array<PlayerAction, 6> single{{
        {false, false, false},
        {false, true,  false},
        {false, false, true },
        {true,  false, false},
        {true,  true,  false},
        {true,  false, true },
    }};

    std::vector<UniversalAction> result;
    result.reserve(dual ? single.size() * single.size() : single.size());
    for (auto const p1 : single) {
        if (!dual) {
            result.push_back({p1.jump, p1.left, p1.right, false, false, false});
            continue;
        }
        for (auto const p2 : single) {
            result.push_back({p1.jump, p1.left, p1.right, p2.jump, p2.left, p2.right});
        }
    }
    return result;
}

std::vector<std::size_t> UniversalSearchCore::durationsFor(
    std::size_t strategyTier,
    bool dual,
    bool platformer
) {
    strategyTier = std::min<std::size_t>(strategyTier, 3);

    if (platformer && dual) {
        switch (strategyTier) {
            case 0: return {8, 2, 1};
            case 1: return {6, 2, 1};
            case 2: return {4, 2, 1};
            default: return {2, 1};
        }
    }

    if (platformer) {
        switch (strategyTier) {
            case 0: return {24, 8, 2, 1};
            case 1: return {16, 6, 2, 1};
            case 2: return {8, 4, 2, 1};
            default: return {4, 2, 1};
        }
    }

    switch (strategyTier) {
        case 0: return {64, 24, 8, 2, 1};
        case 1: return {32, 12, 4, 1};
        case 2: return {16, 6, 2, 1};
        default: return {8, 4, 2, 1};
    }
}

UniversalSearchCore::AdvanceResult UniversalSearchCore::advanceAction(
    IUniversalStateOracle& oracle,
    UniversalAction action,
    std::size_t ticks
) {
    AdvanceResult result{};
    result.observation = oracle.observe();
    auto epoch = oracle.decisionEpoch();

    for (std::size_t i = 0; i < std::max<std::size_t>(1, ticks); ++i) {
        result.observation = oracle.step(action);
        ++result.ticks;
        if (!result.observation.valid || result.observation.dead || result.observation.complete) break;

        const auto nextEpoch = oracle.decisionEpoch();
        if (nextEpoch != epoch) {
            break;
        }
        epoch = nextEpoch;
    }
    return result;
}

void UniversalSearchCore::clearFrontierTokens(IUniversalStateOracle& oracle) {
    for (auto const& entry : m_frontier) {
        if (entry.token != kInvalidUniversalToken && entry.token != m_rootToken) oracle.discard(entry.token);
    }
    for (auto const& entry : m_nextFrontier) {
        if (entry.token != kInvalidUniversalToken && entry.token != m_rootToken) oracle.discard(entry.token);
    }
    m_frontier.clear();
    m_nextFrontier.clear();
}

void UniversalSearchCore::reset(IUniversalStateOracle* oracle) {
    if (oracle) {
        clearFrontierTokens(*oracle);
        if (m_replayToken != kInvalidUniversalToken && m_replayToken != m_rootToken) {
            oracle->discard(m_replayToken);
        }
        if (m_rootToken != kInvalidUniversalToken) oracle->discard(m_rootToken);
    } else {
        m_frontier.clear();
        m_nextFrontier.clear();
    }

    m_stage = UniversalSearchStage::Idle;
    m_rootToken = kInvalidUniversalToken;
    m_rootObservation = {};
    m_rootCanonical = {};
    m_meta.clear();
    m_seenBuckets.clear();
    m_temporalBuckets.clear();
    m_policy.clear();
    m_replayStates.clear();
    m_replayCursor = 0;
    m_replayToken = kInvalidUniversalToken;
    m_totalExpansions = 0;
    m_totalEngineSteps = 0;
    m_currentDepth = 0;
    m_uniqueStates = 0;
    m_exactDedupHits = 0;
    m_temporalDominanceHits = 0;
    m_currentMacroTicks = 1;
    m_strategyTier = 0;
    m_stallRecoveries = 0;
    m_bestProgress = 0.0;
    m_rootProgress = 0.0;
    m_startedAt = {};
}

bool UniversalSearchCore::refineStrategy() {
    if (m_strategyTier >= 3) return false;
    ++m_strategyTier;
    ++m_stallRecoveries;
    prioritizeFrontier();
    return true;
}

bool UniversalSearchCore::shouldPrune(UniversalCanonicalState const& canonical, std::size_t depth) {
    if (canonical.temporalDominanceEligible) {
        auto it = m_temporalBuckets.find(canonical.temporalHash);
        if (it != m_temporalBuckets.end()) {
            for (auto const& seen : it->second) {
                if (seen.depth <= depth && seen.words == canonical.temporalWords) {
                    ++m_temporalDominanceHits;
                    return true;
                }
            }
        }
    }

    if (!canonical.completeRepresentation) return false;

    auto it = m_seenBuckets.find(canonical.hash);
    if (it == m_seenBuckets.end()) return false;
    for (auto const& seen : it->second) {
        if (seen.depth <= depth && seen.words == canonical.words) {
            ++m_exactDedupHits;
            return true;
        }
    }
    return false;
}

void UniversalSearchCore::remember(UniversalCanonicalState const& canonical, std::size_t depth) {
    if (canonical.temporalDominanceEligible) {
        auto& bucket = m_temporalBuckets[canonical.temporalHash];
        bool found = false;
        for (auto& seen : bucket) {
            if (seen.words == canonical.temporalWords) {
                seen.depth = std::min(seen.depth, depth);
                found = true;
                break;
            }
        }
        if (!found) bucket.push_back({canonical.temporalWords, depth});
    }

    if (canonical.completeRepresentation) {
        auto& bucket = m_seenBuckets[canonical.hash];
        bool found = false;
        for (auto& seen : bucket) {
            if (seen.words == canonical.words) {
                seen.depth = std::min(seen.depth, depth);
                found = true;
                break;
            }
        }
        if (!found) bucket.push_back({canonical.words, depth});
    }
    ++m_uniqueStates;
}

bool UniversalSearchCore::begin(IUniversalStateOracle& oracle) {
    reset(&oracle);
    m_startedAt = std::chrono::steady_clock::now();

    const auto root = oracle.observe();
    if (!root.valid || root.dead) {
        m_stage = UniversalSearchStage::Error;
        return false;
    }

    auto token = oracle.capture();
    if (!token || *token == kInvalidUniversalToken) {
        m_stage = UniversalSearchStage::Error;
        return false;
    }

    auto canonical = oracle.canonicalState(*token);
    if (!canonical) {
        oracle.discard(*token);
        m_stage = UniversalSearchStage::Error;
        return false;
    }

    m_rootToken = *token;
    m_rootObservation = root;
    m_rootCanonical = *canonical;
    m_meta.push_back(NodeMeta{});
    remember(*canonical, 0);
    m_frontier.push_back({m_rootToken, 0, root, *canonical});
    m_bestProgress = root.progress;
    m_rootProgress = root.progress;
    m_stage = root.complete ? UniversalSearchStage::Ready : UniversalSearchStage::Searching;
    return true;
}

void UniversalSearchCore::reconstructPolicy(std::size_t metaIndex) {
    m_policy.clear();
    while (metaIndex != 0 && metaIndex < m_meta.size()) {
        auto const& node = m_meta[metaIndex];
        for (std::size_t i = 0; i < node.repeatTicks; ++i) {
            m_policy.push_back(node.action);
        }
        metaIndex = node.parent;
    }
    std::reverse(m_policy.begin(), m_policy.end());
}

void UniversalSearchCore::prioritizeFrontier() {
    auto better = [this](FrontierEntry const& a, FrontierEntry const& b) {
        if (std::abs(a.observation.progress - b.observation.progress) > 1e-9) {
            return a.observation.progress > b.observation.progress;
        }
        const auto da = a.metaIndex < m_meta.size() ? m_meta[a.metaIndex].depth : 0;
        const auto db = b.metaIndex < m_meta.size() ? m_meta[b.metaIndex].depth : 0;
        return da < db;
    };

    if (m_frontier.size() > 1) {
        auto begin = m_frontier.begin();
        if (m_frontier.front().nextCandidate > 0) ++begin;
        if (std::distance(begin, m_frontier.end()) > 1) {
            std::stable_sort(begin, m_frontier.end(), better);
        }
    }
    if (m_nextFrontier.size() > 1) {
        std::stable_sort(m_nextFrontier.begin(), m_nextFrontier.end(), better);
    }
}

bool UniversalSearchCore::beginReplay(IUniversalStateOracle& oracle, std::size_t terminalMetaIndex) {
    reconstructPolicy(terminalMetaIndex);
    clearFrontierTokens(oracle);

    if (!oracle.restore(m_rootToken)) {
        m_stage = UniversalSearchStage::Error;
        return false;
    }

    const auto restored = oracle.observe();
    if (!restored.valid || restored.dead) {
        m_stage = UniversalSearchStage::Error;
        return false;
    }

    if (m_replayToken != kInvalidUniversalToken && m_replayToken != m_rootToken) {
        oracle.discard(m_replayToken);
    }
    m_replayStates.clear();
    m_replayCursor = 0;
    m_replayToken = m_rootToken;
    m_stage = UniversalSearchStage::Replaying;
    return true;
}

UniversalSearchStats UniversalSearchCore::replayWork(IUniversalStateOracle& oracle, std::size_t budget) {
    if (m_stage != UniversalSearchStage::Replaying || budget == 0) return stats();
    if (m_replayToken == kInvalidUniversalToken || !oracle.restore(m_replayToken)) {
        m_stage = UniversalSearchStage::Error;
        return stats();
    }

    std::size_t done = 0;
    while (done < budget && m_replayCursor < m_policy.size()) {
        const auto observation = oracle.step(m_policy[m_replayCursor]);
        ++m_replayCursor;
        ++m_totalEngineSteps;
        ++done;

        if (!observation.valid || observation.dead) {
            m_stage = UniversalSearchStage::Error;
            break;
        }

        auto nextReplayToken = oracle.capture();
        if (!nextReplayToken) {
            m_stage = UniversalSearchStage::Error;
            break;
        }

        auto canonical = oracle.canonicalState(*nextReplayToken);
        if (!canonical) {
            oracle.discard(*nextReplayToken);
            m_stage = UniversalSearchStage::Error;
            break;
        }

        if (m_replayToken != kInvalidUniversalToken && m_replayToken != m_rootToken) {
            oracle.discard(m_replayToken);
        }
        m_replayToken = *nextReplayToken;
        m_replayStates.push_back(std::move(*canonical));
        m_bestProgress = std::max(m_bestProgress, observation.progress);

        if (observation.complete) {
            m_stage = UniversalSearchStage::Ready;
            break;
        }
    }

    if (m_stage == UniversalSearchStage::Replaying && m_replayCursor >= m_policy.size()) {
        const auto observation = oracle.observe();
        m_stage = observation.valid && !observation.dead && observation.complete
            ? UniversalSearchStage::Ready
            : UniversalSearchStage::Error;
    }

    if (m_stage == UniversalSearchStage::Ready || m_stage == UniversalSearchStage::Error) {
        if (m_replayToken != kInvalidUniversalToken && m_replayToken != m_rootToken) {
            oracle.discard(m_replayToken);
        }
        m_replayToken = kInvalidUniversalToken;
    }

    if (m_stage == UniversalSearchStage::Ready) {
        if (!oracle.restore(m_rootToken)) m_stage = UniversalSearchStage::Error;
    }
    return stats();
}

UniversalSearchStats UniversalSearchCore::work(IUniversalStateOracle& oracle, std::size_t expansionBudget) {
    if (m_stage == UniversalSearchStage::Replaying) return replayWork(oracle, expansionBudget);
    if (m_stage != UniversalSearchStage::Searching || expansionBudget == 0) return stats();

    std::size_t expandedThisCall = 0;
    while (expandedThisCall < expansionBudget && m_stage == UniversalSearchStage::Searching) {
        if (m_frontier.empty()) {
            if (m_nextFrontier.empty()) {
                m_stage = UniversalSearchStage::Exhausted;
                break;
            }
            prioritizeFrontier();
            m_frontier.swap(m_nextFrontier);
        }

        auto entry = std::move(m_frontier.front());
        m_frontier.pop_front();
        if (entry.metaIndex >= m_meta.size()) {
            if (entry.token != m_rootToken) oracle.discard(entry.token);
            m_stage = UniversalSearchStage::Error;
            break;
        }

        const auto parentDepth = m_meta[entry.metaIndex].depth;
        m_currentDepth = std::max(m_currentDepth, parentDepth);

        auto actions = actionsFor(entry.observation.dual, entry.observation.platformer);
        if (parentDepth > 0) {
            auto const previous = m_meta[entry.metaIndex].action;
            std::stable_sort(actions.begin(), actions.end(), [&](UniversalAction const& a, UniversalAction const& b) {
                return (a == previous) && !(b == previous);
            });
        }

        if (entry.expansionTier == std::numeric_limits<std::size_t>::max()) {
            entry.expansionTier = m_strategyTier;
        }
        const auto durations = durationsFor(
            entry.expansionTier,
            entry.observation.dual,
            entry.observation.platformer
        );
        const auto candidateCount = actions.size() * durations.size();

        while (entry.nextCandidate < candidateCount
            && expandedThisCall < expansionBudget
            && m_stage == UniversalSearchStage::Searching) {
            const auto candidate = entry.nextCandidate++;
            const auto actionIndex = candidate / durations.size();
            const auto durationIndex = candidate % durations.size();
            const auto action = actions[actionIndex];
            const auto duration = durations[durationIndex];

            if (!oracle.restore(entry.token)) {
                m_stage = UniversalSearchStage::Error;
                break;
            }

            m_currentMacroTicks = duration;
            const auto advanced = advanceAction(oracle, action, duration);
            ++m_totalExpansions;
            ++expandedThisCall;
            m_totalEngineSteps += advanced.ticks;

            const auto observation = advanced.observation;
            if (!observation.valid || observation.dead || advanced.ticks == 0) continue;

            m_bestProgress = std::max(m_bestProgress, observation.progress);
            const auto childDepth = parentDepth + advanced.ticks;
            m_currentDepth = std::max(m_currentDepth, childDepth);

            auto token = oracle.capture();
            if (!token || *token == kInvalidUniversalToken) {
                m_stage = UniversalSearchStage::Error;
                break;
            }

            auto canonical = oracle.canonicalState(*token);
            if (!canonical) {
                oracle.discard(*token);
                m_stage = UniversalSearchStage::Error;
                break;
            }

            if (shouldPrune(*canonical, childDepth)) {
                oracle.discard(*token);
                continue;
            }
            remember(*canonical, childDepth);

            const auto childMeta = m_meta.size();
            m_meta.push_back({entry.metaIndex, action, childDepth, advanced.ticks});

            if (observation.complete) {
                oracle.discard(*token);
                if (entry.token != m_rootToken) oracle.discard(entry.token);
                if (!beginReplay(oracle, childMeta)) break;

                const auto remaining = expansionBudget - expandedThisCall;
                return remaining > 0 ? replayWork(oracle, remaining) : stats();
            }

            FrontierEntry child{*token, childMeta, observation, std::move(*canonical)};
            if (observation.progress > entry.observation.progress + 1e-9) {
                m_frontier.push_back(std::move(child));
            } else {
                m_nextFrontier.push_back(std::move(child));
            }
        }

        if (m_stage != UniversalSearchStage::Searching) {
            if (entry.token != m_rootToken) oracle.discard(entry.token);
            break;
        }

        if (entry.nextCandidate < candidateCount) {
            // Preserve the exact decision-state continuation at the front. No
            // child may overtake it until every candidate from this parent has
            // been considered, matching an uninterrupted expansion exactly.
            m_frontier.push_front(std::move(entry));
            break;
        }

        if (entry.token != m_rootToken) oracle.discard(entry.token);

        // This decision state is now complete. Only at this boundary may the
        // best-first ordering choose the next state.
        prioritizeFrontier();
    }

    if (m_stage == UniversalSearchStage::Replaying && expandedThisCall < expansionBudget) {
        return replayWork(oracle, expansionBudget - expandedThisCall);
    }
    return stats();
}

UniversalSearchStats UniversalSearchCore::stats() const {
    UniversalSearchStats result{};
    result.stage = m_stage;
    result.totalExpansions = m_totalExpansions;
    result.totalEngineSteps = m_totalEngineSteps;
    result.frontierSize = m_frontier.size() + m_nextFrontier.size();
    result.currentDepth = m_currentDepth;
    result.uniqueStates = m_uniqueStates;
    result.replayCursor = m_replayCursor;
    result.exactDedupHits = m_exactDedupHits;
    result.temporalDominanceHits = m_temporalDominanceHits;
    result.currentMacroTicks = m_currentMacroTicks;
    result.strategyTier = m_strategyTier;
    result.stallRecoveries = m_stallRecoveries;
    result.bestProgress = m_bestProgress;

    if (m_startedAt != std::chrono::steady_clock::time_point{}) {
        result.elapsedSeconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - m_startedAt
        ).count();
    }
    if (result.elapsedSeconds > 1e-9) {
        result.expansionsPerSecond = static_cast<double>(m_totalExpansions) / result.elapsedSeconds;
    }

    if (result.elapsedSeconds > 0.25 && m_bestProgress > m_rootProgress + 0.05 && m_bestProgress < 100.0) {
        const auto progressRate = (m_bestProgress - m_rootProgress) / result.elapsedSeconds;
        if (progressRate > 1e-9) {
            result.etaSeconds = std::max(0.0, (100.0 - m_bestProgress) / progressRate);
        }
    }

    std::size_t words = m_rootCanonical.words.size() + m_rootCanonical.temporalWords.size();
    for (auto const& entry : m_frontier) {
        words += entry.canonical.words.size() + entry.canonical.temporalWords.size();
    }
    for (auto const& entry : m_nextFrontier) {
        words += entry.canonical.words.size() + entry.canonical.temporalWords.size();
    }
    for (auto const& [_, bucket] : m_seenBuckets) {
        for (auto const& seen : bucket) words += seen.words.size();
    }
    for (auto const& [_, bucket] : m_temporalBuckets) {
        for (auto const& seen : bucket) words += seen.words.size();
    }

    result.estimatedMemoryBytes =
        words * sizeof(std::uint64_t)
        + m_meta.size() * sizeof(NodeMeta)
        + (m_frontier.size() + m_nextFrontier.size()) * sizeof(FrontierEntry)
        + m_policy.size() * sizeof(UniversalAction);

    return result;
}

} // namespace autobot::presolve
