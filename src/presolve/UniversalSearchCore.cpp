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
        if (m_rootToken != kInvalidUniversalToken) oracle->discard(m_rootToken);
    } else {
        m_frontier.clear();
        m_nextFrontier.clear();
    }
    m_stage = UniversalSearchStage::Idle;
    m_rootToken = kInvalidUniversalToken;
    m_rootObservation = {};
    m_meta.clear();
    m_seenDepth.clear();
    m_policy.clear();
    m_replayCursor = 0;
    m_totalExpansions = 0;
    m_currentDepth = 0;
    m_bestProgress = 0.0;
}

bool UniversalSearchCore::begin(IUniversalStateOracle& oracle) {
    reset(&oracle);
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

    m_rootToken = *token;
    m_rootObservation = root;
    m_meta.push_back(NodeMeta{});
    m_seenDepth.emplace(root.fingerprint, 0);
    m_frontier.push_back({m_rootToken, 0, root});
    m_bestProgress = root.progress;
    m_stage = root.complete ? UniversalSearchStage::Ready : UniversalSearchStage::Searching;
    return true;
}

void UniversalSearchCore::reconstructPolicy(std::size_t metaIndex) {
    m_policy.clear();
    while (metaIndex != 0 && metaIndex < m_meta.size()) {
        auto const& node = m_meta[metaIndex];
        m_policy.push_back(node.action);
        metaIndex = node.parent;
    }
    std::reverse(m_policy.begin(), m_policy.end());
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
    m_replayCursor = 0;
    m_stage = UniversalSearchStage::Replaying;
    return true;
}

UniversalSearchStats UniversalSearchCore::replayWork(IUniversalStateOracle& oracle, std::size_t budget) {
    std::size_t done = 0;
    while (done < budget && m_replayCursor < m_policy.size()) {
        const auto observation = oracle.step(m_policy[m_replayCursor]);
        ++m_replayCursor;
        ++done;
        if (!observation.valid || observation.dead) {
            m_stage = UniversalSearchStage::Error;
            break;
        }
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
            std::stable_sort(
                m_nextFrontier.begin(),
                m_nextFrontier.end(),
                [](FrontierEntry const& a, FrontierEntry const& b) {
                    return a.observation.progress > b.observation.progress;
                }
            );
            m_frontier.swap(m_nextFrontier);
            ++m_currentDepth;
        }

        auto entry = m_frontier.front();
        m_frontier.pop_front();
        if (!oracle.restore(entry.token)) {
            if (entry.token != m_rootToken) oracle.discard(entry.token);
            m_stage = UniversalSearchStage::Error;
            break;
        }

        const auto actions = actionsFor(entry.observation.dual, entry.observation.platformer);
        for (auto const action : actions) {
            if (!oracle.restore(entry.token)) {
                m_stage = UniversalSearchStage::Error;
                break;
            }
            const auto observation = oracle.step(action);
            ++m_totalExpansions;
            ++expandedThisCall;
            if (!observation.valid || observation.dead) {
                continue;
            }

            m_bestProgress = std::max(m_bestProgress, observation.progress);
            const auto childDepth = m_meta[entry.metaIndex].depth + 1;
            auto seen = m_seenDepth.find(observation.fingerprint);
            if (seen != m_seenDepth.end() && seen->second <= childDepth) {
                continue;
            }
            m_seenDepth[observation.fingerprint] = childDepth;

            const auto childMeta = m_meta.size();
            m_meta.push_back({entry.metaIndex, action, childDepth});
            if (observation.complete) {
                if (entry.token != m_rootToken) oracle.discard(entry.token);
                if (!beginReplay(oracle, childMeta)) break;
                return replayWork(oracle, std::numeric_limits<std::size_t>::max());
            }

            auto token = oracle.capture();
            if (!token || *token == kInvalidUniversalToken) {
                m_stage = UniversalSearchStage::Error;
                break;
            }
            m_nextFrontier.push_back({*token, childMeta, observation});
        }

        if (entry.token != m_rootToken) oracle.discard(entry.token);
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
    result.frontierSize = m_frontier.size() + m_nextFrontier.size();
    result.currentDepth = m_currentDepth;
    result.uniqueStates = m_seenDepth.size();
    result.replayCursor = m_replayCursor;
    result.bestProgress = m_bestProgress;
    return result;
}

} // namespace autobot::presolve
