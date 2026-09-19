#include "autobot/presolve/UniversalSearchRuntimeLiveness.hpp"

#include <cmath>

namespace autobot::presolve {

bool UniversalSearchRuntimeLiveness::visibleFrozen(SearchVisibleState const& a, SearchVisibleState const& b) {
    return std::abs(a.playerX - b.playerX) <= 0.0001
        && std::abs(a.progress - b.progress) <= 0.0001
        && std::abs(a.levelTime - b.levelTime) <= 0.0001
        && a.attempts == b.attempts
        && !b.dead;
}

void UniversalSearchRuntimeLiveness::reset(UniversalSearchStats const& initial, SearchVisibleState const& visible) {
    m_initial = initial;
    m_last = initial;
    m_visible = visible;
    m_result = {};
}

SearchLivenessResult UniversalSearchRuntimeLiveness::observe(
    UniversalSearchStats const& stats,
    SearchVisibleState const& visible,
    double elapsedSeconds
) {
    if (m_result.state == SearchLivenessState::Fail) return m_result;

    if (!visibleFrozen(m_visible, visible)) {
        m_result.state = SearchLivenessState::Fail;
        m_result.reason = "VISIBLE GAMEPLAY MOVED DURING SEARCH";
        return m_result;
    }

    // Expansion liveness applies to the search phase only. Once the core enters
    // full-policy replay, totalExpansions intentionally stops increasing.
    if (stats.stage == UniversalSearchStage::Replaying || stats.stage == UniversalSearchStage::Ready) {
        m_result.stagnantFrames = 0;
        m_last = stats;
        return m_result;
    }

    if (stats.totalExpansions > m_last.totalExpansions) {
        m_result.stagnantFrames = 0;
        m_result.sawExpansion = true;
    } else {
        ++m_result.stagnantFrames;
    }
    m_result.sawDepthAdvance = m_result.sawDepthAdvance || stats.currentDepth > m_initial.currentDepth;
    m_result.sawFrontierChange = m_result.sawFrontierChange || stats.frontierSize != m_initial.frontierSize;
    m_result.sawBestAdvance = m_result.sawBestAdvance || stats.bestProgress > m_initial.bestProgress + 1e-9;

    if (m_result.stagnantFrames >= 120) {
        m_result.state = SearchLivenessState::Fail;
        m_result.reason = "SEARCH DRIVER STALLED";
    } else if (elapsedSeconds >= 2.0 && stats.totalExpansions == 0) {
        m_result.state = SearchLivenessState::Fail;
        m_result.reason = "SEARCH DRIVER STALLED: ZERO EXPANSIONS AFTER 2S";
    } else if (elapsedSeconds >= 2.0
        && m_result.sawExpansion
        && (m_result.sawDepthAdvance || m_result.sawFrontierChange)) {
        m_result.state = SearchLivenessState::Pass;
        m_result.reason = "SEARCH DRIVER LIVE AND VISIBLE GAMEPLAY FROZEN";
    }

    m_last = stats;
    return m_result;
}

} // namespace autobot::presolve
