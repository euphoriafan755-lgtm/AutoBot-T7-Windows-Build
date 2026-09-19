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

void UniversalSearchRuntimeLiveness::reset(
    UniversalSearchStats const& initial,
    SearchVisibleState const& visible
) {
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

    if (stats.stage == UniversalSearchStage::Replaying || stats.stage == UniversalSearchStage::Ready) {
        m_result.stagnantFrames = 0;
        m_result.stagnantBestFrames = 0;
        m_last = stats;
        return m_result;
    }

    const bool expanded = stats.totalExpansions > m_last.totalExpansions;
    const bool improved = stats.bestProgress > m_last.bestProgress + 1e-9;

    if (expanded) {
        m_result.stagnantFrames = 0;
        m_result.sawExpansion = true;
    } else {
        ++m_result.stagnantFrames;
    }

    if (improved) {
        m_result.stagnantBestFrames = 0;
        m_result.sawBestAdvance = true;
    } else if (expanded) {
        ++m_result.stagnantBestFrames;
    }

    m_result.sawDepthAdvance = m_result.sawDepthAdvance
        || stats.currentDepth > m_initial.currentDepth;
    m_result.sawFrontierChange = m_result.sawFrontierChange
        || stats.frontierSize != m_initial.frontierSize;

    if (elapsedSeconds >= 2.0 && stats.totalExpansions == 0) {
        m_result.state = SearchLivenessState::Stall;
        m_result.reason = "ZERO EXPANSIONS AFTER 2S";
    } else if (m_result.stagnantFrames >= 120) {
        m_result.state = SearchLivenessState::Stall;
        m_result.reason = "NO EXPANSIONS FOR 120 SEARCH SLICES";
    } else if (m_result.stagnantBestFrames >= 300 && stats.totalExpansions > m_initial.totalExpansions) {
        m_result.state = SearchLivenessState::Stall;
        m_result.reason = "NO BEST-PROGRESS IMPROVEMENT FOR 300 ACTIVE SLICES";
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
