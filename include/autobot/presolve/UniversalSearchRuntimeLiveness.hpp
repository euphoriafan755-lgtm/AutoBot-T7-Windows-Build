#pragma once

#include "autobot/presolve/UniversalSearchCore.hpp"

#include <cstddef>
#include <string>

namespace autobot::presolve {

struct SearchVisibleState {
    double playerX = 0.0;
    double progress = 0.0;
    double levelTime = 0.0;
    int attempts = 0;
    bool dead = false;
};

enum class SearchLivenessState {
    Monitoring,
    Pass,
    Stall,
    Fail,
};

struct SearchLivenessResult {
    SearchLivenessState state = SearchLivenessState::Monitoring;
    std::string reason;
    std::size_t stagnantFrames = 0;
    std::size_t stagnantBestFrames = 0;
    bool sawExpansion = false;
    bool sawDepthAdvance = false;
    bool sawFrontierChange = false;
    bool sawBestAdvance = false;
};

class UniversalSearchRuntimeLiveness final {
public:
    void reset(UniversalSearchStats const& initial, SearchVisibleState const& visible);
    SearchLivenessResult observe(
        UniversalSearchStats const& stats,
        SearchVisibleState const& visible,
        double elapsedSeconds
    );

    [[nodiscard]] SearchLivenessResult const& result() const { return m_result; }

private:
    static bool visibleFrozen(SearchVisibleState const& a, SearchVisibleState const& b);

    UniversalSearchStats m_initial{};
    UniversalSearchStats m_last{};
    SearchVisibleState m_visible{};
    SearchLivenessResult m_result{};
};

} // namespace autobot::presolve
