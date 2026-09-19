#include "autobot/presolve/UniversalSearchRuntimeLiveness.hpp"

#include <cassert>
#include <iostream>

using namespace autobot::presolve;

int main() {
    UniversalSearchRuntimeLiveness live;
    UniversalSearchStats initial{};
    initial.frontierSize = 1;
    initial.uniqueStates = 1;
    SearchVisibleState visible{100.0, 0.0, 0.0, 1, false};
    live.reset(initial, visible);

    auto stats = initial;
    stats.totalExpansions = 32;
    stats.frontierSize = 2;
    stats.uniqueStates = 3;
    auto r = live.observe(stats, visible, 0.2);
    assert(r.state == SearchLivenessState::Monitoring);

    stats.totalExpansions = 320;
    stats.currentDepth = 2;
    stats.frontierSize = 12;
    stats.uniqueStates = 40;
    stats.bestProgress = 1.25;
    r = live.observe(stats, visible, 2.1);
    assert(r.state == SearchLivenessState::Pass);

    // No work is recoverable: runtime should change strategy rather than die.
    live.reset(initial, visible);
    r = live.observe(initial, visible, 2.01);
    assert(r.state == SearchLivenessState::Stall);
    assert(r.reason.find("ZERO EXPANSIONS") != std::string::npos);

    live.reset(initial, visible);
    for (int i = 0; i < 119; ++i) {
        r = live.observe(initial, visible, 0.5);
        assert(r.state == SearchLivenessState::Monitoring);
    }
    r = live.observe(initial, visible, 0.5);
    assert(r.state == SearchLivenessState::Stall);
    assert(r.reason.find("NO EXPANSIONS") != std::string::npos);

    // Expanding forever without improving best progress is also a strategy stall.
    live.reset(initial, visible);
    stats = initial;
    for (int i = 1; i <= 300; ++i) {
        stats.totalExpansions = static_cast<std::size_t>(i);
        stats.frontierSize = 2 + static_cast<std::size_t>(i % 3);
        r = live.observe(stats, visible, 0.01 * i);
    }
    assert(r.state == SearchLivenessState::Stall);
    assert(r.reason.find("BEST-PROGRESS") != std::string::npos);

    // Visible state movement is not recoverable: this is a real invariant failure.
    live.reset(initial, visible);
    auto moved = visible;
    moved.playerX += 0.25;
    stats = initial;
    stats.totalExpansions = 1;
    r = live.observe(stats, moved, 0.1);
    assert(r.state == SearchLivenessState::Fail);
    assert(r.reason == "VISIBLE GAMEPLAY MOVED DURING SEARCH");

    std::cout << "UNIVERSAL_SEARCH_RUNTIME_LIVENESS_TEST=PASS\n";
    return 0;
}
