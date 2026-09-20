#include "autobot/presolve/UniversalRuntimeSession.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>
#include <optional>
#include <unordered_map>

using namespace autobot;

namespace {

struct State {
    std::size_t tick = 0;
};

class Oracle final : public presolve::IUniversalStateOracle {
public:
    presolve::UniversalObservation observe() const override {
        presolve::UniversalObservation out{};
        out.valid = true;
        out.progress = static_cast<double>(m_state.tick);
        return out;
    }

    std::optional<presolve::UniversalToken> capture() override {
        const auto token = ++m_next;
        m_states[token] = m_state;
        return token;
    }

    bool restore(presolve::UniversalToken token) override {
        auto it = m_states.find(token);
        if (it == m_states.end()) return false;
        m_state = it->second;
        return true;
    }

    presolve::UniversalObservation step(presolve::UniversalAction) override {
        ++m_state.tick;
        return observe();
    }

    void discard(presolve::UniversalToken token) override {
        m_states.erase(token);
    }

    std::optional<presolve::UniversalCanonicalState> canonicalState(
        presolve::UniversalToken token
    ) const override {
        auto it = m_states.find(token);
        if (it == m_states.end()) return std::nullopt;
        presolve::UniversalCanonicalState out{};
        out.words = {static_cast<std::uint64_t>(it->second.tick)};
        out.hash = {static_cast<std::uint64_t>(it->second.tick), 0x524f4c4c494e47ULL};
        out.completeRepresentation = true;
        return out;
    }

private:
    State m_state{};
    presolve::UniversalToken m_next = 0;
    std::unordered_map<presolve::UniversalToken, State> m_states;
};

core::GameSnapshot frame(std::uint64_t id, double t, double x) {
    core::GameSnapshot s{};
    s.valid = true;
    s.solverSampleID = id;
    s.levelTime = t;
    s.levelProgress = static_cast<float>(x * 0.1);
    s.player.x = x;
    s.player.y = 100.0;
    s.player.velocityX = 5.0;
    s.player.velocityY = 0.0;
    s.player.mode = core::GameMode::Cube;
    s.player.objectBoundsWidth = 30.0;
    s.player.objectBoundsHeight = 30.0;
    return s;
}

} // namespace

int main() {
    Oracle oracle;
    presolve::UniversalSearchCore search;
    assert(search.begin(oracle));

    const auto root = frame(0, 0.0, 0.0);
    auto previous = root;
    const auto resetSerial = search.stats().resetSerial;

    std::size_t reroots = 0;
    std::size_t previousExpansions = 0;

    for (std::size_t i = 1; i <= 10; ++i) {
        const auto current = frame(i, static_cast<double>(i) / 60.0, static_cast<double>(i) * 5.0);
        const auto assessment = presolve::assessLiveRoot(
            root, previous, current, i, 0
        );
        if (assessment.reroot) ++reroots;
        assert(!assessment.reroot);

        const auto stats = search.work(oracle, 2);
        assert(stats.totalExpansions > previousExpansions);
        assert(stats.resetSerial == resetSerial);
        previousExpansions = stats.totalExpansions;
        previous = current;
    }

    const auto persistentStats = search.stats();
    assert(reroots == 0);
    assert(persistentStats.totalExpansions >= 10);
    assert(persistentStats.currentDepth > 0);

    auto current = previous;
    bool windowReroot = false;
    for (std::size_t i = 11; i <= 24; ++i) {
        current = frame(i, static_cast<double>(i) / 60.0, static_cast<double>(i) * 5.0);
        const auto assessment = presolve::assessLiveRoot(
            root, previous, current, i, 0
        );
        if (i < 24) assert(!assessment.reroot);
        if (i == 24) {
            assert(assessment.reroot);
            windowReroot = true;
        }
        previous = current;
    }
    assert(windowReroot);

    std::cout
        << "ROLLING_SEARCH_PERSISTENCE_TEST=PASS "
        << "real_frames=10 root_survives=YES reroots=0 "
        << "expansions=" << persistentStats.totalExpansions
        << " depth=" << persistentStats.currentDepth
        << " periodic_reroot_frame=24\n";
    return 0;
}
