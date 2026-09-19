#include "autobot/presolve/UniversalSearchCore.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>
#include <optional>
#include <unordered_map>

using namespace autobot::presolve;

namespace {

struct State {
    std::size_t tick = 0;
    bool dead = false;
};

class LongStraightOracle final : public IUniversalStateOracle {
public:
    static constexpr std::size_t kTicks = 2400;

    UniversalObservation observe() const override { return observation(m_state); }

    std::optional<UniversalToken> capture() override {
        const auto token = ++m_next;
        m_snapshots[token] = m_state;
        return token;
    }

    bool restore(UniversalToken token) override {
        auto it = m_snapshots.find(token);
        if (it == m_snapshots.end()) return false;
        m_state = it->second;
        return true;
    }

    UniversalObservation step(UniversalAction action) override {
        if (m_state.dead || m_state.tick >= kTicks) return observation(m_state);
        if (action.p1Hold || action.p1Left || action.p1Right
            || action.p2Hold || action.p2Left || action.p2Right) {
            m_state.dead = true;
        } else {
            ++m_state.tick;
        }
        return observation(m_state);
    }

    void discard(UniversalToken token) override { m_snapshots.erase(token); }

    std::optional<UniversalCanonicalState> canonicalState(UniversalToken token) const override {
        auto it = m_snapshots.find(token);
        if (it == m_snapshots.end()) return std::nullopt;

        UniversalCanonicalState state{};
        state.words = {
            static_cast<std::uint64_t>(it->second.tick),
            static_cast<std::uint64_t>(it->second.dead),
        };
        state.hash = {
            static_cast<std::uint64_t>(it->second.tick),
            0x4d4143524f534541ULL ^ static_cast<std::uint64_t>(it->second.dead),
        };
        state.completeRepresentation = true;
        return state;
    }

private:
    static UniversalObservation observation(State const& state) {
        UniversalObservation out{};
        out.valid = true;
        out.dead = state.dead;
        out.complete = !state.dead && state.tick >= kTicks;
        out.progress = 100.0 * static_cast<double>(state.tick) / static_cast<double>(kTicks);
        return out;
    }

    State m_state{};
    UniversalToken m_next = 0;
    std::unordered_map<UniversalToken, State> m_snapshots;
};

} // namespace

int main() {
    LongStraightOracle oracle;
    UniversalSearchCore search;
    assert(search.begin(oracle));

    std::size_t guard = 0;
    while (!search.ready()
        && search.stage() != UniversalSearchStage::Error
        && search.stage() != UniversalSearchStage::Exhausted
        && guard++ < 10000) {
        search.work(oracle, 16);
    }

    const auto stats = search.stats();
    assert(search.ready());
    assert(search.policy().size() == LongStraightOracle::kTicks);
    assert(stats.bestProgress >= 100.0);

    // A one-tick breadth-first search has exponential width here. The macro
    // scheduler must close the same 2400-tick policy with a small number of
    // decision-state expansions while retaining 1-tick precision as fallback.
    assert(stats.totalExpansions < 1000);
    assert(stats.totalEngineSteps >= LongStraightOracle::kTicks);

    std::cout
        << "UNIVERSAL_MACRO_SEARCH_PERF=PASS expansions=" << stats.totalExpansions
        << " engineSteps=" << stats.totalEngineSteps
        << " policyTicks=" << search.policy().size()
        << "\n";
    return 0;
}
