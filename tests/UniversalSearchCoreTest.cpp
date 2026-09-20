#include "autobot/presolve/UniversalSearchCore.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>
#include <optional>
#include <unordered_map>
#include <vector>

using namespace autobot::presolve;

namespace {

struct State {
    int tick = 0;
    int lane = 0;
    bool dual = false;
    bool dead = false;
};

class GeneratedMechanicsOracle final : public IUniversalStateOracle {
public:
    UniversalObservation observe() const override { return observation(m_state); }

    std::optional<UniversalToken> capture() override {
        const auto token = ++m_next;
        m_snapshots[token] = m_state;
        return token;
    }

    bool restore(UniversalToken token) override {
        auto const it = m_snapshots.find(token);
        if (it == m_snapshots.end()) return false;
        m_state = it->second;
        return true;
    }

    UniversalObservation step(UniversalAction action) override {
        // Generated sequence exercises single input, dual input, mode-like
        // transition points, a moving-obstacle phase, and a final precision gate.
        ++m_state.tick;
        if (m_state.tick == 3) m_state.dual = true;
        if (m_state.tick == 7) m_state.dual = false;

        const bool expectedP1 = (m_state.tick == 2 || m_state.tick == 5 || m_state.tick == 6 || m_state.tick == 9);
        const bool expectedP2 = m_state.dual && (m_state.tick == 4 || m_state.tick == 6);
        if (action.p1Hold != expectedP1 || action.p1Left || action.p1Right
            || (m_state.dual && (action.p2Hold != expectedP2 || action.p2Left || action.p2Right))) {
            m_state.dead = true;
        } else {
            m_state.lane += action.p1Hold ? 2 : 1;
            if (action.p2Hold) ++m_state.lane;
        }
        return observation(m_state);
    }

    void discard(UniversalToken token) override { m_snapshots.erase(token); }

    std::optional<UniversalCanonicalState> canonicalState(UniversalToken token) const override {
        auto const it = m_snapshots.find(token);
        if (it == m_snapshots.end()) return std::nullopt;
        auto const& s = it->second;
        UniversalCanonicalState out{};
        out.hash = observation(s).fingerprint;
        out.words = {
            static_cast<std::uint64_t>(s.tick),
            static_cast<std::uint64_t>(s.lane),
            static_cast<std::uint64_t>(s.dual),
            static_cast<std::uint64_t>(s.dead),
        };
        out.completeRepresentation = true;
        return out;
    }

private:
    static UniversalObservation observation(State const& state) {
        UniversalObservation result{};
        result.valid = true;
        result.dead = state.dead;
        result.complete = !state.dead && state.tick >= 10;
        result.dual = state.dual;
        result.platformer = false;
        result.progress = static_cast<double>(state.tick) * 10.0;
        result.fingerprint.lo = static_cast<std::uint64_t>(state.tick)
            | (static_cast<std::uint64_t>(state.lane) << 16U)
            | (static_cast<std::uint64_t>(state.dual) << 48U)
            | (static_cast<std::uint64_t>(state.dead) << 49U);
        result.fingerprint.hi = 0x554e495645525341ULL ^ result.fingerprint.lo;
        return result;
    }

    State m_state{};
    UniversalToken m_next = 0;
    std::unordered_map<UniversalToken, State> m_snapshots;
};


class SurvivalOracle final : public IUniversalStateOracle {
public:
    UniversalObservation observe() const override {
        UniversalObservation out{};
        out.valid = true;
        out.dead = m_state.dead;
        out.complete = false;
        out.progress = 42.0;
        return out;
    }

    std::optional<UniversalToken> capture() override {
        const auto token = ++m_next;
        m_snapshots[token] = m_state;
        return token;
    }

    bool restore(UniversalToken token) override {
        auto const it = m_snapshots.find(token);
        if (it == m_snapshots.end()) return false;
        m_state = it->second;
        return true;
    }

    UniversalObservation step(UniversalAction action) override {
        if (!m_state.routeChosen) {
            m_state.routeChosen = true;
            m_state.pressRoute = action.p1Hold;
        }

        ++m_state.tick;
        const std::size_t survivalLimit = m_state.pressRoute ? 100 : 10;
        if (m_state.tick >= survivalLimit) m_state.dead = true;
        return observe();
    }

    void discard(UniversalToken token) override {
        m_snapshots.erase(token);
    }

    std::optional<UniversalCanonicalState> canonicalState(
        UniversalToken token
    ) const override {
        auto const it = m_snapshots.find(token);
        if (it == m_snapshots.end()) return std::nullopt;
        auto const& s = it->second;

        UniversalCanonicalState out{};
        out.words = {
            static_cast<std::uint64_t>(s.tick),
            static_cast<std::uint64_t>(s.routeChosen),
            static_cast<std::uint64_t>(s.pressRoute),
            static_cast<std::uint64_t>(s.dead),
        };
        out.hash = {
            static_cast<std::uint64_t>(s.tick)
                ^ (static_cast<std::uint64_t>(s.pressRoute) << 32U),
            0x535552564956414cULL
        };
        out.completeRepresentation = true;
        return out;
    }

private:
    struct SurvivalState {
        std::size_t tick = 0;
        bool routeChosen = false;
        bool pressRoute = false;
        bool dead = false;
    };

    SurvivalState m_state{};
    UniversalToken m_next = 0;
    std::unordered_map<UniversalToken, SurvivalState> m_snapshots;
};

} // namespace

int main() {
    GeneratedMechanicsOracle oracle;
    UniversalSearchCore search;
    assert(search.begin(oracle));

    std::size_t guard = 0;
    while (!search.ready() && search.stage() != UniversalSearchStage::Error
           && search.stage() != UniversalSearchStage::Exhausted && guard++ < 10000) {
        search.work(oracle, 64);
    }

    assert(search.ready());
    assert(search.policy().size() == 10);
    assert(search.stats().bestProgress >= 100.0);
    std::cout << "UNIVERSAL_SEARCH_CORE=PASS\n";
    std::cout << "GENERATED_UNSEEN_POLICY_REPLAY=PASS ticks=" << search.policy().size() << "\n";

    // Equal forward progress must not let a shallow arbitrary branch replace a
    // partial solution that has survived substantially farther into the future.
    assert(betterPartialSolution(false, 42.0, 100, false, 42.0, 10));
    assert(!betterPartialSolution(false, 42.0, 10, false, 42.0, 100));

    SurvivalOracle survivalOracle;
    UniversalSearchCore survivalSearch;
    assert(survivalSearch.begin(survivalOracle));
    survivalSearch.work(survivalOracle, 6);
    const auto prefix = survivalSearch.bestPrefix();
    assert(!prefix.empty());
    assert(prefix.front().p1Hold);

    std::cout
        << "BEST_PREFIX_SURVIVAL_TEST=PASS "
        << "no_press_survival=10 press_survival=100 "
        << "equal_progress=YES best_prefix=PRESS\n";
    return 0;
}
