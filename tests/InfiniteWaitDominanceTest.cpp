#include "autobot/presolve/UniversalSearchCore.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>
#include <optional>
#include <unordered_map>

using namespace autobot::presolve;

namespace {
struct State { int tick = 0; int timer = 0; bool dead = false; bool complete = false; };

class WaitOracle final : public IUniversalStateOracle {
public:
    explicit WaitOracle(bool pendingTimer) { m_state.timer = pendingTimer ? 3 : 0; }
    UniversalObservation observe() const override { return obs(m_state); }
    std::optional<UniversalToken> capture() override { auto t=++m_next; m_snapshots[t]=m_state; return t; }
    bool restore(UniversalToken t) override { auto it=m_snapshots.find(t); if(it==m_snapshots.end()) return false; m_state=it->second; return true; }
    UniversalObservation step(UniversalAction a) override {
        if (m_state.dead || m_state.complete) return obs(m_state);
        if (a.p1Hold) {
            if (m_state.timer == 0) m_state.complete = true;
            else m_state.dead = true;
        } else {
            ++m_state.tick;
            if (m_state.timer > 0) --m_state.timer;
        }
        return obs(m_state);
    }
    void discard(UniversalToken t) override { m_snapshots.erase(t); }
    std::optional<UniversalCanonicalState> canonicalState(UniversalToken t) const override {
        auto it=m_snapshots.find(t); if(it==m_snapshots.end()) return std::nullopt;
        auto const& s=it->second;
        UniversalCanonicalState c{};
        c.hash={static_cast<std::uint64_t>(s.timer),0x1D1EULL};
        c.words={static_cast<std::uint64_t>(s.tick),static_cast<std::uint64_t>(s.timer),static_cast<std::uint64_t>(s.dead),static_cast<std::uint64_t>(s.complete)};
        c.completeRepresentation=true;
        // Absolute tick is irrelevant only when there is no pending timer.
        if (s.timer == 0 && !s.complete && !s.dead) {
            c.temporalDominanceEligible=true;
            c.temporalHash={0x515549455343454EULL,0x54ULL};
            c.temporalWords={0,0};
        }
        return c;
    }
private:
    static UniversalObservation obs(State const& s) {
        UniversalObservation o{}; o.valid=true; o.dead=s.dead; o.complete=s.complete; o.progress=s.complete?100.0:0.0; return o;
    }
    State m_state{}; UniversalToken m_next=0; std::unordered_map<UniversalToken,State> m_snapshots;
};
}

int main() {
    {
        WaitOracle oracle(false);
        UniversalSearchCore search;
        assert(search.begin(oracle));
        for (int i=0;i<8 && !search.ready() && search.stage()!=UniversalSearchStage::Exhausted;++i) search.work(oracle,8);
        // PRESS is valid immediately, so this case also proves the quiescent
        // WAIT self-loop is dominated without blocking another action.
        assert(search.ready());
        assert(search.stats().temporalDominanceHits >= 1);
    }
    {
        WaitOracle oracle(true);
        UniversalSearchCore search;
        assert(search.begin(oracle));
        for (int i=0;i<64 && !search.ready() && search.stage()!=UniversalSearchStage::Error;++i) search.work(oracle,16);
        // The timer makes time relevant for three ticks; dominance must not
        // collapse those states. Once it expires, PRESS completes.
        assert(search.ready());
        assert(search.policy().size() == 4);
    }
    std::cout << "INFINITE_WAIT_DOMINANCE_TEST=PASS\n";
    return 0;
}
