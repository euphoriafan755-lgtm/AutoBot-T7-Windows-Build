#include "autobot/presolve/UniversalSearchCore.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>
#include <optional>
#include <unordered_map>

using namespace autobot::presolve;

namespace {
struct State {
    int depth = 0;
    int hidden = -1;
    bool dead = false;
    bool complete = false;
};

class AliasOracle final : public IUniversalStateOracle {
public:
    UniversalObservation observe() const override { return obs(m_state); }
    std::optional<UniversalToken> capture() override {
        auto t = ++m_next;
        m_snapshots[t] = m_state;
        return t;
    }
    bool restore(UniversalToken t) override {
        auto it = m_snapshots.find(t);
        if (it == m_snapshots.end()) return false;
        m_state = it->second;
        return true;
    }
    UniversalObservation step(UniversalAction a) override {
        if (m_state.dead || m_state.complete) return obs(m_state);
        if (m_state.depth == 0) {
            m_state.depth = 1;
            // Both children intentionally share the same diagnostic hash.
            // false is explored first and is a dead-end one step later;
            // true is the only solvable branch.
            m_state.hidden = a.p1Hold ? 1 : 0;
        } else if (m_state.depth == 1) {
            m_state.depth = 2;
            if (m_state.hidden == 1 && a.p1Hold) m_state.complete = true;
            else m_state.dead = true;
        }
        return obs(m_state);
    }
    void discard(UniversalToken t) override { m_snapshots.erase(t); }
    std::optional<UniversalCanonicalState> canonicalState(UniversalToken t) const override {
        auto it = m_snapshots.find(t);
        if (it == m_snapshots.end()) return std::nullopt;
        UniversalCanonicalState c{};
        c.hash = {0xA11A5ULL, 0xC0111DEULL}; // deliberate hash collision
        c.words = {
            static_cast<std::uint64_t>(it->second.depth),
            static_cast<std::uint64_t>(it->second.hidden + 1),
            static_cast<std::uint64_t>(it->second.dead),
            static_cast<std::uint64_t>(it->second.complete),
        };
        c.completeRepresentation = true;
        return c;
    }
private:
    static UniversalObservation obs(State const& s) {
        UniversalObservation o{};
        o.valid = true;
        o.dead = s.dead;
        o.complete = s.complete;
        o.progress = s.complete ? 100.0 : s.depth * 50.0;
        o.fingerprint = {0xA11A5ULL, 0xC0111DEULL};
        return o;
    }
    State m_state{};
    UniversalToken m_next = 0;
    std::unordered_map<UniversalToken, State> m_snapshots;
};
}

int main() {
    AliasOracle oracle;
    UniversalSearchCore search;
    assert(search.begin(oracle));
    for (int i = 0; i < 32 && !search.ready() && search.stage() != UniversalSearchStage::Error; ++i) {
        search.work(oracle, 8);
    }
    const bool validPolicyExists = true;
    const bool searchFound = search.ready();
    const bool replayPassed = search.ready() && search.policy().size() == 2;
    std::cout << "ALIAS_COUNTEREXAMPLE validPolicyExists=" << validPolicyExists
              << " searchFound=" << searchFound
              << " replayPassed=" << replayPassed << "\n";
    assert(validPolicyExists && searchFound && replayPassed);
    std::cout << "SEARCH_STATE_ALIAS_TEST=PASS\n";
    return 0;
}
