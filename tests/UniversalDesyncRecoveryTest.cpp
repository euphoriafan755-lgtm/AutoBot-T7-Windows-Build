#include "autobot/presolve/UniversalSearchCore.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>
#include <optional>
#include <unordered_map>

using namespace autobot::presolve;

namespace {
struct State { int phase=0; int route=0; bool dead=false; bool complete=false; };
class RecoveryOracle final : public IUniversalStateOracle {
public:
    UniversalObservation observe() const override { return obs(m_state); }
    std::optional<UniversalToken> capture() override { auto t=++m_next; m_snapshots[t]=m_state; return t; }
    bool restore(UniversalToken t) override { auto it=m_snapshots.find(t); if(it==m_snapshots.end()) return false; m_state=it->second; return true; }
    UniversalObservation step(UniversalAction a) override {
        if(m_state.dead||m_state.complete) return obs(m_state);
        if(m_state.phase==0) { m_state.phase=1; m_state.route=a.p1Hold?1:0; }
        else if(m_state.phase==1) {
            m_state.phase=2;
            const bool need = m_state.route==0 ? false : true;
            if(a.p1Hold!=need) m_state.dead=true;
        } else if(m_state.phase==2) {
            const bool need = m_state.route==0 ? true : false;
            if(a.p1Hold==need) m_state.complete=true; else m_state.dead=true;
        }
        return obs(m_state);
    }
    void injectAliveDesync() { m_state.phase=1; m_state.route=1; m_state.dead=false; m_state.complete=false; }
    void discard(UniversalToken t) override { m_snapshots.erase(t); }
    std::optional<UniversalCanonicalState> canonicalState(UniversalToken t) const override {
        auto it=m_snapshots.find(t); if(it==m_snapshots.end()) return std::nullopt;
        UniversalCanonicalState c{}; c.hash={static_cast<std::uint64_t>(it->second.phase),static_cast<std::uint64_t>(it->second.route)};
        c.words={static_cast<std::uint64_t>(it->second.phase),static_cast<std::uint64_t>(it->second.route),static_cast<std::uint64_t>(it->second.dead),static_cast<std::uint64_t>(it->second.complete)};
        c.completeRepresentation=true; return c;
    }
private:
    static UniversalObservation obs(State const& s){UniversalObservation o{};o.valid=true;o.dead=s.dead;o.complete=s.complete;o.progress=s.complete?100.0:s.phase*33.0;return o;}
    State m_state{}; UniversalToken m_next=0; std::unordered_map<UniversalToken,State> m_snapshots;
};

bool solveFromCurrent(RecoveryOracle& oracle, std::vector<UniversalAction>& out) {
    UniversalSearchCore search;
    if(!search.begin(oracle)) return false;
    for(int i=0;i<64 && !search.ready() && search.stage()!=UniversalSearchStage::Error && search.stage()!=UniversalSearchStage::Exhausted;++i) search.work(oracle,16);
    if(!search.ready()) return false;
    out=search.policy();
    return true;
}
}

int main(){
    RecoveryOracle oracle;
    std::vector<UniversalAction> original;
    assert(solveFromCurrent(oracle, original));
    // Runtime drift to another alive state whose correct suffix differs.
    oracle.injectAliveDesync();
    std::vector<UniversalAction> recovery;
    assert(solveFromCurrent(oracle, recovery));
    assert(!recovery.empty());
    // UniversalSearchCore replay already verifies the recovery policy before
    // ready() becomes true.
    std::cout << "UNIVERSAL_DESYNC_RECOVERY_TEST=PASS replannedTicks=" << recovery.size() << "\n";
    return 0;
}
