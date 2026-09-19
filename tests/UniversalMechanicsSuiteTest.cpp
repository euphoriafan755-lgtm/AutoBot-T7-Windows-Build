#include "autobot/presolve/UniversalSearchCore.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>
#include <optional>
#include <string_view>
#include <unordered_map>
#include <vector>

using namespace autobot::presolve;

namespace {

struct SpecStep {
    UniversalAction action{};
    bool dual = false;
    bool platformer = false;
    std::uint32_t worldMutation = 0;
};

struct Scenario {
    std::string_view name;
    std::vector<SpecStep> steps;
};

struct State {
    std::size_t tick = 0;
    bool dead = false;
    bool dual = false;
    bool platformer = false;
    std::uint64_t worldState = 0x123456789abcdef0ULL;
};

class ScenarioOracle final : public IUniversalStateOracle {
public:
    explicit ScenarioOracle(Scenario scenario) : m_scenario(std::move(scenario)) {
        if (!m_scenario.steps.empty()) {
            m_state.dual = m_scenario.steps[0].dual;
            m_state.platformer = m_scenario.steps[0].platformer;
        }
    }

    UniversalObservation observe() const override { return makeObservation(m_state); }

    std::optional<UniversalToken> capture() override {
        const auto token = ++m_nextToken;
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
        if (m_state.dead || m_state.tick >= m_scenario.steps.size()) return makeObservation(m_state);
        auto const& expected = m_scenario.steps[m_state.tick];
        if (action != expected.action) {
            m_state.dead = true;
            return makeObservation(m_state);
        }
        m_state.worldState ^= static_cast<std::uint64_t>(expected.worldMutation) * 0x9e3779b97f4a7c15ULL;
        m_state.worldState = (m_state.worldState << 9U) | (m_state.worldState >> (64U - 9U));
        ++m_state.tick;
        if (m_state.tick < m_scenario.steps.size()) {
            m_state.dual = m_scenario.steps[m_state.tick].dual;
            m_state.platformer = m_scenario.steps[m_state.tick].platformer;
        }
        return makeObservation(m_state);
    }

    void discard(UniversalToken token) override { m_snapshots.erase(token); }

    std::optional<UniversalCanonicalState> canonicalState(UniversalToken token) const override {
        auto const it = m_snapshots.find(token);
        if (it == m_snapshots.end()) return std::nullopt;
        auto const& s = it->second;
        UniversalCanonicalState out{};
        out.hash = makeObservation(s).fingerprint;
        out.words = {
            static_cast<std::uint64_t>(s.tick),
            static_cast<std::uint64_t>(s.dead),
            static_cast<std::uint64_t>(s.dual),
            static_cast<std::uint64_t>(s.platformer),
            s.worldState,
        };
        out.completeRepresentation = true;
        return out;
    }

private:
    UniversalObservation makeObservation(State const& state) const {
        UniversalObservation out{};
        out.valid = true;
        out.dead = state.dead;
        out.complete = !state.dead && state.tick == m_scenario.steps.size();
        out.dual = state.dual;
        out.platformer = state.platformer;
        out.progress = m_scenario.steps.empty()
            ? 100.0
            : 100.0 * static_cast<double>(state.tick) / static_cast<double>(m_scenario.steps.size());
        out.fingerprint.lo = static_cast<std::uint64_t>(state.tick)
            ^ (state.worldState + (static_cast<std::uint64_t>(state.dual) << 61U));
        out.fingerprint.hi = (state.worldState ^ 0xd6e8feb86659fd93ULL)
            + (static_cast<std::uint64_t>(state.platformer) << 62U)
            + (static_cast<std::uint64_t>(state.dead) << 63U);
        return out;
    }

    Scenario m_scenario;
    State m_state{};
    UniversalToken m_nextToken = 0;
    std::unordered_map<UniversalToken, State> m_snapshots;
};

UniversalAction jump(bool p1, bool p2 = false) {
    UniversalAction a{};
    a.p1Hold = p1;
    a.p2Hold = p2;
    return a;
}

UniversalAction platform(bool jumpDown, bool left, bool right, bool p2Jump = false, bool p2Left = false, bool p2Right = false) {
    UniversalAction a{};
    a.p1Hold = jumpDown;
    a.p1Left = left;
    a.p1Right = right;
    a.p2Hold = p2Jump;
    a.p2Left = p2Left;
    a.p2Right = p2Right;
    return a;
}

void runScenario(Scenario const& scenario) {
    ScenarioOracle oracle(scenario);
    UniversalSearchCore core;
    assert(core.begin(oracle));
    std::size_t watchdog = 0;
    while (!core.ready() && core.stage() != UniversalSearchStage::Error
           && core.stage() != UniversalSearchStage::Exhausted && watchdog++ < 200000) {
        core.work(oracle, 128);
    }
    assert(core.ready());
    assert(core.policy().size() == scenario.steps.size());
    assert(core.stats().bestProgress >= 100.0);
    std::cout << "SEARCH_SCENARIO " << scenario.name << "=PASS ticks=" << core.policy().size() << "\n";
}

} // namespace

int main() {
    // Search-core scenarios only. These labels are not evidence that the GD
    // runtime mechanic itself has been exercised.
    const std::vector<Scenario> scenarios{
        {"BASIC_BINARY", {{jump(false),false,false,1},{jump(true),false,false,2},{jump(false),false,false,3},{jump(true),false,false,4}}},
        {"HOLD_RUN_A", {{jump(true),false,false,11},{jump(true),false,false,12},{jump(false),false,false,13},{jump(false),false,false,14}}},
        {"HOLD_RUN_B", {{jump(false),false,false,21},{jump(true),false,false,22},{jump(false),false,false,23},{jump(true),false,false,24}}},
        {"PULSE_A", {{jump(true),false,false,31},{jump(false),false,false,32},{jump(true),false,false,33},{jump(false),false,false,34}}},
        {"PULSE_B", {{jump(false),false,false,41},{jump(true),false,false,42},{jump(true),false,false,43},{jump(false),false,false,44}}},
        {"DUAL_BINARY", {{jump(false,false),true,false,91},{jump(true,false),true,false,92},{jump(false,true),true,false,93},{jump(true,true),true,false,94}}},
        {"PLATFORMER_ACTION_SET", {{platform(false,false,true),false,true,131},{platform(true,false,true),false,true,132},{platform(false,true,false),false,true,133},{platform(true,true,false),false,true,134}}},
        {"DUAL_PLATFORMER_ACTION_SET", {{platform(false,false,true,false,true,false),true,true,141},{platform(true,false,true,true,true,false),true,true,142},{platform(false,true,false,false,false,true),true,true,143}}},
        {"PRECISION_SEQUENCE", {{jump(false),false,false,151},{jump(true),false,false,152},{jump(false),false,false,153},{jump(true),false,false,154},{jump(false),false,false,155},{jump(true),false,false,156},{jump(false),false,false,157},{jump(true),false,false,158}}},
    };

    for (auto const& scenario : scenarios) runScenario(scenario);
    std::cout << "UNIVERSAL_SEARCH_SCENARIO_SUITE=PASS scenarios=" << scenarios.size() << "\n";
    return 0;
}
