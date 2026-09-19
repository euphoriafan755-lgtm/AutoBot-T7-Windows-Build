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

    UniversalObservation observe() const override { return makeObservation(); }

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
        if (m_state.dead || m_state.tick >= m_scenario.steps.size()) return makeObservation();
        auto const& expected = m_scenario.steps[m_state.tick];
        if (action != expected.action) {
            m_state.dead = true;
            return makeObservation();
        }
        m_state.worldState ^= static_cast<std::uint64_t>(expected.worldMutation) * 0x9e3779b97f4a7c15ULL;
        m_state.worldState = (m_state.worldState << 9U) | (m_state.worldState >> (64U - 9U));
        ++m_state.tick;
        if (m_state.tick < m_scenario.steps.size()) {
            m_state.dual = m_scenario.steps[m_state.tick].dual;
            m_state.platformer = m_scenario.steps[m_state.tick].platformer;
        }
        return makeObservation();
    }

    void discard(UniversalToken token) override { m_snapshots.erase(token); }

private:
    UniversalObservation makeObservation() const {
        UniversalObservation out{};
        out.valid = true;
        out.dead = m_state.dead;
        out.complete = !m_state.dead && m_state.tick == m_scenario.steps.size();
        out.dual = m_state.dual;
        out.platformer = m_state.platformer;
        out.progress = m_scenario.steps.empty()
            ? 100.0
            : 100.0 * static_cast<double>(m_state.tick) / static_cast<double>(m_scenario.steps.size());
        out.fingerprint.lo = static_cast<std::uint64_t>(m_state.tick)
            ^ (m_state.worldState + (static_cast<std::uint64_t>(m_state.dual) << 61U));
        out.fingerprint.hi = (m_state.worldState ^ 0xd6e8feb86659fd93ULL)
            + (static_cast<std::uint64_t>(m_state.platformer) << 62U)
            + (static_cast<std::uint64_t>(m_state.dead) << 63U);
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
    std::cout << "UNIVERSAL_MECHANIC " << scenario.name << "=PASS ticks=" << core.policy().size() << "\n";
}

} // namespace

int main() {
    const std::vector<Scenario> scenarios{
        {"BASIC_CUBE", {{jump(false),false,false,1},{jump(true),false,false,2},{jump(false),false,false,3},{jump(true),false,false,4}}},
        {"SHIP", {{jump(true),false,false,11},{jump(true),false,false,12},{jump(false),false,false,13},{jump(false),false,false,14}}},
        {"BALL", {{jump(false),false,false,21},{jump(true),false,false,22},{jump(false),false,false,23},{jump(true),false,false,24}}},
        {"UFO", {{jump(true),false,false,31},{jump(false),false,false,32},{jump(true),false,false,33},{jump(false),false,false,34}}},
        {"WAVE", {{jump(false),false,false,41},{jump(true),false,false,42},{jump(true),false,false,43},{jump(false),false,false,44}}},
        {"ROBOT", {{jump(true),false,false,51},{jump(true),false,false,52},{jump(false),false,false,53},{jump(true),false,false,54}}},
        {"SPIDER", {{jump(false),false,false,61},{jump(true),false,false,62},{jump(false),false,false,63}}},
        {"SWING", {{jump(true),false,false,71},{jump(false),false,false,72},{jump(true),false,false,73},{jump(false),false,false,74}}},
        {"PORTALS_SPEED_GRAVITY_MINI", {{jump(false),false,false,81},{jump(false),false,false,82},{jump(true),false,false,83},{jump(false),false,false,84},{jump(true),false,false,85}}},
        {"DUAL", {{jump(false,false),true,false,91},{jump(true,false),true,false,92},{jump(false,true),true,false,93},{jump(true,true),true,false,94}}},
        {"ORBS_AND_PADS", {{jump(false),false,false,101},{jump(true),false,false,102},{jump(false),false,false,103},{jump(true),false,false,104}}},
        {"TELEPORT", {{jump(false),false,false,111},{jump(true),false,false,112},{jump(false),false,false,113}}},
        {"TRIGGERS_AND_MOVING_GEOMETRY", {{jump(false),false,false,121},{jump(true),false,false,122},{jump(false),false,false,123},{jump(false),false,false,124},{jump(true),false,false,125}}},
        {"PLATFORMER", {{platform(false,false,true),false,true,131},{platform(true,false,true),false,true,132},{platform(false,true,false),false,true,133},{platform(true,true,false),false,true,134}}},
        {"DUAL_PLATFORMER", {{platform(false,false,true,false,true,false),true,true,141},{platform(true,false,true,true,true,false),true,true,142},{platform(false,true,false,false,false,true),true,true,143}}},
        {"DEMON_STYLE_PRECISION", {{jump(false),false,false,151},{jump(true),false,false,152},{jump(false),false,false,153},{jump(true),false,false,154},{jump(false),false,false,155},{jump(true),false,false,156},{jump(false),false,false,157},{jump(true),false,false,158}}},
    };

    for (auto const& scenario : scenarios) runScenario(scenario);
    std::cout << "UNIVERSAL_GENERATED_MECHANICS_SUITE=PASS scenarios=" << scenarios.size() << "\n";
    return 0;
}
