#include "autobot/presolve/UniversalRuntimeSession.hpp"
#include "autobot/control/InputController.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>
#include <limits>
#include <optional>
#include <unordered_map>
#include <vector>

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

struct PolicyEvent {
    std::size_t tick = 0;
    control::InputAction action = control::InputAction::NoPress;

    friend bool operator==(PolicyEvent const&, PolicyEvent const&) = default;
};

std::vector<PolicyEvent> policyTrace(int renderFps) {
    const auto root = frame(0, 0.0, 0.0);
    const std::vector<bool> desiredHold{
        false, false, true, true, true, false, false, true, true, false
    };

    std::vector<PolicyEvent> events;
    bool holding = false;
    std::size_t lastTick = std::numeric_limits<std::size_t>::max();
    const int maxFrames = renderFps / 2;

    for (int renderFrame = 0; renderFrame <= maxFrames; ++renderFrame) {
        const double t = static_cast<double>(renderFrame) / static_cast<double>(renderFps);
        const auto current = frame(
            static_cast<std::uint64_t>(renderFrame),
            t,
            t * 300.0
        );
        const auto tick = presolve::physicsTicksElapsed(root, current, 60.0);
        if (tick >= desiredHold.size()) break;
        if (tick == lastTick) continue;

        const bool wantHold = desiredHold[tick];
        const auto requested = wantHold
            ? (holding ? control::InputAction::Hold : control::InputAction::Press)
            : (holding ? control::InputAction::Release : control::InputAction::NoPress);
        const auto transition = control::InputController::transition(holding, requested);
        events.push_back({tick, transition.effectiveAction});
        holding = transition.nextHolding;
        lastTick = tick;
    }
    return events;
}

std::size_t firstRollingRerootTick(int renderFps) {
    const auto root = frame(0, 0.0, 0.0);
    auto previous = root;

    for (int renderFrame = 1; renderFrame <= renderFps; ++renderFrame) {
        const double t = static_cast<double>(renderFrame) / static_cast<double>(renderFps);
        const auto current = frame(
            static_cast<std::uint64_t>(renderFrame),
            t,
            t * 300.0
        );
        const auto physicsTick = presolve::physicsTicksElapsed(root, current, 60.0);
        const auto assessment = presolve::assessLiveRoot(
            root,
            previous,
            current,
            physicsTick,
            0
        );
        if (assessment.reroot) return physicsTick;
        previous = current;
    }
    return std::numeric_limits<std::size_t>::max();
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
        const auto physicsTick = presolve::physicsTicksElapsed(root, current, 60.0);
        assert(physicsTick == i);
        const auto assessment = presolve::assessLiveRoot(
            root, previous, current, physicsTick, 0
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

    const auto trace60 = policyTrace(60);
    const auto trace144 = policyTrace(144);
    const auto trace240 = policyTrace(240);
    assert(!trace60.empty());
    assert(trace60 == trace144);
    assert(trace60 == trace240);
    assert(trace60[2].action == control::InputAction::Press);
    assert(trace60[3].action == control::InputAction::Hold);
    assert(trace60[5].action == control::InputAction::Release);

    const auto reroot60 = firstRollingRerootTick(60);
    const auto reroot144 = firstRollingRerootTick(144);
    const auto reroot240 = firstRollingRerootTick(240);
    assert(reroot60 == 24);
    assert(reroot144 == reroot60);
    assert(reroot240 == reroot60);

    const auto consumed = frame(1000, 6.0 / 60.0, 30.0);
    const auto consumedTick = presolve::physicsTicksElapsed(root, consumed, 60.0);
    const auto consumedAssessment = presolve::assessLiveRoot(
        root, root, consumed, consumedTick, 6
    );
    assert(consumedAssessment.reroot);
    assert(consumedAssessment.reason == "policy-prefix-consumed");

    std::cout
        << "POLICY_TIMEBASE_INVARIANCE_TEST=PASS "
        << "fps=60/144/240 policy_ticks_and_actions_identical=YES\n";

    std::cout
        << "ROLLING_SEARCH_PERSISTENCE_TEST=PASS "
        << "real_frames_gt_1=YES root_survives=YES reroots=0 "
        << "expansions=" << persistentStats.totalExpansions
        << " depth=" << persistentStats.currentDepth
        << " reroot_physics_tick=" << reroot60
        << " fps_invariant=YES\n";
    return 0;
}
