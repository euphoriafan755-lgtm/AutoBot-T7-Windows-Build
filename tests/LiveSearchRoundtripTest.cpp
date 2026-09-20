#include "autobot/presolve/UniversalSearchCore.hpp"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <optional>
#include <unordered_map>
#include <vector>

using namespace autobot::presolve;

namespace {

struct LiveState {
    double playerX = 0.0;
    double playerY = 0.0;
    double levelTime = 0.0;
    std::uint64_t triggerState = 0;
    std::uint64_t effectState = 0;
    double dynamicGeometryX = 0.0;
    std::uint64_t rng = 0x12345678ULL;
    int attempts = 1;
    bool paused = false;
    std::vector<int> queuedInputs{1, 0, 1};

    friend bool operator==(LiveState const&, LiveState const&) = default;
};

class LiveRoundtripOracle final : public IUniversalStateOracle {
public:
    UniversalObservation observe() const override {
        UniversalObservation out{};
        out.valid = true;
        out.dead = false;
        out.complete = m_state.playerX >= 50.0;
        out.progress = m_state.playerX * 2.0;
        return out;
    }

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
        m_state.playerX += action.p1Hold ? 2.0 : 1.0;
        m_state.playerY += action.p1Hold ? 0.75 : -0.25;
        m_state.levelTime += 1.0 / 120.0;
        m_state.triggerState ^= 0x11ULL;
        m_state.effectState += 7ULL;
        m_state.dynamicGeometryX += 0.5;
        m_state.rng = m_state.rng * 6364136223846793005ULL + 1ULL;
        m_state.attempts += 2;
        m_state.paused = !m_state.paused;
        m_state.queuedInputs.push_back(action.p1Hold ? 1 : 0);
        return observe();
    }

    void discard(UniversalToken token) override {
        m_snapshots.erase(token);
    }

    std::optional<UniversalCanonicalState> canonicalState(UniversalToken token) const override {
        auto it = m_snapshots.find(token);
        if (it == m_snapshots.end()) return std::nullopt;

        UniversalCanonicalState out{};
        out.words = {
            static_cast<std::uint64_t>(it->second.playerX * 1000.0),
            static_cast<std::uint64_t>(it->second.playerY * 1000.0 + 100000.0),
            static_cast<std::uint64_t>(it->second.levelTime * 1000000.0),
            it->second.triggerState,
            it->second.effectState,
            static_cast<std::uint64_t>(it->second.dynamicGeometryX * 1000.0),
            it->second.rng,
            static_cast<std::uint64_t>(it->second.attempts),
            static_cast<std::uint64_t>(it->second.paused),
            static_cast<std::uint64_t>(it->second.queuedInputs.size()),
        };
        for (int input : it->second.queuedInputs) {
            out.words.push_back(static_cast<std::uint64_t>(input));
        }
        out.hash = {out.words[0] ^ it->second.rng, out.words[1] ^ 0x4c495645ULL};
        out.completeRepresentation = true;
        return out;
    }

    LiveState const& state() const { return m_state; }

private:
    LiveState m_state{};
    UniversalToken m_next = 0;
    std::unordered_map<UniversalToken, LiveState> m_snapshots;
};

} // namespace

int main() {
    LiveRoundtripOracle oracle;
    const LiveState before = oracle.state();

    auto liveRoot = oracle.capture();
    assert(liveRoot);

    UniversalSearchCore search;
    assert(search.begin(oracle));

    const auto afterSearch = search.work(oracle, 4);
    assert(afterSearch.totalExpansions > 0);
    assert(afterSearch.totalEngineSteps > 0);
    assert(!(oracle.state() == before));

    assert(oracle.restore(*liveRoot));
    const LiveState after = oracle.state();

    const bool playerRestored = after.playerX == before.playerX && after.playerY == before.playerY;
    const bool timeRestored = after.levelTime == before.levelTime;
    const bool triggerRestored = after.triggerState == before.triggerState;
    const bool effectRestored = after.effectState == before.effectState;
    const bool dynamicRestored = after.dynamicGeometryX == before.dynamicGeometryX;
    const bool rngRestored = after.rng == before.rng;
    const bool attemptsRestored = after.attempts == before.attempts;
    const bool queueRestored = after.queuedInputs == before.queuedInputs;
    const bool pauseRestored = after.paused == before.paused;

    assert(playerRestored);
    assert(timeRestored);
    assert(triggerRestored);
    assert(effectRestored);
    assert(dynamicRestored);
    assert(rngRestored);
    assert(attemptsRestored);
    assert(queueRestored);
    assert(pauseRestored);

    std::cout
        << "LIVE_SEARCH_ROUNDTRIP_TEST=PASS "
        << "player_restored=YES "
        << "levelTime_restored=YES "
        << "trigger_restored=YES "
        << "effect_restored=YES "
        << "dynamic_geometry_restored=YES "
        << "rng_restored=YES "
        << "attempt_state_restored=YES "
        << "queued_real_input_state_restored=YES "
        << "pause_state_restored=YES "
        << "expansions=" << afterSearch.totalExpansions << " "
        << "engineSteps=" << afterSearch.totalEngineSteps
        << "\n";
    return 0;
}
