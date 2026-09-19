#include "autobot/presolve/UniversalSearchCore.hpp"

#include <cassert>
#include <cstddef>
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

class BootstrapOracle final : public IUniversalStateOracle {
public:
    UniversalObservation observe() const override {
        UniversalObservation out{};
        out.valid = true;
        out.dead = m_state.dead;
        out.complete = false;
        out.progress = static_cast<double>(m_state.tick) / 100.0;
        return out;
    }

    std::optional<UniversalToken> capture() override {
        ++m_captureCount;
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
        ++m_stepCount;
        if (action.p1Hold || action.p1Left || action.p1Right
            || action.p2Hold || action.p2Left || action.p2Right) {
            m_state.dead = true;
        } else {
            ++m_state.tick;
        }
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
            static_cast<std::uint64_t>(it->second.tick),
            static_cast<std::uint64_t>(it->second.dead),
        };
        out.hash = {
            0x424f4f5453545241ULL ^ static_cast<std::uint64_t>(it->second.tick),
            0x5053454152434800ULL ^ static_cast<std::uint64_t>(it->second.dead),
        };
        out.completeRepresentation = true;
        return out;
    }

    std::size_t captureCount() const { return m_captureCount; }
    std::size_t stepCount() const { return m_stepCount; }

private:
    State m_state{};
    UniversalToken m_nextToken = 0;
    std::unordered_map<UniversalToken, State> m_snapshots;
    std::size_t m_captureCount = 0;
    std::size_t m_stepCount = 0;
};

} // namespace

int main() {
    BootstrapOracle oracle;
    UniversalSearchCore search;

    std::size_t resetCallbacks = 0;
    search.setLifecycleGeneration(1);
    search.setLifecycleCallback([&](
        std::uint64_t generation,
        UniversalSearchStage,
        UniversalSearchStage newStage,
        std::string_view,
        std::size_t
    ) {
        assert(generation == 1);
        assert(newStage == UniversalSearchStage::Idle);
        ++resetCallbacks;
    });

    const bool began = search.begin(oracle);
    assert(began);

    const auto afterBegin = search.stats();
    assert(afterBegin.started);
    assert(afterBegin.stage == UniversalSearchStage::Searching);
    assert(afterBegin.frontierSize == 1);
    assert(afterBegin.totalExpansions == 0);
    assert(afterBegin.totalEngineSteps == 0);
    assert(oracle.captureCount() >= 1);

    const auto bootstrapResetSerial = afterBegin.resetSerial;
    const auto bootstrapResetCallbacks = resetCallbacks;

    const auto afterSlice = search.work(oracle, 1);

    assert(afterSlice.started);
    assert(afterSlice.stage == UniversalSearchStage::Searching);
    assert(afterSlice.frontierSize > 0);
    assert(afterSlice.totalExpansions > 0);
    assert(afterSlice.totalEngineSteps > 0);
    assert(afterSlice.elapsedSeconds > 0.0);
    assert(oracle.stepCount() > 0);

    // No reset is allowed between a successful begin and the first scheduled slice.
    assert(afterSlice.resetSerial == bootstrapResetSerial);
    assert(resetCallbacks == bootstrapResetCallbacks);

    std::cout
        << "RUNTIME_SEARCH_BOOTSTRAP_TEST=PASS "
        << "root_captured=YES "
        << "frontier_after_begin=" << afterBegin.frontierSize << " "
        << "first_slice_executed=YES "
        << "expansions=" << afterSlice.totalExpansions << " "
        << "engineSteps=" << afterSlice.totalEngineSteps << " "
        << "elapsed_advances=YES "
        << "session_reset_between_begin_and_slice=NO"
        << "\n";
    return 0;
}
