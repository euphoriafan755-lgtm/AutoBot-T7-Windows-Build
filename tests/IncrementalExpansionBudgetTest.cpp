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

std::uint64_t encodeAction(UniversalAction const& a) {
    std::uint64_t v = 0;
    v |= static_cast<std::uint64_t>(a.p1Hold) << 0U;
    v |= static_cast<std::uint64_t>(a.p1Left) << 1U;
    v |= static_cast<std::uint64_t>(a.p1Right) << 2U;
    v |= static_cast<std::uint64_t>(a.p2Hold) << 3U;
    v |= static_cast<std::uint64_t>(a.p2Left) << 4U;
    v |= static_cast<std::uint64_t>(a.p2Right) << 5U;
    return v;
}

struct State {
    std::size_t phase = 0;
    std::uint64_t actionCode = 0;
};

class LargeDualPlatformerOracle final : public IUniversalStateOracle {
public:
    UniversalObservation observe() const override {
        UniversalObservation out{};
        out.valid = true;
        out.dual = true;
        out.platformer = true;
        out.complete = m_state.phase >= 2;
        if (m_state.phase == 0) out.progress = 0.0;
        else if (m_state.phase == 1) out.progress = 10.0 + static_cast<double>(m_state.actionCode);
        else out.progress = 100.0;
        return out;
    }

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
        ++m_stepCalls;
        if (m_state.phase == 0) {
            ++m_rootCandidateSteps;
            m_state.phase = 1;
            m_state.actionCode = encodeAction(action);
        } else if (m_state.phase == 1) {
            m_state.phase = 2;
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
            static_cast<std::uint64_t>(it->second.phase),
            it->second.actionCode,
        };
        out.hash = {
            0x494e4352454d454eULL ^ static_cast<std::uint64_t>(it->second.phase),
            0x54414c4255444745ULL ^ it->second.actionCode,
        };
        out.completeRepresentation = true;
        return out;
    }

    std::uint64_t decisionEpoch() const override {
        return static_cast<std::uint64_t>(m_state.phase);
    }

    std::size_t stepCalls() const { return m_stepCalls; }
    std::size_t rootCandidateSteps() const { return m_rootCandidateSteps; }

private:
    State m_state{};
    UniversalToken m_nextToken = 0;
    std::unordered_map<UniversalToken, State> m_snapshots;
    std::size_t m_stepCalls = 0;
    std::size_t m_rootCandidateSteps = 0;
};

struct RunResult {
    std::vector<UniversalAction> policy;
    std::vector<UniversalCanonicalState> replayStates;
    UniversalSearchStats stats{};
    std::size_t calls = 0;
    std::size_t yields = 0;
    std::size_t oracleSteps = 0;
    std::size_t rootCandidateSteps = 0;
    bool budgetRespected = true;
};

RunResult run(std::size_t budget) {
    LargeDualPlatformerOracle oracle;
    UniversalSearchCore search;
    assert(search.begin(oracle));

    RunResult result{};
    std::size_t guard = 0;
    while (!search.ready()
        && search.stage() != UniversalSearchStage::Error
        && search.stage() != UniversalSearchStage::Exhausted
        && guard++ < 10000) {
        const auto before = search.stats();
        const auto beforeSteps = oracle.stepCalls();
        const auto after = search.work(oracle, budget);
        const auto expansionDelta = after.totalExpansions - before.totalExpansions;
        const auto engineDelta = oracle.stepCalls() - beforeSteps;

        if (expansionDelta > budget || engineDelta > budget) {
            result.budgetRespected = false;
        }
        ++result.calls;
        if (!search.ready()) ++result.yields;
    }

    assert(search.ready());
    result.policy = search.policy();
    result.replayStates = search.replayStates();
    result.stats = search.stats();
    result.oracleSteps = oracle.stepCalls();
    result.rootCandidateSteps = oracle.rootCandidateSteps();
    return result;
}

bool sameReplay(
    std::vector<UniversalCanonicalState> const& a,
    std::vector<UniversalCanonicalState> const& b
) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (a[i].words != b[i].words) return false;
    }
    return true;
}

} // namespace

int main() {
    constexpr std::size_t kSmallBudget = 3;
    constexpr std::size_t kRootCandidates = 36 * 3;
    constexpr std::size_t kExpectedSearchExpansions = kRootCandidates + 1;

    const auto uninterrupted = run(10000);
    const auto sliced = run(kSmallBudget);

    const bool multipleYields = sliced.yields > 1;
    const bool continuationPreserved =
        sliced.stats.totalExpansions == kExpectedSearchExpansions
        && sliced.stats.totalExpansions == uninterrupted.stats.totalExpansions
        && sliced.rootCandidateSteps == uninterrupted.rootCandidateSteps;
    const bool eventualCompletion =
        sliced.stats.stage == UniversalSearchStage::Ready
        && !sliced.policy.empty()
        && sliced.replayStates.size() == sliced.policy.size();
    const bool resultEquivalent =
        sliced.policy == uninterrupted.policy
        && sameReplay(sliced.replayStates, uninterrupted.replayStates)
        && sliced.stats.bestProgress == uninterrupted.stats.bestProgress;
    const bool budgetRespected = sliced.budgetRespected;

    assert(uninterrupted.stats.totalExpansions == kExpectedSearchExpansions);
    assert(multipleYields);
    assert(continuationPreserved);
    assert(eventualCompletion);
    assert(resultEquivalent);
    assert(budgetRespected);

    std::cout
        << "INCREMENTAL_EXPANSION_BUDGET_TEST=PASS "
        << "multiple_yields=YES "
        << "continuation_preserved=YES "
        << "eventual_expansion_completion=YES "
        << "result_equivalent=YES "
        << "budget_respected_per_slice=YES "
        << "calls=" << sliced.calls
        << " yields=" << sliced.yields
        << " expansions=" << sliced.stats.totalExpansions
        << " policyTicks=" << sliced.policy.size()
        << "\n";
    return 0;
}
