#include "autobot/benchmark/UnseenBenchmark.hpp"

#include <cassert>
#include <iostream>

using namespace autobot;

int main() {
    benchmark::UnseenBenchmarkRecorder recorder{};
    recorder.reset();
    recorder.beginAttempt();

    core::GameSnapshot snapshot{};
    snapshot.valid = true;
    snapshot.levelProgress = 42.5f;
    snapshot.player.mode = core::GameMode::Cube;
    solver::PlanDecision plan{};
    plan.plannerDurationMs = 2.0;
    recorder.observe(snapshot, plan, 0);

    snapshot.levelProgress = 77.0f;
    snapshot.player.mode = core::GameMode::Ship;
    plan.plannerDurationMs = 4.0;
    recorder.observe(snapshot, plan, 1);
    recorder.recordFailure(benchmark::FailureClassification::PlannerError);

    const auto& metrics = recorder.metrics();
    assert(metrics.attempts == 1);
    assert(metrics.bestProgress > 76.9);
    assert(metrics.modeTransitions == 1);
    assert(metrics.falseSafes == 1);
    assert(metrics.plannerSamples == 2);
    assert(metrics.plannerLatencyAverageMs() == 3.0);
    assert(metrics.lastFailure == benchmark::FailureClassification::PlannerError);
    std::cout << "UNSEEN_BENCHMARK_HARNESS_TEST=PASS\n";
    return 0;
}
