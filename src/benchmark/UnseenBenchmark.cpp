#include "autobot/benchmark/UnseenBenchmark.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace autobot::benchmark {

void UnseenBenchmarkRecorder::reset(std::string levelLabel) {
    m_metrics = {};
    m_metrics.levelLabel = std::move(levelLabel);
    m_previousMode = core::GameMode::Unknown;
    m_previousModeP2 = core::GameMode::Unknown;
    m_hasPreviousMode = false;
    m_lastFalseSafeTotal = 0;
}

void UnseenBenchmarkRecorder::beginAttempt() {
    ++m_metrics.attempts;
    m_hasPreviousMode = false;
}

void UnseenBenchmarkRecorder::observe(
    core::GameSnapshot const& snapshot,
    solver::PlanDecision const& plan,
    std::size_t falseSafeTotal
) {
    if (!snapshot.valid) return;
    m_metrics.bestProgress = std::max(
        m_metrics.bestProgress,
        static_cast<double>(snapshot.levelProgress)
    );
    if (snapshot.levelProgress >= 99.999f && !snapshot.player.dead) {
        m_metrics.completed = true;
    }

    if (m_hasPreviousMode) {
        if (snapshot.player.mode != m_previousMode) ++m_metrics.modeTransitions;
        if (snapshot.dualMode && snapshot.player2Valid
            && snapshot.player2.mode != m_previousModeP2) {
            ++m_metrics.modeTransitions;
        }
    }
    m_previousMode = snapshot.player.mode;
    m_previousModeP2 = snapshot.player2Valid ? snapshot.player2.mode : core::GameMode::Unknown;
    m_hasPreviousMode = true;

    if (falseSafeTotal > m_lastFalseSafeTotal) {
        m_metrics.falseSafes += falseSafeTotal - m_lastFalseSafeTotal;
    }
    m_lastFalseSafeTotal = falseSafeTotal;

    const auto& error = plan.lastModelError;
    if (error.magnitude() > 25.0 || !error.modeMatched
        || !error.landingMatched || !error.collisionMatched) {
        ++m_metrics.modelDivergences;
    }

    if (std::isfinite(plan.plannerDurationMs) && plan.plannerDurationMs >= 0.0) {
        ++m_metrics.plannerSamples;
        m_metrics.plannerLatencyTotalMs += plan.plannerDurationMs;
        m_metrics.plannerLatencyMaxMs = std::max(
            m_metrics.plannerLatencyMaxMs,
            plan.plannerDurationMs
        );
    }
}

void UnseenBenchmarkRecorder::recordFailure(FailureClassification failure) {
    m_metrics.lastFailure = failure;
}

void UnseenBenchmarkRecorder::markCompleted() {
    m_metrics.completed = true;
    m_metrics.bestProgress = 100.0;
    m_metrics.lastFailure = FailureClassification::None;
}

} // namespace autobot::benchmark
