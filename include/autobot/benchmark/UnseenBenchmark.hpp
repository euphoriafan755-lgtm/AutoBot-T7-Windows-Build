#pragma once

#include "autobot/core/GameSnapshot.hpp"
#include "autobot/solver/SolverTypes.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace autobot::benchmark {

enum class FailureClassification {
    None,
    PhysicsModelError,
    CollisionModelError,
    TriggerModelError,
    PlannerError,
    InputExecutionError,
    StateDesync,
    UnmodeledObject,
    RandomBranch,
    NoSafePath,
};

inline constexpr std::string_view toString(FailureClassification value) {
    switch (value) {
        case FailureClassification::None: return "NONE";
        case FailureClassification::PhysicsModelError: return "PHYSICS_MODEL_ERROR";
        case FailureClassification::CollisionModelError: return "COLLISION_MODEL_ERROR";
        case FailureClassification::TriggerModelError: return "TRIGGER_MODEL_ERROR";
        case FailureClassification::PlannerError: return "PLANNER_ERROR";
        case FailureClassification::InputExecutionError: return "INPUT_EXECUTION_ERROR";
        case FailureClassification::StateDesync: return "STATE_DESYNC";
        case FailureClassification::UnmodeledObject: return "UNMODELED_OBJECT";
        case FailureClassification::RandomBranch: return "RANDOM_BRANCH";
        case FailureClassification::NoSafePath: return "NO_SAFE_PATH";
    }
    return "NONE";
}

struct UnseenBenchmarkMetrics {
    std::string levelLabel = "UNKNOWN LEVEL";
    std::size_t attempts = 0;
    double bestProgress = 0.0;
    bool completed = false;
    std::size_t modeTransitions = 0;
    std::size_t falseSafes = 0;
    std::size_t modelDivergences = 0;
    std::size_t plannerSamples = 0;
    double plannerLatencyTotalMs = 0.0;
    double plannerLatencyMaxMs = 0.0;
    FailureClassification lastFailure = FailureClassification::None;

    [[nodiscard]] double plannerLatencyAverageMs() const {
        return plannerSamples == 0 ? 0.0
            : plannerLatencyTotalMs / static_cast<double>(plannerSamples);
    }
};

class UnseenBenchmarkRecorder final {
public:
    void reset(std::string levelLabel = "UNKNOWN LEVEL");
    void beginAttempt();
    void observe(
        core::GameSnapshot const& snapshot,
        solver::PlanDecision const& plan,
        std::size_t falseSafeTotal
    );
    void recordFailure(FailureClassification failure);
    void markCompleted();

    [[nodiscard]] UnseenBenchmarkMetrics const& metrics() const { return m_metrics; }

private:
    UnseenBenchmarkMetrics m_metrics{};
    core::GameMode m_previousMode = core::GameMode::Unknown;
    core::GameMode m_previousModeP2 = core::GameMode::Unknown;
    bool m_hasPreviousMode = false;
    std::size_t m_lastFalseSafeTotal = 0;
};

} // namespace autobot::benchmark
