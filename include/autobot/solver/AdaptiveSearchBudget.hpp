#pragma once

#include "autobot/core/GameSnapshot.hpp"
#include "autobot/solver/GeneralWorldModel.hpp"
#include "autobot/solver/SolverTypes.hpp"

#include <cstddef>

namespace autobot::solver {

struct SearchBudget {
    std::size_t maxCandidateDelay = 5;
    std::size_t candidateStride = 1;
    std::size_t horizonMin = 24;
    std::size_t horizonMax = 256;
    std::size_t globalSlices = 12;
    std::size_t globalBeamWidth = 8;
    double globalLookaheadX = 1800.0;
    double globalLookaheadY = 1000.0;
    double complexity = 0.0;
};

class AdaptiveSearchBudget final {
public:
    [[nodiscard]] static SearchBudget choose(
        core::GameSnapshot const& snapshot,
        LocalWorldView const& local,
        GeneralWorldView const& global,
        ModelError const& modelError
    );
};

} // namespace autobot::solver
