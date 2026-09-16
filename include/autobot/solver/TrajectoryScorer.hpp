#pragma once

#include "autobot/solver/SolverTypes.hpp"

namespace autobot::solver {

class TrajectoryScorer final {
public:
    [[nodiscard]] double score(TrajectoryResult& trajectory) const;
};

} // namespace autobot::solver
