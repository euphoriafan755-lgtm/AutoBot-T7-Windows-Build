#pragma once

#include "autobot/core/GameSnapshot.hpp"
#include "autobot/solver/AdaptiveSearchBudget.hpp"
#include "autobot/solver/SolverTypes.hpp"

#include <cstddef>
#include <vector>

namespace autobot::solver {

class ModeActionGenerator final {
public:
    [[nodiscard]] std::vector<ActionCandidate> generate(
        core::GameSnapshot const& snapshot,
        std::size_t horizonTicks
    ) const;

    [[nodiscard]] std::vector<ActionCandidate> generate(
        core::GameSnapshot const& snapshot,
        std::size_t horizonTicks,
        SearchBudget const& budget
    ) const;

private:
    static ActionCandidate constant(char const* label, bool hold, std::size_t ticks);
    static ActionCandidate delayedTap(char const* label, std::size_t delay, std::size_t horizon);
    static ActionCandidate holdThenRelease(
        char const* label,
        std::size_t holdTicks,
        std::size_t horizon
    );
    static ActionCandidate releaseThenHold(
        char const* label,
        std::size_t releaseTicks,
        std::size_t horizon
    );
};

} // namespace autobot::solver
