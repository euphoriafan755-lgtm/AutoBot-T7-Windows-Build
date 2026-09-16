#include "autobot/solver/ActionGenerator.hpp"

#include <algorithm>
#include <array>

namespace autobot::solver {

ActionCandidate ModeActionGenerator::constant(
    char const* label,
    bool hold,
    std::size_t ticks
) {
    ActionCandidate candidate{};
    candidate.label = label;
    candidate.segments.push_back({
        hold,
        static_cast<std::uint16_t>(std::min<std::size_t>(ticks, 65535))
    });
    return candidate;
}

ActionCandidate ModeActionGenerator::delayedTap(
    char const* label,
    std::size_t delay,
    std::size_t horizon
) {
    ActionCandidate candidate{};
    candidate.label = label;
    if (delay > 0) {
        candidate.segments.push_back({
            false,
            static_cast<std::uint16_t>(std::min<std::size_t>(delay, 65535))
        });
    }
    candidate.segments.push_back({true, 1});
    if (horizon > delay + 1) {
        candidate.segments.push_back({
            false,
            static_cast<std::uint16_t>(
                std::min<std::size_t>(horizon - delay - 1, 65535)
            )
        });
    }
    return candidate;
}

ActionCandidate ModeActionGenerator::holdThenRelease(
    char const* label,
    std::size_t holdTicks,
    std::size_t horizon
) {
    ActionCandidate candidate{};
    candidate.label = label;
    candidate.segments.push_back({
        true,
        static_cast<std::uint16_t>(std::min<std::size_t>(holdTicks, 65535))
    });
    if (horizon > holdTicks) {
        candidate.segments.push_back({
            false,
            static_cast<std::uint16_t>(
                std::min<std::size_t>(horizon - holdTicks, 65535)
            )
        });
    }
    return candidate;
}

ActionCandidate ModeActionGenerator::releaseThenHold(
    char const* label,
    std::size_t releaseTicks,
    std::size_t horizon
) {
    ActionCandidate candidate{};
    candidate.label = label;
    candidate.segments.push_back({
        false,
        static_cast<std::uint16_t>(std::min<std::size_t>(releaseTicks, 65535))
    });
    if (horizon > releaseTicks) {
        candidate.segments.push_back({
            true,
            static_cast<std::uint16_t>(
                std::min<std::size_t>(horizon - releaseTicks, 65535)
            )
        });
    }
    return candidate;
}

std::vector<ActionCandidate> ModeActionGenerator::generate(
    core::GameSnapshot const& snapshot,
    std::size_t horizonTicks
) const {
    const std::size_t horizon = std::clamp<std::size_t>(horizonTicks, 16, 160);
    std::vector<ActionCandidate> result;
    result.reserve(10);

    switch (snapshot.player.mode) {
        case core::GameMode::Cube:
            result.push_back(constant("NO PRESS", false, horizon));
            result.push_back(delayedTap("PRESS NOW", 0, horizon));
            result.push_back(delayedTap("PRESS +1", 1, horizon));
            result.push_back(delayedTap("PRESS +2", 2, horizon));
            result.push_back(delayedTap("PRESS +3", 3, horizon));
            result.push_back(delayedTap("PRESS +4", 4, horizon));
            result.push_back(delayedTap("PRESS +5", 5, horizon));
            break;

        case core::GameMode::Ship:
            result.push_back(constant("RELEASE", false, horizon));
            result.push_back(constant("HOLD", true, horizon));
            result.push_back(holdThenRelease("HOLD->RELEASE 4", 4, horizon));
            result.push_back(holdThenRelease("HOLD->RELEASE 8", 8, horizon));
            result.push_back(holdThenRelease("HOLD->RELEASE 16", 16, horizon));
            result.push_back(releaseThenHold("RELEASE->HOLD 4", 4, horizon));
            result.push_back(releaseThenHold("RELEASE->HOLD 8", 8, horizon));
            result.push_back(releaseThenHold("RELEASE->HOLD 16", 16, horizon));
            break;

        case core::GameMode::Ball:
            result.push_back(constant("NO FLIP", false, horizon));
            result.push_back(delayedTap("FLIP NOW", 0, horizon));
            result.push_back(delayedTap("FLIP +1", 1, horizon));
            result.push_back(delayedTap("FLIP +2", 2, horizon));
            result.push_back(delayedTap("FLIP +4", 4, horizon));
            result.push_back(delayedTap("FLIP +8", 8, horizon));
            break;

        case core::GameMode::Ufo:
            result.push_back(constant("NO TAP", false, horizon));
            result.push_back(delayedTap("TAP NOW", 0, horizon));
            result.push_back(delayedTap("TAP +1", 1, horizon));
            result.push_back(delayedTap("TAP +2", 2, horizon));
            result.push_back(delayedTap("TAP +4", 4, horizon));
            result.push_back(delayedTap("TAP +8", 8, horizon));
            break;

        case core::GameMode::Wave:
            result.push_back(constant("RELEASE", false, horizon));
            result.push_back(constant("HOLD", true, horizon));
            result.push_back(holdThenRelease("UP THEN DOWN 4", 4, horizon));
            result.push_back(holdThenRelease("UP THEN DOWN 8", 8, horizon));
            result.push_back(releaseThenHold("DOWN THEN UP 4", 4, horizon));
            result.push_back(releaseThenHold("DOWN THEN UP 8", 8, horizon));
            break;

        case core::GameMode::Robot:
            result.push_back(constant("NO PRESS", false, horizon));
            result.push_back(holdThenRelease("SHORT HOLD", 2, horizon));
            result.push_back(holdThenRelease("MEDIUM HOLD", 6, horizon));
            result.push_back(holdThenRelease("LONG HOLD", 14, horizon));
            result.push_back(delayedTap("PRESS +2", 2, horizon));
            result.push_back(delayedTap("PRESS +4", 4, horizon));
            break;

        case core::GameMode::Spider:
            result.push_back(constant("NO TELEPORT", false, horizon));
            result.push_back(delayedTap("SURFACE NOW", 0, horizon));
            result.push_back(delayedTap("SURFACE +1", 1, horizon));
            result.push_back(delayedTap("SURFACE +2", 2, horizon));
            result.push_back(delayedTap("SURFACE +4", 4, horizon));
            break;

        case core::GameMode::Swing:
            result.push_back(constant("NO INPUT", false, horizon));
            result.push_back(delayedTap("FLIP NOW", 0, horizon));
            result.push_back(delayedTap("FLIP +1", 1, horizon));
            result.push_back(delayedTap("FLIP +2", 2, horizon));
            result.push_back(delayedTap("FLIP +4", 4, horizon));
            result.push_back(delayedTap("FLIP +8", 8, horizon));
            break;

        case core::GameMode::Unknown:
            result.push_back(constant("SAFE STOP", false, horizon));
            break;
    }
    return result;
}

} // namespace autobot::solver
