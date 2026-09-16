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
    SearchBudget budget{};
    return generate(snapshot, horizonTicks, budget);
}

std::vector<ActionCandidate> ModeActionGenerator::generate(
    core::GameSnapshot const& snapshot,
    std::size_t horizonTicks,
    SearchBudget const& budget
) const {
    const std::size_t horizon = std::clamp<std::size_t>(
        horizonTicks,
        std::max<std::size_t>(16, budget.horizonMin),
        std::max<std::size_t>(budget.horizonMin, budget.horizonMax)
    );
    const std::size_t delayMax = std::min<std::size_t>(
        std::min<std::size_t>(budget.maxCandidateDelay, 48),
        horizon > 0 ? horizon - 1 : 0
    );
    const std::size_t stride = std::max<std::size_t>(1, budget.candidateStride);

    std::vector<ActionCandidate> result;
    result.reserve(32);

    auto addDelayed = [&](char const* prefix, std::size_t delay) {
        ActionCandidate candidate = delayedTap("", delay, horizon);
        if (delay == 0) candidate.label = std::string(prefix) + " NOW";
        else candidate.label = std::string(prefix) + " +" + std::to_string(delay);
        result.push_back(std::move(candidate));
    };

    auto addDelayFamily = [&](char const* noInputLabel, char const* pressPrefix) {
        result.push_back(constant(noInputLabel, false, horizon));
        addDelayed(pressPrefix, 0);

        // Always keep the immediate precision window dense. Adaptive search
        // adds farther candidates; it must not remove +1..+5 semantics that
        // the committed-action countdown relies on.
        const std::size_t denseEnd = std::min<std::size_t>(5, delayMax);
        for (std::size_t delay = 1; delay <= denseEnd; ++delay) {
            addDelayed(pressPrefix, delay);
        }
        if (delayMax > denseEnd) {
            for (std::size_t delay = denseEnd + 1; delay <= delayMax; delay += stride) {
                addDelayed(pressPrefix, delay);
            }
            const std::size_t tailStart = denseEnd + 1;
            if (delayMax >= tailStart && ((delayMax - tailStart) % stride) != 0) {
                addDelayed(pressPrefix, delayMax);
            }
        }
    };

    auto holdDurations = [&]() {
        std::vector<std::size_t> values{2, 4, 8, 16};
        if (budget.complexity >= 0.35) values.push_back(24);
        if (budget.complexity >= 0.60) values.push_back(32);
        if (budget.complexity >= 0.80) values.push_back(48);
        values.erase(
            std::remove_if(values.begin(), values.end(), [&](std::size_t v) {
                return v >= horizon;
            }),
            values.end()
        );
        return values;
    };

    switch (snapshot.player.mode) {
        case core::GameMode::Cube:
            addDelayFamily("NO PRESS", "PRESS");
            break;

        case core::GameMode::Ship:
        case core::GameMode::Wave: {
            result.push_back(constant("RELEASE", false, horizon));
            result.push_back(constant("HOLD", true, horizon));
            for (auto duration : holdDurations()) {
                auto a = holdThenRelease("", duration, horizon);
                a.label = "HOLD->RELEASE " + std::to_string(duration);
                result.push_back(std::move(a));
                auto b = releaseThenHold("", duration, horizon);
                b.label = "RELEASE->HOLD " + std::to_string(duration);
                result.push_back(std::move(b));
            }
            break;
        }

        case core::GameMode::Ball:
            addDelayFamily("NO FLIP", "FLIP");
            break;

        case core::GameMode::Ufo:
            addDelayFamily("NO TAP", "TAP");
            break;

        case core::GameMode::Robot: {
            result.push_back(constant("NO PRESS", false, horizon));
            for (auto duration : holdDurations()) {
                auto a = holdThenRelease("", duration, horizon);
                a.label = "ROBOT HOLD " + std::to_string(duration);
                result.push_back(std::move(a));
            }
            for (std::size_t delay = 0; delay <= delayMax; delay += stride) {
                addDelayed("PRESS", delay);
            }
            break;
        }

        case core::GameMode::Spider:
            addDelayFamily("NO TELEPORT", "SURFACE");
            break;

        case core::GameMode::Swing:
            addDelayFamily("NO INPUT", "FLIP");
            break;

        case core::GameMode::Unknown:
            result.push_back(constant("SAFE STOP", false, horizon));
            break;
    }
    return result;
}

} // namespace autobot::solver
