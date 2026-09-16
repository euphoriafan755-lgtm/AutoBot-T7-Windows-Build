#include "autobot/solver/AdaptiveSearchBudget.hpp"

#include <algorithm>
#include <cmath>

namespace autobot::solver {
namespace {

double modeComplexity(core::GameMode mode) {
    switch (mode) {
        case core::GameMode::Cube: return 0.10;
        case core::GameMode::Ball: return 0.18;
        case core::GameMode::Ufo: return 0.28;
        case core::GameMode::Robot: return 0.30;
        case core::GameMode::Spider: return 0.34;
        case core::GameMode::Ship: return 0.42;
        case core::GameMode::Wave: return 0.48;
        case core::GameMode::Swing: return 0.50;
        case core::GameMode::Unknown: return 0.75;
    }
    return 0.75;
}

} // namespace

SearchBudget AdaptiveSearchBudget::choose(
    core::GameSnapshot const& snapshot,
    LocalWorldView const& local,
    GeneralWorldView const& global,
    ModelError const& modelError
) {
    SearchBudget result{};

    const double localDensity = std::clamp(
        static_cast<double>(local.hazards.size() + local.solids.size()) / 24.0,
        0.0,
        1.0
    );
    const double portalDensity = std::clamp(
        static_cast<double>(global.portals) / 5.0,
        0.0,
        1.0
    );
    const double dynamicDensity = std::clamp(
        static_cast<double>(global.dynamic) / 10.0,
        0.0,
        1.0
    );
    const double unknownDensity = std::clamp(
        static_cast<double>(global.unknown) / 8.0,
        0.0,
        1.0
    );
    const double divergence = std::clamp(modelError.magnitude() / 80.0, 0.0, 1.0);

    result.complexity = std::clamp(
        modeComplexity(snapshot.player.mode)
            + localDensity * 0.30
            + portalDensity * 0.18
            + dynamicDensity * 0.22
            + unknownDensity * 0.12
            + divergence * 0.30,
        0.0,
        1.0
    );

    // Difficulty labels never participate. Search grows only from observed
    // scene complexity and model uncertainty.
    result.maxCandidateDelay = static_cast<std::size_t>(
        std::clamp(std::lround(5.0 + result.complexity * 19.0), 5L, 24L)
    );
    result.candidateStride = result.complexity >= 0.70 ? 1 : (result.complexity >= 0.35 ? 2 : 3);
    result.horizonMin = static_cast<std::size_t>(
        std::clamp(std::lround(24.0 + result.complexity * 40.0), 24L, 64L)
    );
    result.horizonMax = static_cast<std::size_t>(
        std::clamp(std::lround(160.0 + result.complexity * 352.0), 160L, 512L)
    );
    result.globalSlices = static_cast<std::size_t>(
        std::clamp(std::lround(10.0 + result.complexity * 18.0), 10L, 28L)
    );
    result.globalBeamWidth = static_cast<std::size_t>(
        std::clamp(std::lround(6.0 + result.complexity * 18.0), 6L, 24L)
    );
    result.globalLookaheadX = std::clamp(1400.0 + result.complexity * 4600.0, 1400.0, 6000.0);
    result.globalLookaheadY = std::clamp(800.0 + result.complexity * 1400.0, 800.0, 2200.0);
    return result;
}

} // namespace autobot::solver
