#include "autobot/solver/TrajectoryScorer.hpp"

#include <algorithm>
#include <cmath>

namespace autobot::solver {

double TrajectoryScorer::score(TrajectoryResult& trajectory) const {
    if (trajectory.fatalCollision || trajectory.hazardCollision) {
        trajectory.score = -1000000.0;
        return trajectory.score;
    }

    // A trajectory that simply failed to reach the relevant event is not SAFE.
    // Keep it far below any conclusive non-fatal candidate so short horizons
    // cannot manufacture a false-safe winner.
    if (!trajectory.horizonConclusive
        || trajectory.classification == TrajectoryClass::HorizonInconclusive) {
        trajectory.score = -500000.0
            + std::clamp(trajectory.progress, 0.0, 1000.0);
        return trajectory.score;
    }

    const double progressScore = std::clamp(trajectory.progress, -500.0, 1500.0) * 1.5;
    const double clearanceScore = std::clamp(trajectory.minimumClearance, 0.0, 240.0) * 8.0;
    const double confidenceScore = trajectory.confidence * 900.0;
    const double portalBonus = trajectory.portalCrossed ? 80.0 : 0.0;
    const double transitionBonus = trajectory.modeChanged ? 120.0 : 0.0;
    const double landingBonus = trajectory.landed ? 40.0 : 0.0;
    const double inputPenalty = static_cast<double>(trajectory.candidate.transitionCount()) * 18.0;
    const double uncertaintyPenalty = trajectory.uncertainGeometry ? 420.0 : 0.0;

    trajectory.score = progressScore
        + clearanceScore
        + confidenceScore
        + portalBonus
        + transitionBonus
        + landingBonus
        - inputPenalty
        - uncertaintyPenalty;

    if (trajectory.classification == TrajectoryClass::Risky) {
        trajectory.score -= 180.0;
    } else if (trajectory.classification == TrajectoryClass::Unknown) {
        trajectory.score -= 600.0;
    }
    return trajectory.score;
}

} // namespace autobot::solver
