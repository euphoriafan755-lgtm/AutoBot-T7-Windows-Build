#pragma once

#include "autobot/core/GameSnapshot.hpp"
#include "autobot/solver/PhysicsValidationHarness.hpp"
#include "autobot/solver/SolverTypes.hpp"

#include <cmath>

namespace autobot::solver {

inline double effectiveNormalizedStepDt(ModeCalibration const& calibration) {
    const double measured = calibration.normalizedStepDt();
    return std::isfinite(measured) && measured > 0.00001 ? measured : 1.0;
}

inline bool provisionalPhysicsUsable(core::PlayerState const& player) {
    return player.mode != core::GameMode::Unknown
        && std::isfinite(player.x)
        && std::isfinite(player.y)
        && std::isfinite(player.velocityX)
        && std::isfinite(player.velocityY)
        && std::isfinite(player.gravity)
        && std::isfinite(player.gravityModifier)
        && std::isfinite(player.jumpVelocity)
        && player.objectBoundsWidth > 0.0
        && player.objectBoundsHeight > 0.0;
}

inline PhysicsModelStatus physicsStatus(
    PhysicsValidationHarness const& validation,
    core::GameMode mode
) {
    if (validation.modeReady(mode)) return PhysicsModelStatus::Verified;

    auto const& c = validation.calibration(mode);
    if (c.timingSamples > 0
        || c.horizontalScaleSamples > 0
        || c.neutralSamples > 0
        || c.inputReady()) {
        return PhysicsModelStatus::Calibrating;
    }
    return PhysicsModelStatus::Provisional;
}

} // namespace autobot::solver
