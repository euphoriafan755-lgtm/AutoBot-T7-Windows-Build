#include "autobot/solver/PhysicsValidationHarness.hpp"

#include <algorithm>
#include <cmath>

namespace autobot::solver {

void PhysicsValidationHarness::reset() {
    m_hasPrevious = false;
    m_previous = {};
    m_calibrations = {};
    m_stats = {};
}

std::size_t PhysicsValidationHarness::modeIndex(core::GameMode mode) {
    switch (mode) {
        case core::GameMode::Cube: return 0;
        case core::GameMode::Ship: return 1;
        case core::GameMode::Ball: return 2;
        case core::GameMode::Ufo: return 3;
        case core::GameMode::Wave: return 4;
        case core::GameMode::Robot: return 5;
        case core::GameMode::Spider: return 6;
        case core::GameMode::Swing: return 7;
        case core::GameMode::Unknown: return 8;
    }
    return 8;
}

void PhysicsValidationHarness::ema(double& target, double sample, double alpha) {
    if (!std::isfinite(sample)) return;
    if (target == 0.0) {
        target = sample;
        return;
    }
    target = target * (1.0 - alpha) + sample * alpha;
}

void PhysicsValidationHarness::recomputeConfidence(ModeCalibration& c) {
    const double timing = std::min(1.0, static_cast<double>(c.timingSamples) / 8.0);
    const double neutral = std::min(1.0, static_cast<double>(c.neutralSamples) / 8.0);
    const double input = std::min(
        1.0,
        static_cast<double>(c.pressSamples + c.holdSamples + c.releaseSamples) / 10.0
    );
    c.confidence = std::clamp(0.45 * timing + 0.35 * neutral + 0.20 * input, 0.0, 1.0);
}

void PhysicsValidationHarness::observe(
    core::GameSnapshot const& current,
    control::InputAction previousAction,
    bool previousDesiredHold
) {
    if (!current.valid) {
        m_hasPrevious = false;
        return;
    }

    if (!m_hasPrevious) {
        m_previous = current;
        m_hasPrevious = true;
        return;
    }

    const double dt = current.levelTime - m_previous.levelTime;
    const bool comparable = m_previous.valid
        && !current.player.dead
        && !m_previous.player.dead
        && current.player.mode == m_previous.player.mode
        && dt > 0.00001
        && dt < 0.25;

    if (!comparable) {
        ++m_stats.rejectedSamples;
        m_previous = current;
        return;
    }

    auto& calibration = m_calibrations[modeIndex(current.player.mode)];
    ema(calibration.sampleDt, dt, 0.18);
    ++calibration.timingSamples;

    const double dvY = current.player.velocityY - m_previous.player.velocityY;
    const double accelerationY = dvY / dt;
    ema(calibration.averageVelocityX, current.player.velocityX, 0.12);

    switch (previousAction) {
        case control::InputAction::Press:
            ema(calibration.pressDeltaVelocityY, dvY, 0.20);
            ++calibration.pressSamples;
            if (previousDesiredHold) {
                ema(calibration.holdAccelerationY, accelerationY, 0.12);
                ema(calibration.averageHoldVelocityY, current.player.velocityY, 0.12);
                ++calibration.holdSamples;
            }
            break;

        case control::InputAction::Hold:
            ema(calibration.holdAccelerationY, accelerationY, 0.12);
            ema(calibration.averageHoldVelocityY, current.player.velocityY, 0.12);
            ++calibration.holdSamples;
            break;

        case control::InputAction::Release:
            ema(calibration.releaseAccelerationY, accelerationY, 0.12);
            ema(calibration.averageReleaseVelocityY, current.player.velocityY, 0.12);
            ++calibration.releaseSamples;
            break;

        case control::InputAction::NoPress:
        case control::InputAction::SafeStop:
            ema(calibration.neutralAccelerationY, accelerationY, 0.10);
            ema(calibration.averageReleaseVelocityY, current.player.velocityY, 0.10);
            ++calibration.neutralSamples;
            break;
    }

    recomputeConfidence(calibration);
    ++m_stats.samples;
    m_previous = current;
}

void PhysicsValidationHarness::recordPrediction(
    SimState const& predicted,
    core::GameSnapshot const& actual
) {
    if (!actual.valid) return;

    ModelError error{};
    error.dx = actual.player.x - predicted.x;
    error.dy = actual.player.y - predicted.y;
    error.dvx = actual.player.velocityX - predicted.vx;
    error.dvy = actual.player.velocityY - predicted.vy;
    error.modeMatched = actual.player.mode == predicted.mode;
    error.landingMatched = actual.player.grounded == predicted.grounded;

    m_stats.lastError = error;
    const double magnitude = error.magnitude() + (error.modeMatched ? 0.0 : 100.0);
    if (m_stats.errorEma == 0.0) m_stats.errorEma = magnitude;
    else m_stats.errorEma = m_stats.errorEma * 0.9 + magnitude * 0.1;
}

ModeCalibration const& PhysicsValidationHarness::calibration(core::GameMode mode) const {
    return m_calibrations[modeIndex(mode)];
}

bool PhysicsValidationHarness::modeReady(core::GameMode mode) const {
    auto const& c = calibration(mode);
    return c.timingReady() && c.neutralReady();
}

} // namespace autobot::solver
