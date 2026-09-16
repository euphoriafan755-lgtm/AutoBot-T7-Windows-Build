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
    const double horizontal = std::min(
        1.0,
        static_cast<double>(c.horizontalScaleSamples) / 8.0
    );
    const double vertical = c.verticalScaleSamples == 0
        ? 0.35
        : std::min(1.0, static_cast<double>(c.verticalScaleSamples) / 8.0);
    const double input = std::min(
        1.0,
        static_cast<double>(c.pressSamples + c.holdSamples + c.releaseSamples) / 10.0
    );
    c.confidence = std::clamp(
        0.30 * timing + 0.40 * horizontal + 0.20 * vertical + 0.10 * input,
        0.0,
        1.0
    );
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

    const double dtSeconds = current.levelTime - m_previous.levelTime;
    const bool comparable = m_previous.valid
        && !current.player.dead
        && !m_previous.player.dead
        && current.player.mode == m_previous.player.mode
        && dtSeconds > 0.00001
        && dtSeconds < 0.25;

    if (!comparable) {
        ++m_stats.rejectedSamples;
        m_previous = current;
        return;
    }

    auto& calibration = m_calibrations[modeIndex(current.player.mode)];
    ema(calibration.sampleDt, dtSeconds, 0.18);
    ++calibration.timingSamples;

    const double dx = current.player.x - m_previous.player.x;
    const double dy = current.player.y - m_previous.player.y;
    const double rawVx = (current.player.velocityX + m_previous.player.velocityX) * 0.5;

    if (std::abs(rawVx) > 0.01) {
        const double normalizedTicks = dx / rawVx;
        const double ticksPerSecond = normalizedTicks / dtSeconds;
        if (std::isfinite(ticksPerSecond)
            && ticksPerSecond > 1.0
            && ticksPerSecond < 1000.0) {
            ema(calibration.physicsTicksPerSecond, ticksPerSecond, 0.18);
            ++calibration.horizontalScaleSamples;
        }
    }

    ema(calibration.observedWorldVelocityX, dx / dtSeconds, 0.12);
    ema(calibration.averageVelocityX, dx / dtSeconds, 0.12);
    ema(calibration.observedWorldVelocityY, dy / dtSeconds, 0.12);

    const double normalizedDt = calibration.normalizedStepDt();
    if (normalizedDt > 0.00001
        && !current.player.grounded
        && !m_previous.player.grounded
        && std::abs(current.player.velocityY) > 0.25) {
        const double scale = dy / (current.player.velocityY * normalizedDt);
        if (std::isfinite(scale) && scale > 0.3 && scale < 1.5) {
            ema(calibration.verticalPositionScale, scale, 0.12);
            ++calibration.verticalScaleSamples;
        }
    }

    const double dvY = current.player.velocityY - m_previous.player.velocityY;
    const double rawAccelerationY = normalizedDt > 0.00001 ? dvY / normalizedDt : 0.0;

    switch (previousAction) {
        case control::InputAction::Press:
            ema(calibration.pressVelocityY, current.player.velocityY, 0.20);
            ++calibration.pressSamples;
            if (previousDesiredHold && normalizedDt > 0.00001) {
                ema(calibration.holdAccelerationY, rawAccelerationY, 0.12);
                ema(calibration.averageHoldVelocityY, current.player.velocityY, 0.12);
                ++calibration.holdSamples;
            }
            break;

        case control::InputAction::Hold:
            if (normalizedDt > 0.00001) {
                ema(calibration.holdAccelerationY, rawAccelerationY, 0.12);
                ema(calibration.averageHoldVelocityY, current.player.velocityY, 0.12);
                ++calibration.holdSamples;
            }
            break;

        case control::InputAction::Release:
            if (normalizedDt > 0.00001) {
                ema(calibration.releaseAccelerationY, rawAccelerationY, 0.12);
                ema(calibration.averageReleaseVelocityY, current.player.velocityY, 0.12);
                ++calibration.releaseSamples;
            }
            break;

        case control::InputAction::NoPress:
        case control::InputAction::SafeStop:
            // Grounded frames often have vy clamped by collision resolution and
            // therefore do not measure gravity. Only learn free-flight gravity
            // from clean airborne samples.
            if (normalizedDt > 0.00001
                && !current.player.grounded
                && !m_previous.player.grounded) {
                ema(calibration.neutralAccelerationY, rawAccelerationY, 0.10);
                ema(calibration.averageReleaseVelocityY, current.player.velocityY, 0.10);
                ++calibration.neutralSamples;
            }
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
    error.collisionMatched = (!actual.player.dead) == predicted.alive;

    m_stats.lastError = error;
    const double magnitude = error.magnitude()
        + (error.modeMatched ? 0.0 : 100.0)
        + (error.landingMatched ? 0.0 : 50.0)
        + (error.collisionMatched ? 0.0 : 100.0);
    if (m_stats.errorEma == 0.0) m_stats.errorEma = magnitude;
    else m_stats.errorEma = m_stats.errorEma * 0.9 + magnitude * 0.1;
}

ModeCalibration const& PhysicsValidationHarness::calibration(core::GameMode mode) const {
    return m_calibrations[modeIndex(mode)];
}

bool PhysicsValidationHarness::modeReady(core::GameMode mode) const {
    auto const& c = calibration(mode);
    return c.timingReady() && c.horizontalReady();
}

} // namespace autobot::solver
