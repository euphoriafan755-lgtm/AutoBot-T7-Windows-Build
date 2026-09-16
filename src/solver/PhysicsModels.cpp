#include "autobot/solver/PhysicsModels.hpp"

#include <algorithm>
#include <cmath>

namespace autobot::solver {
namespace {

double stepDt(SimState const& state, PhysicsStepContext const& context) {
    if (context.dt > 0.0) return context.dt;
    if (state.sampleDt > 0.0) return state.sampleDt;
    return 0.0;
}

double gravityMagnitude(SimState const& state, PhysicsStepContext const& context) {
    const auto dt = stepDt(state, context);
    if (dt <= 0.0) return 0.0;
    if (context.calibration.neutralSamples > 0) {
        return std::abs(context.calibration.neutralAccelerationY) * dt;
    }
    return std::abs(state.rawGravity * state.gravityModifier) * dt;
}

double pressVelocity(SimState const& state, PhysicsStepContext const& context) {
    if (std::abs(state.jumpVelocity) > 0.0001) return std::abs(state.jumpVelocity);
    if (context.calibration.pressSamples > 0) {
        return std::abs(context.calibration.pressVelocityY);
    }
    return 0.0;
}

double gravityDirectedDelta(
    SimState const& state,
    PhysicsStepContext const& context,
    bool upsideDown
) {
    const double magnitude = gravityMagnitude(state, context);
    return upsideDown ? magnitude : -magnitude;
}

void integratePosition(SimState& state, double dt) {
    state.x += state.vx * dt;
    state.y += state.vy * dt * state.verticalPositionScale;
}

} // namespace

void CubePhysicsModel::step(
    SimState& state,
    bool desiredHold,
    bool pressEdge,
    bool,
    PhysicsStepContext const& context
) const {
    const double dt = stepDt(state, context);
    if (pressEdge && state.grounded) {
        const double jump = pressVelocity(state, context);
        state.vy = state.upsideDown ? -jump : jump;
        state.grounded = false;
    }

    state.vy += gravityDirectedDelta(state, context, state.upsideDown);
    if (desiredHold && context.calibration.holdSamples > 0 && dt > 0.0) {
        const double measured = context.calibration.holdAccelerationY * dt;
        const double gravity = gravityDirectedDelta(state, context, state.upsideDown);
        state.vy += measured - gravity;
    }
    integratePosition(state, dt);
}

void ShipPhysicsModel::step(
    SimState& state,
    bool desiredHold,
    bool,
    bool,
    PhysicsStepContext const& context
) const {
    const double dt = stepDt(state, context);
    double delta = gravityDirectedDelta(state, context, state.upsideDown);
    if (desiredHold) {
        if (context.calibration.holdSamples > 0 && dt > 0.0) {
            delta = context.calibration.holdAccelerationY * dt;
        } else {
            delta = -delta;
        }
    } else if (context.calibration.releaseSamples > 0 && dt > 0.0) {
        delta = context.calibration.releaseAccelerationY * dt;
    }
    state.vy += delta;
    integratePosition(state, dt);
}

void BallPhysicsModel::step(
    SimState& state,
    bool,
    bool pressEdge,
    bool,
    PhysicsStepContext const& context
) const {
    const double dt = stepDt(state, context);
    if (pressEdge) {
        state.upsideDown = !state.upsideDown;
        state.grounded = false;
    }
    state.vy += gravityDirectedDelta(state, context, state.upsideDown);
    integratePosition(state, dt);
}

void UfoPhysicsModel::step(
    SimState& state,
    bool,
    bool pressEdge,
    bool,
    PhysicsStepContext const& context
) const {
    const double dt = stepDt(state, context);
    if (pressEdge) {
        const double jump = pressVelocity(state, context);
        state.vy = state.upsideDown ? -jump : jump;
        state.grounded = false;
    }
    state.vy += gravityDirectedDelta(state, context, state.upsideDown);
    integratePosition(state, dt);
}

void WavePhysicsModel::step(
    SimState& state,
    bool desiredHold,
    bool,
    bool,
    PhysicsStepContext const& context
) const {
    const double dt = stepDt(state, context);
    const double diagonalSpeed = std::max(std::abs(state.vx), 0.001);
    double sign = desiredHold ? 1.0 : -1.0;
    if (state.upsideDown) sign = -sign;

    if (desiredHold && context.calibration.holdSamples > 0) {
        state.vy = context.calibration.averageHoldVelocityY;
    } else if (!desiredHold
        && (context.calibration.releaseSamples + context.calibration.neutralSamples) > 0) {
        state.vy = context.calibration.averageReleaseVelocityY;
    } else {
        state.vy = sign * diagonalSpeed;
    }
    integratePosition(state, dt);
}

void RobotPhysicsModel::step(
    SimState& state,
    bool desiredHold,
    bool pressEdge,
    bool,
    PhysicsStepContext const& context
) const {
    const double dt = stepDt(state, context);
    if (pressEdge && state.grounded) {
        const double jump = pressVelocity(state, context) * 0.5;
        state.vy = state.upsideDown ? -jump : jump;
        state.grounded = false;
    }

    if (desiredHold && context.calibration.holdSamples > 0 && dt > 0.0) {
        state.vy += context.calibration.holdAccelerationY * dt;
    } else {
        state.vy += gravityDirectedDelta(state, context, state.upsideDown);
    }
    integratePosition(state, dt);
}

void SpiderPhysicsModel::step(
    SimState& state,
    bool,
    bool pressEdge,
    bool,
    PhysicsStepContext const& context
) const {
    const double dt = stepDt(state, context);
    if (pressEdge) {
        state.upsideDown = !state.upsideDown;
        state.grounded = false;
        state.vy = 0.0;
    }
    state.vy += gravityDirectedDelta(state, context, state.upsideDown);
    integratePosition(state, dt);
}

void SwingPhysicsModel::step(
    SimState& state,
    bool,
    bool pressEdge,
    bool,
    PhysicsStepContext const& context
) const {
    const double dt = stepDt(state, context);
    if (pressEdge) {
        state.upsideDown = !state.upsideDown;
    }
    state.vy += gravityDirectedDelta(state, context, state.upsideDown);
    integratePosition(state, dt);
}

ModePhysicsModel const& ModePhysicsRegistry::modelFor(core::GameMode mode) const {
    switch (mode) {
        case core::GameMode::Cube: return m_cube;
        case core::GameMode::Ship: return m_ship;
        case core::GameMode::Ball: return m_ball;
        case core::GameMode::Ufo: return m_ufo;
        case core::GameMode::Wave: return m_wave;
        case core::GameMode::Robot: return m_robot;
        case core::GameMode::Spider: return m_spider;
        case core::GameMode::Swing: return m_swing;
        case core::GameMode::Unknown: return m_cube;
    }
    return m_cube;
}

} // namespace autobot::solver
