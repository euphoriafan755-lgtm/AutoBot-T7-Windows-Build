#pragma once

#include "autobot/solver/PhysicsValidationHarness.hpp"
#include "autobot/solver/SolverTypes.hpp"

namespace autobot::solver {

struct PhysicsStepContext {
    ModeCalibration calibration{};
    double dt = 0.0;
};

class ModePhysicsModel {
public:
    virtual ~ModePhysicsModel() = default;
    [[nodiscard]] virtual core::GameMode mode() const = 0;
    virtual void step(
        SimState& state,
        bool desiredHold,
        bool pressEdge,
        bool releaseEdge,
        PhysicsStepContext const& context
    ) const = 0;
};

class CubePhysicsModel final : public ModePhysicsModel {
public:
    [[nodiscard]] core::GameMode mode() const override { return core::GameMode::Cube; }
    void step(SimState&, bool, bool, bool, PhysicsStepContext const&) const override;
};

class ShipPhysicsModel final : public ModePhysicsModel {
public:
    [[nodiscard]] core::GameMode mode() const override { return core::GameMode::Ship; }
    void step(SimState&, bool, bool, bool, PhysicsStepContext const&) const override;
};

class BallPhysicsModel final : public ModePhysicsModel {
public:
    [[nodiscard]] core::GameMode mode() const override { return core::GameMode::Ball; }
    void step(SimState&, bool, bool, bool, PhysicsStepContext const&) const override;
};

class UfoPhysicsModel final : public ModePhysicsModel {
public:
    [[nodiscard]] core::GameMode mode() const override { return core::GameMode::Ufo; }
    void step(SimState&, bool, bool, bool, PhysicsStepContext const&) const override;
};

class WavePhysicsModel final : public ModePhysicsModel {
public:
    [[nodiscard]] core::GameMode mode() const override { return core::GameMode::Wave; }
    void step(SimState&, bool, bool, bool, PhysicsStepContext const&) const override;
};

class RobotPhysicsModel final : public ModePhysicsModel {
public:
    [[nodiscard]] core::GameMode mode() const override { return core::GameMode::Robot; }
    void step(SimState&, bool, bool, bool, PhysicsStepContext const&) const override;
};

class SpiderPhysicsModel final : public ModePhysicsModel {
public:
    [[nodiscard]] core::GameMode mode() const override { return core::GameMode::Spider; }
    void step(SimState&, bool, bool, bool, PhysicsStepContext const&) const override;
};

class SwingPhysicsModel final : public ModePhysicsModel {
public:
    [[nodiscard]] core::GameMode mode() const override { return core::GameMode::Swing; }
    void step(SimState&, bool, bool, bool, PhysicsStepContext const&) const override;
};

class ModePhysicsRegistry final {
public:
    [[nodiscard]] ModePhysicsModel const& modelFor(core::GameMode mode) const;

private:
    CubePhysicsModel m_cube{};
    ShipPhysicsModel m_ship{};
    BallPhysicsModel m_ball{};
    UfoPhysicsModel m_ufo{};
    WavePhysicsModel m_wave{};
    RobotPhysicsModel m_robot{};
    SpiderPhysicsModel m_spider{};
    SwingPhysicsModel m_swing{};
};

} // namespace autobot::solver
