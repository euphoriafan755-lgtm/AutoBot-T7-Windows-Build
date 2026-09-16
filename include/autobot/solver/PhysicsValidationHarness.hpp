#pragma once

#include "autobot/core/GameSnapshot.hpp"
#include "autobot/control/InputController.hpp"
#include "autobot/solver/SolverTypes.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace autobot::solver {

struct ModeCalibration {
    // Real wall/game time between solver observations (seconds).
    double sampleDt = 0.0;

    // Conversion learned from real dx versus GD raw X velocity. One solver
    // observation normally spans about one normalized GD physics tick, while
    // levelTime advances in seconds. Never multiply raw velocity by sampleDt
    // directly without this scale.
    double physicsTicksPerSecond = 0.0;
    double observedWorldVelocityX = 0.0;
    double observedWorldVelocityY = 0.0;
    double verticalPositionScale = 0.0;

    // Raw-velocity deltas per normalized physics tick.
    double neutralAccelerationY = 0.0;
    double holdAccelerationY = 0.0;
    double pressVelocityY = 0.0;
    double releaseAccelerationY = 0.0;

    double averageHoldVelocityY = 0.0;
    double averageReleaseVelocityY = 0.0;
    double averageVelocityX = 0.0;

    std::size_t timingSamples = 0;
    std::size_t horizontalScaleSamples = 0;
    std::size_t verticalScaleSamples = 0;
    std::size_t neutralSamples = 0;
    std::size_t holdSamples = 0;
    std::size_t pressSamples = 0;
    std::size_t releaseSamples = 0;

    double confidence = 0.0;

    [[nodiscard]] bool timingReady() const { return timingSamples >= 2 && sampleDt > 0.0; }
    [[nodiscard]] bool horizontalReady() const {
        return horizontalScaleSamples >= 2 && physicsTicksPerSecond > 0.0;
    }
    [[nodiscard]] bool neutralReady() const { return neutralSamples >= 2; }
    [[nodiscard]] bool inputReady() const {
        return pressSamples >= 1 || holdSamples >= 2 || releaseSamples >= 2;
    }
    [[nodiscard]] double normalizedStepDt() const {
        return sampleDt > 0.0 && physicsTicksPerSecond > 0.0
            ? sampleDt * physicsTicksPerSecond
            : 0.0;
    }
    [[nodiscard]] double yPositionScale() const {
        // PlayerObject::update uses a 0.9 vertical dt scale in the modeled
        // classic physics path. Runtime observations replace this default as
        // soon as clean airborne samples are available.
        return verticalScaleSamples > 0 ? verticalPositionScale : 0.9;
    }
};

struct PhysicsValidationStats {
    std::size_t samples = 0;
    std::size_t rejectedSamples = 0;
    ModelError lastError{};
    double errorEma = 0.0;
};

class PhysicsValidationHarness final {
public:
    void reset();

    void observe(
        core::GameSnapshot const& current,
        control::InputAction previousAction,
        bool previousDesiredHold
    );

    void recordPrediction(
        SimState const& predicted,
        core::GameSnapshot const& actual
    );

    [[nodiscard]] ModeCalibration const& calibration(core::GameMode mode) const;
    [[nodiscard]] PhysicsValidationStats const& stats() const { return m_stats; }
    [[nodiscard]] bool modeReady(core::GameMode mode) const;

private:
    static std::size_t modeIndex(core::GameMode mode);
    static void ema(double& target, double sample, double alpha);
    static void recomputeConfidence(ModeCalibration& calibration);

    bool m_hasPrevious = false;
    core::GameSnapshot m_previous{};
    std::array<ModeCalibration, 9> m_calibrations{};
    PhysicsValidationStats m_stats{};
};

} // namespace autobot::solver
