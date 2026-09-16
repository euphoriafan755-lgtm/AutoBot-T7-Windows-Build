#pragma once

#include "autobot/core/GameSnapshot.hpp"
#include "autobot/control/InputController.hpp"
#include "autobot/solver/SolverTypes.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace autobot::solver {

struct ModeCalibration {
    double sampleDt = 0.0;
    double neutralAccelerationY = 0.0;
    double holdAccelerationY = 0.0;
    double pressDeltaVelocityY = 0.0;
    double releaseAccelerationY = 0.0;

    double averageHoldVelocityY = 0.0;
    double averageReleaseVelocityY = 0.0;
    double averageVelocityX = 0.0;

    std::size_t timingSamples = 0;
    std::size_t neutralSamples = 0;
    std::size_t holdSamples = 0;
    std::size_t pressSamples = 0;
    std::size_t releaseSamples = 0;

    double confidence = 0.0;

    [[nodiscard]] bool timingReady() const { return timingSamples >= 2 && sampleDt > 0.0; }
    [[nodiscard]] bool neutralReady() const { return neutralSamples >= 2; }
    [[nodiscard]] bool inputReady() const {
        return pressSamples >= 1 || holdSamples >= 2 || releaseSamples >= 2;
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
