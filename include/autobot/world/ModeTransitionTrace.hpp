#pragma once

#include "autobot/core/GameSnapshot.hpp"
#include "autobot/ui/CollisionDebugOverlay.hpp"
#include "autobot/world/CollisionWorld.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

class PlayLayer;

namespace autobot::world {

struct ModeTransitionSample {
    std::uint64_t sampleTick = 0;
    double levelTime = 0.0;
    core::GameMode mode = core::GameMode::Unknown;
    double playerX = 0.0;
    double playerY = 0.0;
    double playerNodeX = 0.0;
    double playerNodeY = 0.0;
    double velocityX = 0.0;
    double velocityY = 0.0;
    float cameraX = 0.0f;
    float cameraY = 0.0f;
    float camera2X = 0.0f;
    float camera2Y = 0.0f;
    float objectLayerX = 0.0f;
    float objectLayerY = 0.0f;
    float objectLayerScaleX = 1.0f;
    float objectLayerScaleY = 1.0f;
    float objectLayerRotation = 0.0f;
    WorldRect queryRect{};
    SpatialQueryDebug queryDebug{};
    std::vector<std::size_t> returnedIndices;
    std::size_t fullScanExpected = 0;
    std::size_t missing = 0;
    std::size_t unexpected = 0;
    std::size_t duplicates = 0;
    std::size_t invalidIndices = 0;
    std::size_t rendererReceived = 0;
    std::size_t rendererDrawn = 0;
    std::uint64_t collisionWorldGeneration = 0;
    std::uint64_t spatialHashGeneration = 0;
};

class ModeTransitionTrace final {
public:
    void observe(
        PlayLayer* playLayer,
        core::GameSnapshot const& snapshot,
        CollisionWorld const& collisionWorld,
        CollisionQueryResult const& query,
        ui::CollisionRenderStats const& renderStats,
        std::uint64_t worldGeneration
    );

    [[nodiscard]] bool captureActive() const { return m_afterRemaining > 0; }

private:
    static void audit(ModeTransitionSample& sample, CollisionWorld const& collisionWorld);
    static void logSample(std::uint64_t transitionID, char const* phase, ModeTransitionSample const& sample);

    bool m_previousModeValid = false;
    core::GameMode m_previousMode = core::GameMode::Unknown;
    std::deque<ModeTransitionSample> m_history;
    std::size_t m_afterRemaining = 0;
    std::uint64_t m_transitionID = 0;
};

} // namespace autobot::world
