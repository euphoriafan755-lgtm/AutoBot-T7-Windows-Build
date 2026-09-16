#pragma once

#include "autobot/world/CollisionWorld.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace autobot::world {

struct DynamicPrimitivePrediction {
    WorldRect bounds{};
    bool available = false;
    bool dynamic = false;
    bool uncertain = false;
    double confidence = 0.0;
};

class DynamicWorldModel final {
public:
    void reset();
    void observe(CollisionWorld const& collisionWorld, std::uint64_t solverSampleID);

    [[nodiscard]] DynamicPrimitivePrediction predict(
        CollisionWorld const& collisionWorld,
        std::size_t primitiveIndex,
        std::size_t samplesAhead
    ) const;

    [[nodiscard]] std::size_t trackedCount() const { return m_trackedCount; }
    [[nodiscard]] std::size_t motionConfirmedCount() const { return m_motionConfirmedCount; }

private:
    struct Track {
        bool initialized = false;
        bool motionConfirmed = false;
        bool uncertain = false;
        bool enabled = true;
        std::uint64_t lastSampleID = 0;
        std::size_t observations = 0;
        WorldRect bounds{};
        double dxPerSample = 0.0;
        double dyPerSample = 0.0;
        double dwPerSample = 0.0;
        double dhPerSample = 0.0;
        double confidence = 0.0;
    };

    std::vector<Track> m_tracks;
    std::size_t m_trackedCount = 0;
    std::size_t m_motionConfirmedCount = 0;
};

} // namespace autobot::world
