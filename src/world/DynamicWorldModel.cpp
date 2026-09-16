#include "autobot/world/DynamicWorldModel.hpp"

#include <algorithm>
#include <cmath>

namespace autobot::world {
namespace {

bool finiteRect(WorldRect const& r) {
    return std::isfinite(r.x) && std::isfinite(r.y)
        && std::isfinite(r.width) && std::isfinite(r.height);
}

double maxAbsDelta(WorldRect const& a, WorldRect const& b) {
    return std::max({
        std::abs(static_cast<double>(a.x - b.x)),
        std::abs(static_cast<double>(a.y - b.y)),
        std::abs(static_cast<double>(a.width - b.width)),
        std::abs(static_cast<double>(a.height - b.height)),
    });
}

} // namespace

void DynamicWorldModel::reset() {
    m_tracks.clear();
    m_trackedCount = 0;
    m_motionConfirmedCount = 0;
}

void DynamicWorldModel::observe(
    CollisionWorld const& collisionWorld,
    std::uint64_t solverSampleID
) {
    auto const& primitives = collisionWorld.primitives();
    if (m_tracks.size() != primitives.size()) {
        m_tracks.assign(primitives.size(), {});
    }

    m_trackedCount = 0;
    m_motionConfirmedCount = 0;

    for (std::size_t i = 0; i < primitives.size(); ++i) {
        auto const& primitive = primitives[i];
        if (!primitive.potentiallyDynamic && !primitive.observedDynamic) continue;
        ++m_trackedCount;

        auto& track = m_tracks[i];
        auto const current = primitive.broadphaseBounds;
        if (!finiteRect(current)) {
            track.uncertain = true;
            track.confidence = 0.0;
            continue;
        }

        if (!track.initialized) {
            track.initialized = true;
            track.lastSampleID = solverSampleID;
            track.bounds = current;
            track.enabled = primitive.enabled;
            track.observations = 1;
            track.confidence = 0.15;
            continue;
        }

        if (solverSampleID <= track.lastSampleID) {
            if (track.motionConfirmed) ++m_motionConfirmedCount;
            continue;
        }

        const auto sampleDelta = static_cast<double>(solverSampleID - track.lastSampleID);
        const double dx = static_cast<double>(current.x - track.bounds.x) / sampleDelta;
        const double dy = static_cast<double>(current.y - track.bounds.y) / sampleDelta;
        const double dw = static_cast<double>(current.width - track.bounds.width) / sampleDelta;
        const double dh = static_cast<double>(current.height - track.bounds.height) / sampleDelta;

        // Large one-sample jumps can be legitimate teleports/spawns, but a
        // constant-velocity extrapolation is not trustworthy across them.
        const double magnitude = maxAbsDelta(current, track.bounds) / sampleDelta;
        const bool discontinuity = magnitude > 240.0 || primitive.enabled != track.enabled;
        if (discontinuity) {
            track.dxPerSample = 0.0;
            track.dyPerSample = 0.0;
            track.dwPerSample = 0.0;
            track.dhPerSample = 0.0;
            track.uncertain = true;
            track.confidence = std::min(track.confidence, 0.25);
        } else {
            constexpr double kAlpha = 0.35;
            if (track.observations <= 1) {
                track.dxPerSample = dx;
                track.dyPerSample = dy;
                track.dwPerSample = dw;
                track.dhPerSample = dh;
            } else {
                track.dxPerSample = track.dxPerSample * (1.0 - kAlpha) + dx * kAlpha;
                track.dyPerSample = track.dyPerSample * (1.0 - kAlpha) + dy * kAlpha;
                track.dwPerSample = track.dwPerSample * (1.0 - kAlpha) + dw * kAlpha;
                track.dhPerSample = track.dhPerSample * (1.0 - kAlpha) + dh * kAlpha;
            }

            const double motion = std::max({
                std::abs(track.dxPerSample),
                std::abs(track.dyPerSample),
                std::abs(track.dwPerSample),
                std::abs(track.dhPerSample),
            });
            track.motionConfirmed = track.motionConfirmed || motion > 0.001;
            track.uncertain = false;
            track.confidence = std::clamp(
                0.2 + static_cast<double>(std::min<std::size_t>(track.observations, 8)) * 0.1,
                0.0,
                0.95
            );
        }

        ++track.observations;
        track.bounds = current;
        track.enabled = primitive.enabled;
        track.lastSampleID = solverSampleID;
        if (track.motionConfirmed) ++m_motionConfirmedCount;
    }
}

DynamicPrimitivePrediction DynamicWorldModel::predict(
    CollisionWorld const& collisionWorld,
    std::size_t primitiveIndex,
    std::size_t samplesAhead
) const {
    DynamicPrimitivePrediction result{};
    auto const& primitives = collisionWorld.primitives();
    if (primitiveIndex >= primitives.size()) return result;

    auto const& primitive = primitives[primitiveIndex];
    result.bounds = primitive.broadphaseBounds;
    result.available = true;
    result.dynamic = primitive.potentiallyDynamic || primitive.observedDynamic;

    if (!result.dynamic || primitiveIndex >= m_tracks.size()) {
        result.confidence = result.dynamic ? 0.0 : 1.0;
        result.uncertain = result.dynamic;
        return result;
    }

    auto const& track = m_tracks[primitiveIndex];
    if (!track.initialized || track.observations < 2 || !track.motionConfirmed) {
        result.confidence = track.initialized ? track.confidence : 0.0;
        result.uncertain = true;
        return result;
    }

    // Constant-velocity prediction is deliberately bounded. Beyond this
    // window the caller should rely on trigger semantics/global replanning.
    constexpr std::size_t kMaxExtrapolationSamples = 120;
    const auto clampedSamples = std::min(samplesAhead, kMaxExtrapolationSamples);
    const double t = static_cast<double>(clampedSamples);
    result.bounds.x = static_cast<float>(track.bounds.x + track.dxPerSample * t);
    result.bounds.y = static_cast<float>(track.bounds.y + track.dyPerSample * t);
    result.bounds.width = static_cast<float>(track.bounds.width + track.dwPerSample * t);
    result.bounds.height = static_cast<float>(track.bounds.height + track.dhPerSample * t);
    result.confidence = track.confidence;
    result.uncertain = track.uncertain || samplesAhead > kMaxExtrapolationSamples;
    return result;
}

} // namespace autobot::world
