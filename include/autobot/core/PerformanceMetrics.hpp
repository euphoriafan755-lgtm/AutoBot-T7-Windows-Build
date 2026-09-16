#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <numeric>

namespace autobot::core {

struct TimingSummary {
    std::size_t samples = 0;
    double averageMs = 0.0;
    double p95Ms = 0.0;
    double p99Ms = 0.0;
};

class RollingTiming final {
public:
    static constexpr std::size_t kCapacity = 120;

    void add(double milliseconds) {
        if (milliseconds < 0.0) return;
        m_values[m_next] = milliseconds;
        m_next = (m_next + 1) % kCapacity;
        if (m_count < kCapacity) ++m_count;
    }

    [[nodiscard]] TimingSummary summary() const {
        TimingSummary result{};
        result.samples = m_count;
        if (m_count == 0) return result;

        std::array<double, kCapacity> ordered{};
        for (std::size_t i = 0; i < m_count; ++i) ordered[i] = m_values[i];
        std::sort(ordered.begin(), ordered.begin() + static_cast<std::ptrdiff_t>(m_count));

        result.averageMs = std::accumulate(
            ordered.begin(),
            ordered.begin() + static_cast<std::ptrdiff_t>(m_count),
            0.0
        ) / static_cast<double>(m_count);

        const auto percentileIndex = [&](std::size_t numerator) {
            if (m_count <= 1) return std::size_t{0};
            return std::min(
                m_count - 1,
                ((m_count - 1) * numerator) / 100
            );
        };
        result.p95Ms = ordered[percentileIndex(95)];
        result.p99Ms = ordered[percentileIndex(99)];
        return result;
    }

private:
    std::array<double, kCapacity> m_values{};
    std::size_t m_count = 0;
    std::size_t m_next = 0;
};

struct SolverPerformanceMetrics {
    RollingTiming stateRead{};
    RollingTiming worldSync{};
    RollingTiming query{};
    RollingTiming candidateGeneration{};
    RollingTiming physicsSimulation{};
    RollingTiming trajectoryScoring{};
    RollingTiming plannerTotal{};
    RollingTiming input{};
    RollingTiming debugOverlay{};
};

} // namespace autobot::core
