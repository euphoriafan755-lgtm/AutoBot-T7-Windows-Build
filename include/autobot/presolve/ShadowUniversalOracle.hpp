#pragma once

#include "autobot/core/GameSnapshot.hpp"
#include "autobot/presolve/UniversalSearchCore.hpp"
#include "autobot/solver/PhysicsModels.hpp"
#include "autobot/solver/PhysicsValidationHarness.hpp"
#include "autobot/world/CollisionWorld.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

namespace autobot::presolve {

// Pure solver-side oracle. It never owns or calls PlayLayer/GJBaseGameLayer.
// Search tokens are copies of plain solver state only.
class ShadowUniversalOracle final : public IUniversalStateOracle {
public:
    ShadowUniversalOracle() = default;

    void configure(
        world::CollisionWorld const* collisionWorld,
        solver::PhysicsValidationHarness const* validation,
        double completionBoundaryX,
        bool platformer
    );

    bool setLiveRoot(
        core::GameSnapshot const& snapshot,
        bool p1Holding
    );

    [[nodiscard]] bool ready() const { return m_ready; }
    [[nodiscard]] double stepDt() const;
    [[nodiscard]] std::string const& lastError() const { return m_error; }

    [[nodiscard]] UniversalObservation observe() const override;
    [[nodiscard]] std::optional<UniversalToken> capture() override;
    bool restore(UniversalToken token) override;
    [[nodiscard]] UniversalObservation step(UniversalAction action) override;
    void discard(UniversalToken token) override;
    [[nodiscard]] std::optional<UniversalCanonicalState> canonicalState(UniversalToken token) const override;
    [[nodiscard]] std::uint64_t decisionEpoch() const override;

    [[nodiscard]] std::size_t simulatedSteps() const { return m_simulatedSteps; }
    [[nodiscard]] static constexpr std::size_t internalGdUpdateCalls() { return 0; }
    [[nodiscard]] static constexpr std::size_t checkpointCalls() { return 0; }

private:
    struct State {
        core::GameSnapshot snapshot{};
        bool holding = false;
        bool complete = false;
        std::uint64_t shadowTick = 0;
    };

    [[nodiscard]] solver::SimState makeSimState() const;
    void applySimState(solver::SimState const& state);
    void resolveBasicCollision(
        solver::SimState const& previous,
        solver::SimState& current
    ) const;
    [[nodiscard]] double progressFor(double x) const;
    [[nodiscard]] UniversalCanonicalState canonical(State const& state) const;

    world::CollisionWorld const* m_collisionWorld = nullptr;
    solver::PhysicsValidationHarness const* m_validation = nullptr;
    solver::ModePhysicsRegistry m_physics{};
    double m_completionBoundaryX = 0.0;
    double m_rootX = 0.0;
    double m_rootProgress = 0.0;
    double m_direction = 1.0;
    bool m_platformer = false;
    bool m_ready = false;
    State m_state{};
    UniversalToken m_nextToken = 0;
    std::unordered_map<UniversalToken, State> m_snapshots;
    std::size_t m_simulatedSteps = 0;
    std::string m_error;
};

} // namespace autobot::presolve
