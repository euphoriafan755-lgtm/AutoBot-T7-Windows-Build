#include "autobot/core/GameSnapshot.hpp"
#include "autobot/solver/ActionGenerator.hpp"
#include "autobot/solver/PhysicsModels.hpp"
#include "autobot/solver/TrajectoryScorer.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

using autobot::core::GameMode;
using autobot::solver::ActionCandidate;
using autobot::solver::ModeActionGenerator;
using autobot::solver::ModeCalibration;
using autobot::solver::ModePhysicsRegistry;
using autobot::solver::PhysicsStepContext;
using autobot::solver::SimState;
using autobot::solver::TrajectoryClass;
using autobot::solver::TrajectoryResult;
using autobot::solver::TrajectoryScorer;

namespace {

bool sameCandidate(ActionCandidate const& a, ActionCandidate const& b) {
    if (a.label != b.label || a.segments.size() != b.segments.size()) return false;
    for (std::size_t i = 0; i < a.segments.size(); ++i) {
        if (a.segments[i].hold != b.segments[i].hold) return false;
        if (a.segments[i].ticks != b.segments[i].ticks) return false;
    }
    return true;
}

void verifyMode(GameMode mode) {
    autobot::core::GameSnapshot snapshot{};
    snapshot.valid = true;
    snapshot.player.mode = mode;
    snapshot.player.velocityX = 300.0;

    ModeActionGenerator generator{};
    auto first = generator.generate(snapshot, 64);
    auto second = generator.generate(snapshot, 64);

    assert(!first.empty());
    assert(first.size() == second.size());
    for (std::size_t i = 0; i < first.size(); ++i) {
        assert(!first[i].label.empty());
        assert(first[i].label != "SAFE STOP");
        assert(!first[i].segments.empty());
        for (auto const& segment : first[i].segments) assert(segment.ticks > 0);
        assert(sameCandidate(first[i], second[i]));
    }

    ModePhysicsRegistry registry{};
    auto const& model = registry.modelFor(mode);
    assert(model.mode() == mode);

    SimState state{};
    state.x = 10.0;
    state.y = 20.0;
    state.vx = 300.0;
    state.vy = 0.0;
    state.mode = mode;
    state.grounded = true;
    state.sampleDt = 1.0 / 60.0;
    state.rawGravity = -0.8;
    state.gravityModifier = 1.0;
    state.jumpAcceleration = 12.0;

    ModeCalibration calibration{};
    calibration.sampleDt = state.sampleDt;
    calibration.timingSamples = 8;
    calibration.neutralAccelerationY = -48.0;
    calibration.neutralSamples = 8;
    calibration.holdAccelerationY = 48.0;
    calibration.holdSamples = 8;
    calibration.releaseAccelerationY = -48.0;
    calibration.releaseSamples = 8;
    calibration.pressDeltaVelocityY = 12.0;
    calibration.pressSamples = 2;
    calibration.averageHoldVelocityY = 300.0;
    calibration.averageReleaseVelocityY = -300.0;

    PhysicsStepContext context{calibration, calibration.sampleDt};
    model.step(state, true, true, false, context);
    assert(std::isfinite(state.x));
    assert(std::isfinite(state.y));
    assert(std::isfinite(state.vx));
    assert(std::isfinite(state.vy));
    assert(state.mode == mode);
}

} // namespace

int main() {
    constexpr std::array<GameMode, 8> modes{{
        GameMode::Cube,
        GameMode::Ship,
        GameMode::Ball,
        GameMode::Ufo,
        GameMode::Wave,
        GameMode::Robot,
        GameMode::Spider,
        GameMode::Swing,
    }};

    for (auto mode : modes) verifyMode(mode);

    TrajectoryScorer scorer{};

    TrajectoryResult collision{};
    collision.classification = TrajectoryClass::Collision;
    collision.fatalCollision = true;
    collision.confidence = 1.0;
    collision.minimumClearance = 100.0;
    const double collisionScore = scorer.score(collision);
    assert(std::isfinite(collisionScore));
    assert(collisionScore <= -1000000.0);

    TrajectoryResult safe{};
    safe.classification = TrajectoryClass::Safe;
    safe.progress = 100.0;
    safe.minimumClearance = 40.0;
    safe.confidence = 0.8;
    safe.candidate.label = "NO PRESS";
    safe.candidate.segments.push_back({false, 10});
    const double safeScore = scorer.score(safe);
    assert(std::isfinite(safeScore));
    assert(safeScore > collisionScore);

    std::cout << "GENERAL_SOLVER_INVARIANTS=PASS\n";
    return 0;
}
