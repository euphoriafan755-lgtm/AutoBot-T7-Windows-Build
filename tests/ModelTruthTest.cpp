#include "autobot/control/AutonomousTestDriver.hpp"
#include "autobot/core/GameSnapshot.hpp"
#include "autobot/solver/LocalWorldView.hpp"
#include "autobot/solver/PhysicsValidationHarness.hpp"
#include "autobot/solver/PortalTransition.hpp"
#include "autobot/solver/RealtimePlanner.hpp"
#include "autobot/solver/TrajectoryScorer.hpp"
#include "autobot/solver/TrajectorySimulator.hpp"
#include "autobot/world/CollisionWorld.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

using namespace autobot;

namespace autobot::solver {
PortalEffect PortalTransitionResolver::resolve(world::CollisionPrimitive const&) {
    return PortalEffect::None;
}
PortalApplication PortalTransitionResolver::apply(world::CollisionPrimitive const&, SimState&) {
    return {};
}
} // namespace autobot::solver

namespace {

world::WorldObject makeObject(
    world::GameplayObjectType type,
    float x,
    float y,
    float width,
    float height,
    int id
) {
    world::WorldObject object{};
    object.objectID = id;
    object.uniqueID = id * 100;
    object.rawGameObjectType = type == world::GameplayObjectType::Hazard ? 2 : 0;
    object.type = type;
    object.v01Support = world::V01Support::Supported;
    object.x = x + width * 0.5f;
    object.y = y + height * 0.5f;
    object.objectRect = {x, y, width, height};
    object.enabled = true;
    return object;
}

core::GameSnapshot snapshot(
    std::uint64_t tick,
    double time,
    double x,
    double rawVx
) {
    core::GameSnapshot s{};
    s.valid = true;
    s.gameTick = tick;
    s.solverSampleID = tick;
    s.levelTime = time;
    s.player.x = x;
    s.player.y = 15.0;
    s.player.velocityX = rawVx;
    s.player.velocityY = 0.0;
    s.player.gravity = 0.958199;
    s.player.gravityModifier = 1.0;
    s.player.jumpVelocity = 11.180032;
    s.player.mode = core::GameMode::Cube;
    s.player.grounded = true;
    s.player.objectBoundsWidth = 30.0;
    s.player.objectBoundsHeight = 30.0;
    return s;
}

solver::TrajectoryResult const* findTrajectory(
    solver::PlanDecision const& plan,
    std::string const& label
) {
    for (auto const& trajectory : plan.trajectories) {
        if (trajectory.candidate.label == label) return &trajectory;
    }
    return nullptr;
}

} // namespace

int main() {
    world::StaticWorld source{};
    source.parsed = true;
    source.objects = {
        makeObject(world::GameplayObjectType::Solid, -100.0f, -30.0f, 600.0f, 30.0f, 1),
        makeObject(world::GameplayObjectType::Hazard, 60.0f, 0.0f, 30.0f, 20.0f, 8),
    };
    source.solids = 1;
    source.hazards = 1;

    world::CollisionWorld collisionWorld{};
    assert(collisionWorld.build(source, 0.0));

    constexpr double realDt = 1.0 / 60.0;
    constexpr double rawClassicVx = 0.9 * 5.770002;
    constexpr double realDxPerSample = rawClassicVx;

    solver::PhysicsValidationHarness validation{};
    auto s0 = snapshot(0, 0.0, -2.0 * realDxPerSample, rawClassicVx);
    auto s1 = snapshot(1, realDt, -1.0 * realDxPerSample, rawClassicVx);
    auto s2 = snapshot(2, realDt * 2.0, 0.0, rawClassicVx);
    validation.observe(s0, control::InputAction::NoPress, false);
    validation.observe(s1, control::InputAction::NoPress, false);
    validation.observe(s2, control::InputAction::NoPress, false);

    auto const& calibration = validation.calibration(core::GameMode::Cube);
    assert(validation.modeReady(core::GameMode::Cube));
    assert(std::abs(calibration.physicsTicksPerSecond - 60.0) < 0.5);
    assert(std::abs(calibration.normalizedStepDt() - 1.0) < 0.02);

    solver::LocalWorldViewBuilder localBuilder{};
    auto local = localBuilder.build(s2, collisionWorld);
    assert(local.complete);
    assert(local.hazards.size() == 1);

    const double distancePerSample = rawClassicVx * calibration.normalizedStepDt();
    const double requiredDistance = local.hazards.front().forwardDistance
        + local.hazards.front().bounds.width
        + s2.player.objectBoundsWidth * 0.5
        + distancePerSample * 12.0;
    const auto horizon = static_cast<std::size_t>(std::ceil(requiredDistance / distancePerSample));

    solver::ActionCandidate noPress{};
    noPress.label = "NO PRESS";
    noPress.segments.push_back({false, static_cast<std::uint16_t>(horizon)});

    solver::ActionCandidate pressNow{};
    pressNow.label = "PRESS NOW";
    pressNow.segments.push_back({true, 1});
    pressNow.segments.push_back({false, static_cast<std::uint16_t>(horizon - 1)});

    solver::TrajectorySimulator simulator{};
    solver::TrajectoryScorer scorer{};

    auto noPressResult = simulator.simulate(
        s2, collisionWorld, local, validation, noPress, horizon, false, requiredDistance
    );
    auto jumpResult = simulator.simulate(
        s2, collisionWorld, local, validation, pressNow, horizon, false, requiredDistance
    );
    const double noPressScore = scorer.score(noPressResult);
    const double jumpScore = scorer.score(jumpResult);

    std::cout
        << "MODEL_TRUTH_UNITS raw_vx=" << rawClassicVx
        << " real_dt=" << calibration.sampleDt
        << " physics_tps=" << calibration.physicsTicksPerSecond
        << " sim_dt=" << calibration.normalizedStepDt()
        << "\n";
    std::cout
        << "NO_PRESS classification=" << solver::toString(noPressResult.classification)
        << " fatal=" << noPressResult.fatalCollision
        << " collision_tick=" << noPressResult.collisionTick
        << " progress=" << noPressResult.progress
        << " score=" << noPressScore
        << "\n";
    std::cout
        << "PRESS_NOW classification=" << solver::toString(jumpResult.classification)
        << " fatal=" << jumpResult.fatalCollision
        << " collision_tick=" << jumpResult.collisionTick
        << " progress=" << jumpResult.progress
        << " score=" << jumpScore
        << " collision_primitive=" << jumpResult.collisionPrimitive
        << " final_y=" << jumpResult.predictedFinalY
        << "\n";

    assert(noPressResult.horizonConclusive);
    assert(noPressResult.fatalCollision);
    assert(noPressResult.hazardCollision);
    assert(noPressResult.classification == solver::TrajectoryClass::Collision);
    assert(noPressResult.collisionTick != std::numeric_limits<std::size_t>::max());

    assert(jumpResult.horizonConclusive);
    assert(!jumpResult.fatalCollision);
    assert(jumpResult.classification == solver::TrajectoryClass::Risky
        || jumpResult.classification == solver::TrajectoryClass::Safe);
    assert(jumpScore > noPressScore);

    solver::RealtimePlanner planner{};
    const auto plan = planner.plan(s2, collisionWorld, validation, false);
    auto const* plannedNoPress = findTrajectory(plan, "NO PRESS");
    assert(plannedNoPress != nullptr);
    assert(plannedNoPress->fatalCollision);
    assert(plan.selectedTrajectory < plan.trajectories.size());
    auto const& selected = plan.trajectories[plan.selectedTrajectory];
    assert(selected.candidate.label != "NO PRESS");
    assert(!selected.fatalCollision);
    assert(selected.horizonConclusive);

    std::cout
        << "PLANNER_SELECTED=" << selected.candidate.label
        << " input=" << static_cast<int>(plan.inputAction)
        << " score=" << selected.score
        << "\n";

    // False-safe detector self-test: force an impossible real death immediately
    // after a conclusive non-fatal selected trajectory. This validates the
    // runtime invariant machinery independently from the model-truth success
    // case above.
    control::AutonomousTestDriver detector{};
    auto q0 = collisionWorld.queryAhead(static_cast<float>(s0.player.x), static_cast<float>(s0.player.y));
    auto q1 = collisionWorld.queryAhead(static_cast<float>(s1.player.x), static_cast<float>(s1.player.y));
    auto q2 = collisionWorld.queryAhead(static_cast<float>(s2.player.x), static_cast<float>(s2.player.y));
    (void)detector.decide(s0, collisionWorld, q0, true, false);
    (void)detector.decide(s1, collisionWorld, q1, true, false);
    auto detectorLive = detector.decide(s2, collisionWorld, q2, true, false);
    assert(detectorLive.plan.selectedTrajectory < detectorLive.plan.trajectories.size());
    assert(!detectorLive.plan.trajectories[detectorLive.plan.selectedTrajectory].fatalCollision);

    auto forcedDeath = snapshot(3, realDt * 3.0, realDxPerSample, rawClassicVx);
    forcedDeath.player.dead = true;
    auto q3 = collisionWorld.queryAhead(
        static_cast<float>(forcedDeath.player.x),
        static_cast<float>(forcedDeath.player.y)
    );
    auto detectorDeath = detector.decide(forcedDeath, collisionWorld, q3, true, false);
    assert(detectorDeath.deathSnapshot.valid);
    assert(detectorDeath.falseSafeDetected);
    assert(detectorDeath.falseSafeTotal == 1);
    std::cout << "FALSE_SAFE_DETECTOR_SELFTEST=PASS history="
              << detectorDeath.deathSnapshot.history.size() << "\n";

    std::cout << "MODEL_TRUTH_TESTS=PASS\n";
    return 0;
}
