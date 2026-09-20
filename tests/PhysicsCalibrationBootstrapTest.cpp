#include "autobot/solver/RealtimePlanner.hpp"
#include "autobot/solver/PhysicsBootstrap.hpp"
#include "autobot/world/CollisionWorld.hpp"

#include "autobot/solver/PortalTransition.hpp"

#include <cassert>
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

int main() {
    world::StaticWorld source{};
    source.parsed = true;

    world::WorldObject farSolid{};
    farSolid.objectID = 1;
    farSolid.uniqueID = 1;
    farSolid.rawGameObjectType = 0;
    farSolid.type = world::GameplayObjectType::Solid;
    farSolid.v01Support = world::V01Support::Supported;
    farSolid.runtimeTypeKnown = true;
    farSolid.x = 10000.0f;
    farSolid.y = 0.0f;
    farSolid.objectRect = {9990.0f, -10.0f, 20.0f, 20.0f};
    source.objects.push_back(farSolid);
    source.solids = 1;

    world::CollisionWorld collision;
    assert(collision.build(source, 0.0));

    core::GameSnapshot snapshot{};
    snapshot.valid = true;
    snapshot.player.x = 0.0;
    snapshot.player.y = 100.0;
    snapshot.player.velocityX = 5.0;
    snapshot.player.velocityY = 0.0;
    snapshot.player.gravity = 0.8;
    snapshot.player.gravityModifier = 1.0;
    snapshot.player.jumpVelocity = 12.0;
    snapshot.player.speed = 1.0;
    snapshot.player.objectBoundsWidth = 30.0;
    snapshot.player.objectBoundsHeight = 30.0;
    snapshot.player.mode = core::GameMode::Cube;
    snapshot.player.grounded = true;

    solver::PhysicsValidationHarness validation;
    world::DynamicWorldModel dynamicWorld;
    solver::RealtimePlanner planner;

    assert(solver::physicsStatus(validation, core::GameMode::Cube)
        == solver::PhysicsModelStatus::Provisional);

    const auto plan = planner.plan(
        snapshot,
        collision,
        validation,
        false,
        dynamicWorld,
        nullptr
    );

    assert(plan.physicsReady);
    assert(plan.physicsModelStatus == solver::PhysicsModelStatus::Provisional);
    assert(plan.reason.find("WAITING FOR PHYSICS UNIT CALIBRATION") == std::string::npos);

    auto sample0 = snapshot;
    sample0.solverSampleID = 1;
    sample0.levelTime = 0.0;
    validation.observe(sample0, control::InputAction::NoPress, false);
    assert(solver::physicsStatus(validation, core::GameMode::Cube)
        == solver::PhysicsModelStatus::Provisional);

    auto sample1 = sample0;
    sample1.solverSampleID = 2;
    sample1.levelTime = 1.0 / 60.0;
    sample1.player.x += 5.0;
    validation.observe(sample1, control::InputAction::NoPress, false);
    assert(solver::physicsStatus(validation, core::GameMode::Cube)
        == solver::PhysicsModelStatus::Calibrating);

    auto sample2 = sample1;
    sample2.solverSampleID = 3;
    sample2.levelTime = 2.0 / 60.0;
    sample2.player.x += 5.0;
    validation.observe(sample2, control::InputAction::NoPress, false);
    assert(solver::physicsStatus(validation, core::GameMode::Cube)
        == solver::PhysicsModelStatus::Verified);

    std::cout
        << "PHYSICS_BOOTSTRAP_TEST=PASS "
        << "model=PROVISIONAL->CALIBRATING->VERIFIED "
        << "control_ready=YES no_calibration_deadlock=YES\n";
    return 0;
}
