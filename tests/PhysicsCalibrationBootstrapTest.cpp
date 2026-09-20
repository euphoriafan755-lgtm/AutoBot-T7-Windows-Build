#include "autobot/solver/RealtimePlanner.hpp"
#include "autobot/world/CollisionWorld.hpp"

#include <cassert>
#include <iostream>
#include <string>

using namespace autobot;

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

    std::cout
        << "PHYSICS_CALIBRATION_BOOTSTRAP_TEST=PASS "
        << "model=PROVISIONAL control_ready=YES no_calibration_deadlock=YES\n";
    return 0;
}
