#include "autobot/core/GameSnapshot.hpp"
#include "autobot/solver/LocalWorldView.hpp"
#include "autobot/solver/PhysicsValidationHarness.hpp"
#include "autobot/solver/TrajectorySimulator.hpp"
#include "autobot/world/CollisionWorld.hpp"
#include "autobot/solver/PortalTransition.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using namespace autobot;

namespace autobot::solver {
PortalEffect PortalTransitionResolver::resolve(world::CollisionPrimitive const&) { return PortalEffect::None; }
PortalApplication PortalTransitionResolver::apply(world::CollisionPrimitive const&, SimState&) { return {}; }
}


static world::WorldObject solid(float x, float y, float w, float h, int id) {
    world::WorldObject o{};
    o.objectID = id;
    o.uniqueID = id * 10;
    o.rawGameObjectType = 0;
    o.type = world::GameplayObjectType::Solid;
    o.v01Support = world::V01Support::Supported;
    o.x = x + w * .5f;
    o.y = y + h * .5f;
    o.objectRect = {x, y, w, h};
    o.enabled = true;
    return o;
}

static world::WorldObject hazard(float x, float y, float w, float h, int id) {
    auto o = solid(x, y, w, h, id);
    o.rawGameObjectType = 2;
    o.type = world::GameplayObjectType::Hazard;
    return o;
}

static core::GameSnapshot snap(std::uint64_t tick, double t, double x, double vxBinding) {
    core::GameSnapshot s{};
    s.valid = true;
    s.gameTick = tick;
    s.solverSampleID = tick;
    s.levelTime = t;
    s.player.x = x;
    s.player.y = 15.0;
    s.player.velocityX = vxBinding;
    s.player.velocityY = 0.0;
    s.player.mode = core::GameMode::Cube;
    s.player.grounded = true;
    s.player.objectBoundsWidth = 30.0;
    s.player.objectBoundsHeight = 30.0;
    return s;
}

int main() {
    world::StaticWorld worldState{};
    worldState.parsed = true;
    worldState.objects = {
        solid(-100.f, -30.f, 600.f, 30.f, 1),
        hazard(100.f, 0.f, 30.f, 30.f, 8),
    };
    worldState.solids = 1;
    worldState.hazards = 1;

    world::CollisionWorld collisionWorld{};
    assert(collisionWorld.build(worldState, 0.0));

    constexpr double dt = 1.0 / 60.0;
    constexpr double rawBindingX = 0.9 * 5.770002;
    constexpr double realDxPerSample = rawBindingX;

    solver::PhysicsValidationHarness validation{};
    auto s0 = snap(0, 0.0, -2.0 * realDxPerSample, rawBindingX);
    auto s1 = snap(1, dt, -1.0 * realDxPerSample, rawBindingX);
    auto s2 = snap(2, dt * 2.0, 0.0, rawBindingX);
    validation.observe(s0, control::InputAction::NoPress, false);
    validation.observe(s1, control::InputAction::NoPress, false);
    validation.observe(s2, control::InputAction::NoPress, false);
    assert(validation.modeReady(core::GameMode::Cube));

    solver::LocalWorldViewBuilder localBuilder{};
    auto local = localBuilder.build(s2, collisionWorld);
    assert(local.complete);
    assert(!local.hazards.empty());

    solver::ActionCandidate noPress{};
    noPress.label = "NO PRESS";
    noPress.segments.push_back({false, 144});

    solver::TrajectorySimulator simulator{};
    auto result = simulator.simulate(
        s2, collisionWorld, local, validation, noPress, 144, false, 0.0
    );

    std::cout << "binding_vx=" << rawBindingX << " dt=" << validation.calibration(core::GameMode::Cube).sampleDt
              << " predicted_progress=" << result.progress << " fatal=" << result.fatalCollision
              << " class=" << static_cast<int>(result.classification) << "\n";

    // This assertion describes the required model truth. It failed before the unit fix.
    assert(result.fatalCollision && "FALSE SAFE REPRODUCED: NO PRESS never reaches basic hazard");
    std::cout << "MODEL_TRUTH_REPRO=PASS\n";
}
