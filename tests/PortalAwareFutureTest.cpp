#include "autobot/core/GameSnapshot.hpp"
#include "autobot/solver/AdaptiveSearchBudget.hpp"
#include "autobot/solver/GeneralWorldModel.hpp"
#include "autobot/solver/GlobalPlanner.hpp"
#include "autobot/solver/LocalWorldView.hpp"
#include "autobot/solver/PhysicsValidationHarness.hpp"
#include "autobot/solver/PortalTransition.hpp"
#include "autobot/solver/TrajectorySimulator.hpp"
#include "autobot/world/CollisionWorld.hpp"

#include <cassert>
#include <iostream>

using namespace autobot;

namespace autobot::solver {
PortalEffect PortalTransitionResolver::resolve(world::CollisionPrimitive const& primitive) {
    if (primitive.objectID == 1001) return PortalEffect::ModeShip;
    if (primitive.objectID == 1002) return PortalEffect::ModeWave;
    return PortalEffect::None;
}

PortalApplication PortalTransitionResolver::apply(
    world::CollisionPrimitive const& primitive,
    SimState& state
) {
    PortalApplication result{};
    result.effect = resolve(primitive);
    if (result.effect == PortalEffect::ModeShip) {
        result.modeChanged = state.mode != core::GameMode::Ship;
        result.stateChanged = true;
        state.mode = core::GameMode::Ship;
    } else if (result.effect == PortalEffect::ModeWave) {
        result.modeChanged = state.mode != core::GameMode::Wave;
        result.stateChanged = true;
        state.mode = core::GameMode::Wave;
    }
    return result;
}
} // namespace autobot::solver

namespace {
world::WorldObject portal(float x, int id) {
    world::WorldObject o{};
    o.objectID = id;
    o.uniqueID = id * 10;
    o.rawGameObjectType = 0;
    o.type = world::GameplayObjectType::Portal;
    o.v01Support = world::V01Support::Supported;
    o.x = x + 15.0f;
    o.y = 100.0f;
    o.objectRect = {x, 40.0f, 30.0f, 120.0f};
    o.enabled = true;
    return o;
}

core::GameSnapshot snap(std::uint64_t tick, double time, double x) {
    core::GameSnapshot s{};
    s.valid = true;
    s.gameTick = tick;
    s.solverSampleID = tick;
    s.levelTime = time;
    s.player.x = x;
    s.player.y = 100.0;
    s.player.velocityX = 5.193;
    s.player.velocityY = 0.0;
    s.player.gravity = 0.0;
    s.player.gravityModifier = 1.0;
    s.player.mode = core::GameMode::Cube;
    s.player.objectBoundsWidth = 20.0;
    s.player.objectBoundsHeight = 20.0;
    return s;
}
} // namespace

int main() {
    world::StaticWorld source{};
    source.parsed = true;
    source.objects = {portal(45.0f, 1001), portal(105.0f, 1002)};
    source.portals = 2;
    world::CollisionWorld collision{};
    assert(collision.build(source, 0.0));

    constexpr double dt = 1.0 / 60.0;
    solver::PhysicsValidationHarness validation{};
    validation.observe(snap(1, 0.0, -10.386), control::InputAction::NoPress, false);
    validation.observe(snap(2, dt, -5.193), control::InputAction::NoPress, false);
    validation.observe(snap(3, dt * 2.0, 0.0), control::InputAction::NoPress, false);
    assert(validation.calibration(core::GameMode::Cube).normalizedStepDt() > 0.9);

    auto current = snap(3, dt * 2.0, 0.0);
    solver::LocalWorldViewBuilder localBuilder{};
    auto local = localBuilder.build(current, collision);
    assert(local.complete);
    assert(local.portals.size() == 2);

    solver::GeneralWorldModelBuilder generalBuilder{};
    const auto general = generalBuilder.build(current, collision, 600.0, 400.0);
    assert(general.complete);
    solver::ModelError modelError{};
    const auto budget = solver::AdaptiveSearchBudget::choose(current, local, general, modelError);
    solver::GlobalPlanner globalPlanner{};
    const auto route = globalPlanner.plan(current, general, collision, budget);
    assert(route.valid);
    assert(route.nextPortalPrimitive != world::kInvalidPrimitiveIndex);

    solver::ActionCandidate noPress{};
    noPress.label = "NO PRESS";
    noPress.segments.push_back({false, 200});

    solver::TrajectorySimulator simulator{};

    // A portal crossing by itself must NOT make a short future conclusive.
    auto shortFuture = simulator.simulate(
        current, collision, local, validation, noPress, 20, false, 300.0
    );
    assert(shortFuture.portalCrossed);
    assert(!shortFuture.horizonConclusive);
    assert(shortFuture.classification == solver::TrajectoryClass::HorizonInconclusive);

    // With enough lookahead the same future crosses both mode transitions and
    // only becomes conclusive after reaching the requested forward objective.
    auto longFuture = simulator.simulate(
        current, collision, local, validation, noPress, 80, false, 300.0
    );
    assert(longFuture.portalCrossed);
    assert(longFuture.modeChanged);
    assert(longFuture.finalMode == core::GameMode::Wave);
    assert(longFuture.horizonConclusive);
    assert(!longFuture.fatalCollision);

    std::cout << "PORTAL_AWARE_FUTURE_TEST=PASS finalMode="
              << core::toString(longFuture.finalMode)
              << " progress=" << longFuture.progress << "\n";
    std::cout << "GLOBAL_PLAN_PORTAL_CHAIN_TEST=PASS nextPortal="
              << route.nextPortalPrimitive << "\n";
    return 0;
}
