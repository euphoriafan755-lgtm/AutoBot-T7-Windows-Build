#include "autobot/presolve/ShadowUniversalOracle.hpp"
#include "autobot/presolve/UniversalSearchCore.hpp"

#include <cassert>
#include <iostream>

using namespace autobot;

namespace {

world::CollisionWorld makeWorld() {
    world::StaticWorld source{};
    source.parsed = true;
    source.completionBoundaryValid = true;
    source.completionBoundaryX = 2000.0;

    world::WorldObject farSolid{};
    farSolid.objectID = 1;
    farSolid.uniqueID = 1;
    farSolid.type = world::GameplayObjectType::Solid;
    farSolid.v01Support = world::V01Support::Supported;
    farSolid.runtimeTypeKnown = true;
    farSolid.x = 10000.0f;
    farSolid.objectRect = {9990.0f, -10.0f, 20.0f, 20.0f};
    source.objects.push_back(farSolid);
    source.solids = 1;

    world::CollisionWorld collision;
    assert(collision.build(source, 0.0));
    return collision;
}

core::GameSnapshot root() {
    core::GameSnapshot s{};
    s.valid = true;
    s.player.x = 0.0;
    s.player.y = 100.0;
    s.player.velocityX = 5.0;
    s.player.gravity = 0.8;
    s.player.gravityModifier = 1.0;
    s.player.jumpVelocity = 12.0;
    s.player.objectBoundsWidth = 30.0;
    s.player.objectBoundsHeight = 30.0;
    s.player.mode = core::GameMode::Cube;
    s.player.grounded = true;
    return s;
}

} // namespace

int main() {
    auto world = makeWorld();
    solver::PhysicsValidationHarness validation;
    presolve::ShadowUniversalOracle oracle;
    oracle.configure(&world, &validation, 2000.0, false);
    assert(oracle.setLiveRoot(root(), false));

    presolve::UniversalSearchCore search;
    assert(search.begin(oracle));

    for (int i = 0; i < 20 && search.searching(); ++i) {
        search.work(oracle, 8);
    }

    const auto stats = search.stats();
    assert(stats.totalExpansions > 0);
    assert(stats.totalEngineSteps > 0);
    assert(oracle.internalGdUpdateCalls() == 0);
    assert(oracle.checkpointCalls() == 0);

    std::cout
        << "NO_INTERNAL_GD_UPDATE_DURING_SEARCH_TEST=PASS "
        << "expansions=" << stats.totalExpansions
        << " shadowSteps=" << stats.totalEngineSteps
        << " internal_GJBaseGameLayer_updates=0"
        << " internal_PlayLayer_updates=0"
        << " checkpoints=0\n";
    return 0;
}
