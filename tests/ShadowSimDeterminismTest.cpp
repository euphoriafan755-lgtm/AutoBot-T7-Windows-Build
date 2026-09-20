#include "autobot/presolve/ShadowUniversalOracle.hpp"

#include <cassert>
#include <iostream>
#include <vector>

using namespace autobot;

namespace {

world::CollisionWorld makeWorld() {
    world::StaticWorld source{};
    source.parsed = true;
    source.completionBoundaryValid = true;
    source.completionBoundaryX = 1000.0;

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
    return collision;
}

core::GameSnapshot makeRoot() {
    core::GameSnapshot snapshot{};
    snapshot.valid = true;
    snapshot.solverSampleID = 1;
    snapshot.levelTime = 0.0;
    snapshot.levelProgress = 0.0f;
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
    return snapshot;
}

std::vector<presolve::UniversalObservation> simulate(
    presolve::ShadowUniversalOracle& oracle,
    std::vector<presolve::UniversalAction> const& actions
) {
    std::vector<presolve::UniversalObservation> out;
    out.reserve(actions.size());
    for (auto const action : actions) out.push_back(oracle.step(action));
    return out;
}

} // namespace

int main() {
    auto world = makeWorld();
    solver::PhysicsValidationHarness validation;
    const auto root = makeRoot();

    presolve::ShadowUniversalOracle a;
    presolve::ShadowUniversalOracle b;
    a.configure(&world, &validation, 1000.0, false);
    b.configure(&world, &validation, 1000.0, false);
    assert(a.setLiveRoot(root, false));
    assert(b.setLiveRoot(root, false));

    const std::vector<presolve::UniversalAction> actions{
        {false, false, false, false, false, false},
        {true,  false, false, false, false, false},
        {true,  false, false, false, false, false},
        {false, false, false, false, false, false},
        {false, false, false, false, false, false},
        {true,  false, false, false, false, false},
    };

    const auto resultA = simulate(a, actions);
    const auto resultB = simulate(b, actions);

    assert(resultA.size() == resultB.size());
    for (std::size_t i = 0; i < resultA.size(); ++i) {
        assert(resultA[i].valid == resultB[i].valid);
        assert(resultA[i].dead == resultB[i].dead);
        assert(resultA[i].complete == resultB[i].complete);
        assert(resultA[i].progress == resultB[i].progress);
        assert(resultA[i].fingerprint == resultB[i].fingerprint);
    }

    auto tokenA = a.capture();
    auto tokenB = b.capture();
    assert(tokenA && tokenB);
    const auto canonicalA = a.canonicalState(*tokenA);
    const auto canonicalB = b.canonicalState(*tokenB);
    assert(canonicalA && canonicalB);
    assert(canonicalA->words == canonicalB->words);
    assert(canonicalA->hash == canonicalB->hash);

    assert(a.internalGdUpdateCalls() == 0);
    assert(a.checkpointCalls() == 0);

    std::cout
        << "SHADOW_SIM_DETERMINISM_TEST=PASS "
        << "same_copied_state=YES same_actions=YES same_result=YES "
        << "internal_gd_updates=0 checkpoints=0\n";
    return 0;
}
