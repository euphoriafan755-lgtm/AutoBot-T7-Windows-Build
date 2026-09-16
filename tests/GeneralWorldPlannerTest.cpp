#include "autobot/core/GameSnapshot.hpp"
#include "autobot/solver/AdaptiveSearchBudget.hpp"
#include "autobot/solver/GeneralWorldModel.hpp"
#include "autobot/solver/GlobalPlanner.hpp"
#include "autobot/solver/LocalWorldView.hpp"
#include "autobot/world/CollisionWorld.hpp"
#include "autobot/world/DynamicWorldModel.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using namespace autobot;

namespace {

world::WorldObject object(
    world::GameplayObjectType type,
    float x,
    float y,
    float w,
    float h,
    int id,
    bool dynamic = false
) {
    world::WorldObject o{};
    o.objectID = id;
    o.uniqueID = id * 10;
    o.rawGameObjectType = 0;
    o.type = type;
    o.v01Support = world::V01Support::Supported;
    o.x = x + w * 0.5f;
    o.y = y + h * 0.5f;
    o.nodeX = o.x;
    o.nodeY = o.y;
    o.objectRect = {x, y, w, h};
    o.enabled = true;
    o.groupCount = dynamic ? 1 : 0;
    return o;
}

core::GameSnapshot snapshot() {
    core::GameSnapshot s{};
    s.valid = true;
    s.gameTick = 100;
    s.solverSampleID = 100;
    s.levelTime = 1.0;
    s.player.x = 0.0;
    s.player.y = 100.0;
    s.player.velocityX = 5.193;
    s.player.velocityY = 0.0;
    s.player.mode = core::GameMode::Cube;
    s.player.objectBoundsWidth = 30.0;
    s.player.objectBoundsHeight = 30.0;
    s.player.grounded = false;
    return s;
}

} // namespace

int main() {
    world::StaticWorld source{};
    source.parsed = true;
    source.objects = {
        object(world::GameplayObjectType::Solid, -100.0f, -40.0f, 2500.0f, 40.0f, 1),
        object(world::GameplayObjectType::Hazard, 350.0f, 70.0f, 120.0f, 120.0f, 2),
        object(world::GameplayObjectType::Hazard, 800.0f, 0.0f, 100.0f, 90.0f, 3),
        object(world::GameplayObjectType::Portal, 1050.0f, 40.0f, 30.0f, 160.0f, 4),
        object(world::GameplayObjectType::Hazard, 1250.0f, 120.0f, 80.0f, 120.0f, 5, true),
    };
    source.solids = 1;
    source.hazards = 3;
    source.portals = 1;

    world::CollisionWorld collision{};
    assert(collision.build(source, 0.0));
    const auto s = snapshot();

    solver::LocalWorldViewBuilder localBuilder{};
    auto local = localBuilder.build(s, collision);
    assert(local.complete);

    solver::GeneralWorldModelBuilder generalBuilder{};
    auto general = generalBuilder.build(s, collision, 1800.0, 900.0);
    assert(general.complete);
    assert(general.hazards == 3);
    assert(general.portals == 1);
    assert(general.dynamic >= 1);
    std::cout << "GENERAL_WORLD_MODEL_TEST=PASS events=" << general.events.size() << "\n";

    solver::ModelError error{};
    auto budget = solver::AdaptiveSearchBudget::choose(s, local, general, error);
    assert(budget.globalLookaheadX >= 1400.0);
    assert(budget.globalSlices >= 10);
    assert(budget.horizonMax >= 160);
    std::cout << "ADAPTIVE_SEARCH_TEST=PASS complexity=" << budget.complexity
              << " horizonMax=" << budget.horizonMax << "\n";

    solver::GlobalPlanner planner{};
    auto route = planner.plan(s, general, collision, budget);
    assert(route.valid);
    assert(route.steps.size() >= 2);
    assert(route.plannedForwardDistance > 0.0);
    assert(route.targetX > s.player.x);
    assert(route.targetY >= route.targetYMin && route.targetY <= route.targetYMax);
    std::cout << "GLOBAL_PLANNER_TEST=PASS steps=" << route.steps.size()
              << " branches=" << route.branchesConsidered << "\n";

    world::StaticWorld dynamicSource{};
    dynamicSource.parsed = true;
    dynamicSource.objects = {
        object(world::GameplayObjectType::Hazard, 100.0f, 50.0f, 30.0f, 30.0f, 10, true),
    };
    dynamicSource.hazards = 1;
    world::CollisionWorld dynamicCollision{};
    assert(dynamicCollision.build(dynamicSource, 0.0));

    world::DynamicWorldModel dynamicModel{};
    dynamicModel.observe(dynamicCollision, 10);

    auto moved = dynamicSource.objects.front();
    moved.x += 10.0f;
    moved.nodeX += 10.0f;
    moved.objectRect.x += 10.0f;
    bool reindexed = false;
    assert(dynamicCollision.updatePrimitiveFromWorldObject(0, moved, reindexed));
    dynamicModel.observe(dynamicCollision, 11);

    const auto predicted = dynamicModel.predict(dynamicCollision, 0, 3);
    assert(predicted.available);
    assert(predicted.dynamic);
    assert(predicted.confidence > 0.0);
    const double expectedX = static_cast<double>(moved.objectRect.x) + 30.0;
    assert(std::abs(static_cast<double>(predicted.bounds.x) - expectedX) < 0.01);
    std::cout << "DYNAMIC_WORLD_T_TEST=PASS predictedX=" << predicted.bounds.x << "\n";

    return 0;
}
