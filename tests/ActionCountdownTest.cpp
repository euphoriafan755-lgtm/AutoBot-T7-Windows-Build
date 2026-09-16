#include "autobot/control/AutonomousTestDriver.hpp"
#include "autobot/core/GameSnapshot.hpp"
#include "autobot/solver/PortalTransition.hpp"
#include "autobot/world/CollisionWorld.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>
#include <string_view>

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

std::string_view selectedLabel(control::AutonomousDecision const& decision) {
    if (decision.plan.selectedTrajectory >= decision.plan.trajectories.size()) return "NONE";
    return decision.plan.trajectories[decision.plan.selectedTrajectory].candidate.label;
}

void assertCountdown(
    control::AutonomousDecision const& decision,
    std::size_t dueIn,
    std::string_view expectedLabel,
    control::InputAction expectedAction,
    bool fired
) {
    assert(decision.countdown.dueIn == dueIn);
    assert(selectedLabel(decision) == expectedLabel);
    assert(decision.action == expectedAction);
    assert(decision.countdown.fired == fired);
    if (dueIn > 0) assert(decision.countdown.active);
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
    constexpr double dx = rawClassicVx;

    control::AutonomousTestDriver driver{};
    bool botHolding = false;

    auto s0 = snapshot(100, 0.0, -2.0 * dx, rawClassicVx);
    auto s1 = snapshot(101, realDt, -1.0 * dx, rawClassicVx);
    auto s2 = snapshot(102, realDt * 2.0, 0.0, rawClassicVx);

    (void)driver.decide(s0, collisionWorld, collisionWorld.queryAhead(static_cast<float>(s0.player.x), 15.0f), true, botHolding);
    (void)driver.decide(s1, collisionWorld, collisionWorld.queryAhead(static_cast<float>(s1.player.x), 15.0f), true, botHolding);
    auto d0 = driver.decide(s2, collisionWorld, collisionWorld.queryAhead(static_cast<float>(s2.player.x), 15.0f), true, botHolding);

    assertCountdown(d0, 3, "PRESS +3", control::InputAction::NoPress, false);
    const auto absoluteTarget = d0.countdown.targetSampleID;
    assert(absoluteTarget == 105);
    std::cout << "sample 102: PRESS +3 -> due in 3\n";

    auto s3 = snapshot(103, realDt * 3.0, dx, rawClassicVx);
    auto d1 = driver.decide(s3, collisionWorld, collisionWorld.queryAhead(static_cast<float>(s3.player.x), 15.0f), true, botHolding);
    assert(d1.countdown.targetSampleID == absoluteTarget);
    assertCountdown(d1, 2, "PRESS +2", control::InputAction::NoPress, false);
    std::cout << "sample 103: PRESS +2 -> due in 2\n";

    auto s4 = snapshot(104, realDt * 4.0, 2.0 * dx, rawClassicVx);
    auto d2 = driver.decide(s4, collisionWorld, collisionWorld.queryAhead(static_cast<float>(s4.player.x), 15.0f), true, botHolding);
    assert(d2.countdown.targetSampleID == absoluteTarget);
    assertCountdown(d2, 1, "PRESS +1", control::InputAction::NoPress, false);
    std::cout << "sample 104: PRESS +1 -> due in 1\n";

    auto s5 = snapshot(105, realDt * 5.0, 3.0 * dx, rawClassicVx);
    auto d3 = driver.decide(s5, collisionWorld, collisionWorld.queryAhead(static_cast<float>(s5.player.x), 15.0f), true, botHolding);
    assert(d3.countdown.targetSampleID == absoluteTarget);
    assertCountdown(d3, 0, "PRESS NOW", control::InputAction::Press, true);
    std::cout << "sample 105: PRESS NOW -> PRESS\n";

    std::cout << "ACTION_COUNTDOWN_TEST=PASS\n";
    return 0;
}
