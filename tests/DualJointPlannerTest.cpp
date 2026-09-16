#include "autobot/core/GameSnapshot.hpp"
#include "autobot/solver/DualJointPlanner.hpp"

#include <cassert>
#include <iostream>

using namespace autobot;

namespace autobot::solver {
PortalEffect PortalTransitionResolver::resolve(world::CollisionPrimitive const&) {
    return PortalEffect::None;
}
PortalApplication PortalTransitionResolver::apply(
    world::CollisionPrimitive const&, SimState&
) {
    return {};
}
} // namespace autobot::solver

namespace {

solver::TrajectoryResult trajectory(
    char const* label,
    bool fatal,
    bool conclusive,
    bool compatible,
    double score
) {
    solver::TrajectoryResult t{};
    t.candidate.label = label;
    t.fatalCollision = fatal;
    t.horizonConclusive = conclusive;
    t.globalRouteCompatible = compatible;
    t.score = score;
    t.classification = fatal
        ? solver::TrajectoryClass::Collision
        : (conclusive ? solver::TrajectoryClass::Safe
                      : solver::TrajectoryClass::HorizonInconclusive);
    return t;
}

} // namespace

int main() {
    core::GameSnapshot joint{};
    joint.valid = true;
    joint.dualMode = true;
    joint.player2Valid = true;
    joint.player.mode = core::GameMode::Cube;
    joint.player.x = 10.0;
    joint.player2.mode = core::GameMode::Wave;
    joint.player2.x = 12.0;
    assert(joint.dualMode && joint.player2Valid);
    assert(joint.player.mode == core::GameMode::Cube);
    assert(joint.player2.mode == core::GameMode::Wave);
    assert(joint.player.x != joint.player2.x);
    std::cout << "DUAL_JOINT_STATE_TEST=PASS\n";

    auto p1Safe = trajectory("P1 SAFE", false, true, true, 100.0);
    auto p2Fatal = trajectory("P2 FATAL", true, true, true, 100000.0);
    assert(solver::DualJointPlanner::safetyTier(p1Safe, p2Fatal, true) == 0);
    std::cout << "DUAL_COLLISION_TEST=PASS\n";

    std::vector<solver::TrajectoryResult> p1{
        trajectory("P1 FATAL HIGH SCORE", true, true, true, 500000.0),
        trajectory("P1 SAFE", false, true, true, 80.0),
        trajectory("P1 INCONCLUSIVE", false, false, true, 1000.0),
    };
    std::vector<solver::TrajectoryResult> p2{
        trajectory("P2 SAFE", false, true, true, 90.0),
        trajectory("P2 FATAL", true, true, true, 999999.0),
        trajectory("P2 SAFE OFF ROUTE", false, true, false, 5000.0),
    };

    const auto best = solver::DualJointPlanner::selectBestPair(p1, p2);
    assert(best.p1Index == 1);
    assert(best.p2Index == 0);
    assert(!best.fatalCollision);
    assert(best.horizonConclusive);
    assert(best.globalCompatible);
    assert(best.safetyTier == 3);
    std::cout << "DUAL_ACTION_SEARCH_TEST=PASS\n";
    return 0;
}
