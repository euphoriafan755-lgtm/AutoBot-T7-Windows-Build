#include "autobot/presolve/PreRunSolver.hpp"
#include "autobot/solver/PortalTransition.hpp"

#include <cassert>
#include <iostream>

using namespace autobot;

namespace autobot::solver {
PortalEffect PortalTransitionResolver::resolve(world::CollisionPrimitive const& p) {
    if (p.objectID == 1001) return PortalEffect::ModeShip;
    if (p.objectID == 1002) return PortalEffect::ModeWave;
    return PortalEffect::None;
}
PortalApplication PortalTransitionResolver::apply(world::CollisionPrimitive const& p, SimState& s) {
    PortalApplication out{};
    out.effect = resolve(p);
    if (out.effect == PortalEffect::ModeShip) {
        out.modeChanged = s.mode != core::GameMode::Ship;
        out.stateChanged = true;
        s.mode = core::GameMode::Ship;
        s.grounded = false;
    } else if (out.effect == PortalEffect::ModeWave) {
        out.modeChanged = s.mode != core::GameMode::Wave;
        out.stateChanged = true;
        s.mode = core::GameMode::Wave;
        s.grounded = false;
    }
    return out;
}
} // namespace autobot::solver

namespace {
world::WorldObject baseObject(world::GameplayObjectType type, int id, float x, float y, float w, float h) {
    world::WorldObject o{};
    o.objectID = id;
    o.uniqueID = id * 10 + static_cast<int>(x);
    o.type = type;
    o.v01Support = world::V01Support::Supported;
    o.x = x + w * 0.5f;
    o.y = y + h * 0.5f;
    o.nodeX = o.x;
    o.nodeY = o.y;
    o.objectRect = {x, y, w, h};
    o.contentWidth = w;
    o.contentHeight = h;
    o.enabled = true;
    return o;
}

world::StaticWorld simpleWorld(bool portals = false) {
    world::StaticWorld w{};
    w.parsed = true;
    w.objects.push_back(baseObject(world::GameplayObjectType::Solid, 1, -100.0f, 0.0f, 900.0f, 10.0f));
    w.solids = 1;
    if (portals) {
        w.objects.push_back(baseObject(world::GameplayObjectType::Portal, 1001, 220.0f, 0.0f, 24.0f, 220.0f));
        w.objects.push_back(baseObject(world::GameplayObjectType::Portal, 1002, 440.0f, 0.0f, 24.0f, 220.0f));
        w.portals = 2;
    }
    // Deliberately different from the last collider (x=800): completion truth
    // must come from the level boundary, not collision geometry.
    w.endPositionX = 760.0;
    w.gdLevelLength = 760.0;
    w.completionBoundaryX = 760.0;
    w.completionBoundaryValid = true;
    w.completionSourcesConsistent = true;
    return w;
}


world::StaticWorld portalWorld() {
    world::StaticWorld w{};
    w.parsed = true;
    w.objects.push_back(baseObject(world::GameplayObjectType::Portal, 1001, 160.0f, -800.0f, 24.0f, 2000.0f));
    w.objects.push_back(baseObject(world::GameplayObjectType::Portal, 1002, 340.0f, -800.0f, 24.0f, 2000.0f));
    w.portals = 2;
    w.endPositionX = 360.0;
    w.gdLevelLength = 360.0;
    w.completionBoundaryX = 360.0;
    w.completionBoundaryValid = true;
    w.completionSourcesConsistent = true;
    return w;
}

core::GameSnapshot snapshot(bool dual = false) {
    core::GameSnapshot s{};
    s.valid = true;
    s.gameTick = 1;
    s.solverSampleID = 1;
    s.levelTime = 0.0;
    s.player.x = 0.0;
    s.player.y = 25.0;
    s.player.velocityX = 5.2;
    s.player.velocityY = 0.0;
    s.player.gravity = 0.95;
    s.player.gravityModifier = 1.0;
    s.player.jumpVelocity = 11.5;
    s.player.speed = 1.0;
    s.player.objectBoundsWidth = 20.0;
    s.player.objectBoundsHeight = 20.0;
    s.player.mode = core::GameMode::Cube;
    s.player.grounded = true;
    if (dual) {
        s.dualMode = true;
        s.player2Valid = true;
        s.player2 = s.player;
        s.player2.y = 45.0;
    }
    return s;
}

world::TriggerDescriptor spawnDescriptor(int target) {
    world::TriggerDescriptor d{};
    d.kind = world::TriggerKind::Spawn;
    d.gameplayRelevant = true;
    d.targetGroupID = target;
    return d;
}

world::TriggerDescriptor toggleDescriptor(int target) {
    world::TriggerDescriptor d{};
    d.kind = world::TriggerKind::Toggle;
    d.gameplayRelevant = true;
    d.targetGroupID = target;
    return d;
}
} // namespace

int main() {
    {
        auto source = simpleWorld(false);
        world::CollisionWorld collision{};
        assert(collision.build(source, 0.0));
        world::TriggerWorldModel trigger{};
        assert(trigger.build(source, collision));
        presolve::PreRunSolver solver{};
        const auto start = snapshot(false);
        const bool prepared = solver.prepare(start, source, collision, &trigger);
        if (!prepared) {
            std::cerr << "PREPARE_FAIL reason=" << solver.solution().reason
                      << " replayTicks=" << solver.solution().replayTicks
                      << " completion=" << solver.solution().simulatedCompletion
                      << " fatal=" << solver.solution().fatalCollisions << "\n";
        }
        assert(prepared);
        auto const& model = solver.worldModel();
        auto const& solution = solver.solution();
        assert(model.complete);
        assert(model.completionBoundaryValid);
        assert(model.completionSourcesConsistent);
        assert(model.endX == 760.0);
        assert(model.lastColliderX != model.endX);
        assert(solution.routeFound);
        assert(solution.simulationPassed);
        assert(solution.fullPolicyReplayPassed);
        assert(solution.replayTicks > 0);
        assert(solution.verified);
        assert(solution.ready);
        assert(solution.simulatedCompletion >= 99.999);
        assert(solution.fatalCollisions == 0);
        assert(solution.unmodeledMechanics == 0);
        assert(solution.unresolvedBranches == 0);
        std::cout << "FULL_LEVEL_PARSE=PASS objects=" << model.sourceObjects << "\n";
        std::cout << "FULL_WORLD_MODEL=PASS endX=" << model.endX << "\n";
        std::cout << "FULL_SOLUTION_SEARCH=PASS nodes=" << solution.nodes.size() << "\n";
        std::cout << "FULL_SIMULATION=PASS completion=" << solution.simulatedCompletion << "\n";
        std::cout << "FULL_POLICY_REPLAY_TEST=PASS ticks=" << solution.replayTicks << "\n";
        std::cout << "ACTUAL_LEVEL_COMPLETION_BOUNDARY_TEST=PASS boundary=" << model.endX
                  << " lastCollider=" << model.lastColliderX << "\n";
        std::cout << "PRE_RUN_VERIFIED_SOLUTION=PASS\n";
    }

    {
        auto source = portalWorld();
        world::CollisionWorld collision{};
        assert(collision.build(source, 0.0));
        world::TriggerWorldModel trigger{};
        assert(trigger.build(source, collision));
        presolve::PreRunSolver solver{};
        auto portalStart = snapshot(false);
        portalStart.player.y = 130.0;
        portalStart.player.grounded = false;
        assert(solver.prepare(portalStart, source, collision, &trigger));
        bool sawShip = false;
        bool sawWave = false;
        for (auto const& node : solver.solution().nodes) {
            for (auto mode : node.modesVisited) {
                sawShip = sawShip || mode == core::GameMode::Ship;
                sawWave = sawWave || mode == core::GameMode::Wave;
            }
        }
        assert(sawShip && sawWave);
        std::cout << "PRE_RUN_MODE_PORTAL_CHAIN=PASS nodes=" << solver.solution().nodes.size() << "\n";
    }

    {
        auto source = simpleWorld(false);
        world::CollisionWorld collision{};
        assert(collision.build(source, 0.0));
        world::TriggerWorldModel trigger{};
        assert(trigger.build(source, collision));
        presolve::PreRunSolver solver{};
        assert(solver.prepare(snapshot(true), source, collision, &trigger));
        assert(solver.solution().dual);
        bool hasJointPolicy = false;
        for (auto const& node : solver.solution().nodes) {
            hasJointPolicy = hasJointPolicy || (node.dual && !node.p1Policy.segments.empty() && !node.p2Policy.segments.empty());
        }
        assert(hasJointPolicy);
        std::cout << "PRE_RUN_DUAL_JOINT_POLICY=PASS nodes=" << solver.solution().nodes.size() << "\n";
    }

    {
        presolve::FreezeInvariantSnapshot anchor{};
        anchor.playerX = 100.0;
        anchor.progress = 12.5;
        anchor.levelTime = 3.25;
        anchor.attempts = 1;
        anchor.dead = false;
        auto frozen = anchor;
        assert(presolve::shouldFreezeGameplay(presolve::PreRunStage::NotReady));
        assert(!presolve::shouldFreezeGameplay(presolve::PreRunStage::Ready));
        assert(presolve::freezeInvariantHolds(anchor, frozen));
        auto moved = frozen; moved.playerX += 0.01;
        auto progressed = frozen; progressed.progress += 0.01;
        auto timed = frozen; timed.levelTime += 0.01;
        auto restarted = frozen; ++restarted.attempts;
        auto died = frozen; died.dead = true;
        assert(!presolve::freezeInvariantHolds(anchor, moved));
        assert(!presolve::freezeInvariantHolds(anchor, progressed));
        assert(!presolve::freezeInvariantHolds(anchor, timed));
        assert(!presolve::freezeInvariantHolds(anchor, restarted));
        assert(!presolve::freezeInvariantHolds(anchor, died));
        std::cout << "PRE_RUN_FREEZE_TEST=PASS playerXDelta=0 progressDelta=0 deaths=0 attemptRestart=0\n";
    }

    {
        world::StaticWorld source{};
        source.parsed = true;
        auto a = baseObject(world::GameplayObjectType::Decoration, 1268, 10, 0, 10, 10);
        a.trigger = spawnDescriptor(20);
        a.groups = {10};
        a.groupCount = 1;
        auto b = baseObject(world::GameplayObjectType::Decoration, 1049, 20, 0, 10, 10);
        b.trigger = toggleDescriptor(30);
        b.groups = {20};
        b.groupCount = 1;
        source.objects = {a, b};
        source.decorations = 2;
        auto graph = presolve::PreRunSolver::buildTriggerDependencyGraph(source);
        assert(graph.nodes == 2);
        assert(graph.edges.size() == 1);
        assert(graph.unresolved == 0);
        assert(!graph.cyclic);
        std::cout << "TRIGGER_DEPENDENCY_GRAPH=PASS nodes=" << graph.nodes
                  << " edges=" << graph.edges.size() << "\n";
    }
    return 0;
}
