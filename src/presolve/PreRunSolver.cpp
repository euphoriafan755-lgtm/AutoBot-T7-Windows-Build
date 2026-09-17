#include "autobot/presolve/PreRunSolver.hpp"
#include "autobot/solver/PortalTransition.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace autobot::presolve {
namespace {

double rectLeft(world::WorldRect const& r) {
    return std::min<double>(r.x, r.x + r.width);
}

double rectRight(world::WorldRect const& r) {
    return std::max<double>(r.x, r.x + r.width);
}

bool gameplayRelevantTrigger(world::WorldObject const& object) {
    return object.trigger.gameplayRelevant && object.trigger.kind != world::TriggerKind::None;
}

bool unsupportedTrigger(world::WorldObject const& object) {
    if (!gameplayRelevantTrigger(object)) return false;
    // The causal model has no faithful contact-condition branch solver yet.
    // Refuse READY instead of treating a touch-triggered branch as deterministic.
    return object.trigger.uncertain || object.trigger.touchTriggered;
}



} // namespace

void PreRunSolver::reset() {
    m_stage = PreRunStage::Idle;
    m_worldModel = {};
    m_triggerGraph = {};
    m_solution = {};
    m_validation.reset();
    m_validationP2.reset();
    m_dynamicWorld.reset();
}

void PreRunSolver::setStage(PreRunStage stage, StageCallback const& callback) {
    m_stage = stage;
    if (callback) callback(stage);
}

TriggerDependencyGraph PreRunSolver::buildTriggerDependencyGraph(
    world::StaticWorld const& source
) {
    TriggerDependencyGraph graph{};
    std::vector<std::size_t> triggerSources;
    std::unordered_map<int, std::vector<std::size_t>> groupToNode;

    for (std::size_t sourceIndex = 0; sourceIndex < source.objects.size(); ++sourceIndex) {
        auto const& object = source.objects[sourceIndex];
        if (!gameplayRelevantTrigger(object)) continue;
        const std::size_t node = triggerSources.size();
        triggerSources.push_back(sourceIndex);
        for (int group : object.groups) {
            if (group > 0) groupToNode[group].push_back(node);
        }
    }
    graph.nodes = triggerSources.size();

    std::vector<std::vector<std::size_t>> adjacency(graph.nodes);
    for (std::size_t node = 0; node < triggerSources.size(); ++node) {
        auto const& object = source.objects[triggerSources[node]];
        if (object.trigger.kind != world::TriggerKind::Spawn) continue;
        const int target = object.trigger.targetGroupID;
        if (target <= 0) {
            ++graph.unresolved;
            continue;
        }
        auto const found = groupToNode.find(target);
        if (found == groupToNode.end()) {
            // A Spawn group may intentionally contain only gameplay objects.
            // That is a resolved leaf, not an unresolved trigger branch.
            continue;
        }
        for (auto to : found->second) {
            graph.edges.push_back({node, to, target});
            adjacency[node].push_back(to);
        }
    }

    std::vector<int> color(graph.nodes, 0);
    std::function<void(std::size_t)> visit = [&](std::size_t u) {
        color[u] = 1;
        for (auto v : adjacency[u]) {
            if (color[v] == 1) graph.cyclic = true;
            else if (color[v] == 0) visit(v);
        }
        color[u] = 2;
    };
    for (std::size_t i = 0; i < graph.nodes; ++i) {
        if (color[i] == 0) visit(i);
    }
    return graph;
}

std::size_t PreRunSolver::countUnmodeled(
    world::StaticWorld const& source,
    world::CollisionWorld const& collisionWorld
) {
    std::size_t count = source.unsupportedGameplay;
    for (auto const& object : source.objects) {
        if (unsupportedTrigger(object)) ++count;
    }
    for (auto const& primitive : collisionWorld.primitives()) {
        const bool gameplayCollision = primitive.classification == world::GameplayObjectType::Solid
            || primitive.classification == world::GameplayObjectType::Hazard
            || primitive.classification == world::GameplayObjectType::Portal;
        if (!gameplayCollision || !primitive.enabled || primitive.noTouch) continue;
        if (primitive.classification == world::GameplayObjectType::Portal) {
            const auto effect = solver::PortalTransitionResolver::resolve(primitive);
            if (effect == solver::PortalEffect::Unknown
                || effect == solver::PortalEffect::Teleport
                || effect == solver::PortalEffect::SpeedChange
                || effect == solver::PortalEffect::Dual
                || effect == solver::PortalEffect::Solo) {
                ++count;
            }
            continue;
        }
        if (primitive.support == world::V01Support::NotSupported
            || primitive.geometryVerification == world::GeometryVerification::NotSupported) {
            ++count;
        }
    }
    return count;
}

double PreRunSolver::worldEndX(
    world::CollisionWorld const& collisionWorld,
    double direction
) {
    if (collisionWorld.primitives().empty()) return 0.0;
    double result = direction >= 0.0
        ? -std::numeric_limits<double>::infinity()
        : std::numeric_limits<double>::infinity();
    for (auto const& primitive : collisionWorld.primitives()) {
        if (!primitive.enabled) continue;
        if (direction >= 0.0) result = std::max(result, rectRight(primitive.broadphaseBounds));
        else result = std::min(result, rectLeft(primitive.broadphaseBounds));
    }
    return std::isfinite(result) ? result : 0.0;
}

bool PreRunSolver::buildWorldModel(
    core::GameSnapshot const& initialSnapshot,
    world::StaticWorld const& source,
    world::CollisionWorld const& collisionWorld
) {
    m_worldModel = {};
    if (!initialSnapshot.valid || !source.parsed || !collisionWorld.ready()) return false;

    m_worldModel.startX = initialSnapshot.player.x;
    m_worldModel.direction = initialSnapshot.player.velocityX < -0.001 ? -1.0 : 1.0;
    m_worldModel.endX = worldEndX(collisionWorld, m_worldModel.direction);
    m_worldModel.sourceObjects = source.objects.size();
    m_worldModel.collisionPrimitives = collisionWorld.primitives().size();
    m_worldModel.hazards = source.hazards;
    m_worldModel.portals = source.portals;
    m_worldModel.unmodeledMechanics = countUnmodeled(source, collisionWorld);
    for (auto const& object : source.objects) {
        if (gameplayRelevantTrigger(object)) ++m_worldModel.gameplayTriggers;
    }

    const double signedSpan = m_worldModel.direction * (m_worldModel.endX - m_worldModel.startX);
    m_worldModel.complete = signedSpan >= -1.0 && m_worldModel.unmodeledMechanics == 0;
    return m_worldModel.complete;
}

solver::PhysicsValidationHarness PreRunSolver::bootstrapValidation(
    core::GameSnapshot const& snapshot
) {
    solver::PhysicsValidationHarness validation{};
    ensureModeCalibration(validation, snapshot);
    return validation;
}

void PreRunSolver::ensureModeCalibration(
    solver::PhysicsValidationHarness& validation,
    core::GameSnapshot const& snapshot
) {
    if (!snapshot.valid || snapshot.player.mode == core::GameMode::Unknown
        || validation.modeReady(snapshot.player.mode)) return;

    auto sample = snapshot;
    sample.valid = true;
    sample.player.dead = false;
    const double rawVx = std::abs(sample.player.velocityX) > 0.01
        ? sample.player.velocityX
        : (sample.player.goingLeft ? -5.0 : 5.0);
    sample.player.velocityX = rawVx;

    constexpr double dt = 1.0 / 60.0;
    sample.levelTime = 10.0;
    sample.solverSampleID = 1000;
    sample.gameTick = 1000;
    validation.observe(sample, control::InputAction::NoPress, false);
    for (int i = 1; i <= 3; ++i) {
        sample.levelTime = 10.0 + dt * i;
        sample.solverSampleID = 1000 + static_cast<std::uint64_t>(i);
        sample.gameTick = sample.solverSampleID;
        sample.player.x += rawVx;
        validation.observe(sample, control::InputAction::NoPress, false);
    }
}

solver::SimState PreRunSolver::stateFromSnapshot(core::GameSnapshot const& snapshot) {
    solver::SimState state{};
    state.x = snapshot.player.x;
    state.y = snapshot.player.y;
    state.vx = snapshot.player.velocityX;
    state.vy = snapshot.player.velocityY;
    state.mode = snapshot.player.mode;
    state.mini = snapshot.player.mini;
    state.grounded = snapshot.player.grounded;
    state.upsideDown = snapshot.player.upsideDown;
    state.holding = snapshot.player.holding;
    state.alive = !snapshot.player.dead;
    state.objectWidth = std::max(1.0, snapshot.player.objectBoundsWidth);
    state.objectHeight = std::max(1.0, snapshot.player.objectBoundsHeight);
    state.rawGravity = snapshot.player.gravity;
    state.gravityModifier = snapshot.player.gravityModifier;
    state.jumpVelocity = snapshot.player.jumpVelocity;
    state.speedScalar = snapshot.player.speed;
    return state;
}

core::GameSnapshot PreRunSolver::snapshotFromTrajectory(
    core::GameSnapshot const& previous,
    solver::TrajectoryResult const& trajectory,
    solver::ActionCandidate const& policy,
    solver::PhysicsValidationHarness const& validation
) {
    auto next = previous;
    if (trajectory.points.empty()) return next;
    auto const& p = trajectory.points.back();
    next.player.x = p.x;
    next.player.y = p.y;
    next.player.velocityX = p.vx;
    next.player.velocityY = p.vy;
    next.player.mode = p.mode;
    next.player.grounded = p.landing;
    next.player.holding = policy.desiredHoldAt(
        trajectory.simulatedTicks > 0 ? trajectory.simulatedTicks - 1 : 0
    );
    next.player.dead = p.collision;
    const auto& calibration = validation.calibration(previous.player.mode);
    const double dt = calibration.sampleDt > 0.0 ? calibration.sampleDt : 1.0 / 60.0;
    next.levelTime += dt * static_cast<double>(std::max<std::size_t>(trajectory.simulatedTicks, 1));
    next.solverSampleID += std::max<std::size_t>(trajectory.simulatedTicks, 1);
    next.gameTick = next.solverSampleID;
    return next;
}

core::GameSnapshot PreRunSolver::player2Snapshot(core::GameSnapshot const& joint) {
    auto p2 = joint;
    p2.player = joint.player2;
    p2.dualMode = false;
    p2.player2Valid = false;
    p2.player2 = {};
    return p2;
}

control::InputAction PreRunSolver::actionFor(
    solver::ActionCandidate const& policy,
    std::size_t tick,
    bool holding
) {
    if (policy.segments.empty()) return control::InputAction::SafeStop;
    const bool desired = policy.desiredHoldAt(tick);
    if (desired && !holding) return control::InputAction::Press;
    if (desired && holding) return control::InputAction::Hold;
    if (!desired && holding) return control::InputAction::Release;
    return control::InputAction::NoPress;
}

bool PreRunSolver::stateWithin(
    core::PlayerState const& actual,
    solver::SimState const& expected,
    StateTolerance const& tolerance
) {
    if (actual.mode != expected.mode || actual.dead || !expected.alive) return false;
    return std::abs(actual.x - expected.x) <= tolerance.x
        && std::abs(actual.y - expected.y) <= tolerance.y
        && std::abs(actual.velocityX - expected.vx) <= tolerance.vx
        && std::abs(actual.velocityY - expected.vy) <= tolerance.vy;
}

std::size_t PreRunSolver::policyTickFor(PlanNode const& node, double x) {
    if (node.simulatedTicks <= 1) return 0;
    const double span = node.endX - node.startX;
    if (std::abs(span) < 0.001) return 0;
    const double t = std::clamp((x - node.startX) / span, 0.0, 1.0);
    return std::min(
        node.simulatedTicks - 1,
        static_cast<std::size_t>(std::floor(t * static_cast<double>(node.simulatedTicks)))
    );
}

bool PreRunSolver::searchSolution(
    core::GameSnapshot const& initialSnapshot,
    world::CollisionWorld const& collisionWorld,
    world::TriggerWorldModel const* triggerWorld
) {
    m_solution = {};
    m_solution.dual = initialSnapshot.dualMode;
    m_solution.unmodeledMechanics = m_worldModel.unmodeledMechanics;
    m_solution.unresolvedBranches = m_triggerGraph.unresolved;

    if (m_solution.unmodeledMechanics > 0 || m_solution.unresolvedBranches > 0) {
        m_solution.reason = "UNMODELED MECHANICS OR UNRESOLVED TRIGGER BRANCH";
        return false;
    }

    // A cyclic Spawn dependency can be a deliberate loop, but the current
    // pre-run finite policy cannot prove its termination. Refuse READY.
    if (m_triggerGraph.cyclic) {
        ++m_solution.unresolvedBranches;
        m_solution.reason = "CYCLIC TRIGGER DEPENDENCY NOT PROVABLY FINITE";
        return false;
    }

    m_validation = bootstrapValidation(initialSnapshot);
    if (initialSnapshot.dualMode && initialSnapshot.player2Valid) {
        m_validationP2 = bootstrapValidation(player2Snapshot(initialSnapshot));
    }
    m_dynamicWorld.observe(collisionWorld, initialSnapshot.solverSampleID);

    auto cursor = initialSnapshot;
    bool p1Holding = cursor.player.holding;
    bool p2Holding = cursor.player2Valid ? cursor.player2.holding : false;
    constexpr std::size_t kMaxNodes = 512;
    constexpr double kFinishMargin = 16.0;

    for (std::size_t nodeIndex = 0; nodeIndex < kMaxNodes; ++nodeIndex) {
        const double remaining = m_worldModel.direction * (m_worldModel.endX - cursor.player.x);
        if (remaining <= kFinishMargin) {
            m_solution.routeFound = !m_solution.nodes.empty() || remaining <= 0.0;
            m_solution.reason = m_solution.routeFound ? "FULL ROUTE FOUND" : "EMPTY ROUTE";
            return m_solution.routeFound;
        }

        ensureModeCalibration(m_validation, cursor);
        if (cursor.dualMode && cursor.player2Valid) {
            ensureModeCalibration(m_validationP2, player2Snapshot(cursor));
        }
        m_dynamicWorld.observe(collisionWorld, cursor.solverSampleID);

        auto decision = m_planner.plan(
            cursor,
            collisionWorld,
            m_validation,
            p1Holding,
            m_dynamicWorld,
            triggerWorld && triggerWorld->ready() ? triggerWorld : nullptr,
            p2Holding,
            cursor.dualMode ? &m_validationP2 : nullptr
        );

        if (!decision.active || decision.selectedTrajectory >= decision.trajectories.size()) {
            m_solution.reason = "OFFLINE HIERARCHICAL SEARCH FOUND NO VERIFIED SEGMENT: " + decision.reason;
            return false;
        }
        auto const& selected = decision.trajectories[decision.selectedTrajectory];
        if (selected.fatalCollision || !selected.horizonConclusive || selected.points.size() < 2) {
            if (selected.fatalCollision) ++m_solution.fatalCollisions;
            m_solution.reason = "SELECTED SEGMENT IS FATAL OR INCONCLUSIVE";
            return false;
        }

        PlanNode node{};
        node.expectedInitial = stateFromSnapshot(cursor);
        node.expectedFinal = node.expectedInitial;
        node.p1Policy = selected.candidate;
        node.dual = cursor.dualMode;
        node.simulatedTicks = std::max<std::size_t>(selected.simulatedTicks, 1);
        node.startX = cursor.player.x;
        node.endX = selected.predictedFinalX;
        node.expectedFinal.x = selected.predictedFinalX;
        node.expectedFinal.y = selected.predictedFinalY;
        node.expectedFinal.mode = selected.finalMode;
        node.expectedFinal.alive = !selected.fatalCollision;
        for (auto const& point : selected.points) {
            if (node.modesVisited.empty() || node.modesVisited.back() != point.mode) {
                node.modesVisited.push_back(point.mode);
                node.modeCheckpoints.push_back({point.x, point.mode});
            }
        }
        if (!selected.points.empty()) {
            auto const& last = selected.points.back();
            node.expectedFinal.vx = last.vx;
            node.expectedFinal.vy = last.vy;
            node.expectedFinal.grounded = last.landing;
            node.expectedFinal.holding = selected.candidate.desiredHoldAt(node.simulatedTicks - 1);
        }

        if (cursor.dualMode) {
            if (decision.selectedP2Trajectory >= decision.p2Trajectories.size()) {
                ++m_solution.unresolvedBranches;
                m_solution.reason = "DUAL POLICY MISSING P2 TRAJECTORY";
                return false;
            }
            auto const& p2 = decision.p2Trajectories[decision.selectedP2Trajectory];
            node.expectedP2Initial = stateFromSnapshot(player2Snapshot(cursor));
            if (p2.fatalCollision || !p2.horizonConclusive || p2.points.size() < 2) {
                if (p2.fatalCollision) ++m_solution.fatalCollisions;
                m_solution.reason = "DUAL P2 SEGMENT IS FATAL OR INCONCLUSIVE";
                return false;
            }
            node.p2Policy = p2.candidate;
            node.expectedP2Final = node.expectedP2Initial;
            node.expectedP2Final.x = p2.predictedFinalX;
            node.expectedP2Final.y = p2.predictedFinalY;
            node.expectedP2Final.mode = p2.finalMode;
            node.expectedP2Final.alive = !p2.fatalCollision;
            if (!p2.points.empty()) {
                auto const& lastP2 = p2.points.back();
                node.expectedP2Final.vx = lastP2.vx;
                node.expectedP2Final.vy = lastP2.vy;
                node.expectedP2Final.grounded = lastP2.landing;
                node.expectedP2Final.holding = p2.candidate.desiredHoldAt(
                    std::max<std::size_t>(p2.simulatedTicks, 1) - 1
                );
            }
        }

        m_solution.nodes.push_back(node);
        auto next = snapshotFromTrajectory(cursor, selected, selected.candidate, m_validation);
        if (m_worldModel.direction * (next.player.x - cursor.player.x) <= 0.01) {
            ++m_solution.unresolvedBranches;
            m_solution.reason = "OFFLINE SEARCH MADE NO FORWARD PROGRESS";
            return false;
        }

        if (cursor.dualMode && cursor.player2Valid) {
            auto const& p2 = decision.p2Trajectories[decision.selectedP2Trajectory];
            auto oldP2 = player2Snapshot(cursor);
            auto nextP2 = snapshotFromTrajectory(oldP2, p2, p2.candidate, m_validationP2);
            next.player2 = nextP2.player;
            next.player2Valid = true;
            next.dualMode = true;
            p2Holding = p2.candidate.desiredHoldAt(
                p2.simulatedTicks > 0 ? p2.simulatedTicks - 1 : 0
            );
        }
        p1Holding = selected.candidate.desiredHoldAt(
            selected.simulatedTicks > 0 ? selected.simulatedTicks - 1 : 0
        );
        cursor = next;
    }

    ++m_solution.unresolvedBranches;
    m_solution.reason = "OFFLINE SEARCH NODE LIMIT REACHED";
    return false;
}

bool PreRunSolver::simulateAndVerify(core::GameSnapshot const& initialSnapshot) {
    if (!m_solution.routeFound) return false;
    if (m_solution.nodes.empty()) {
        m_solution.simulatedCompletion = 100.0;
        m_solution.simulationPassed = true;
        m_solution.verified = true;
        return true;
    }

    double furthest = initialSnapshot.player.x;
    for (std::size_t i = 0; i < m_solution.nodes.size(); ++i) {
        auto& node = m_solution.nodes[i];
        node.nextNode = i + 1 < m_solution.nodes.size()
            ? i + 1
            : std::numeric_limits<std::size_t>::max();
        if (!node.expectedInitial.alive || !node.expectedFinal.alive
            || node.p1Policy.segments.empty() || node.simulatedTicks == 0) {
            ++m_solution.fatalCollisions;
            m_solution.reason = "POLICY REPLAY CONTAINS INVALID OR FATAL NODE";
            return false;
        }
        furthest = m_worldModel.direction >= 0.0
            ? std::max(furthest, node.endX)
            : std::min(furthest, node.endX);
    }

    const double total = std::max(
        1.0,
        std::abs(m_worldModel.endX - m_worldModel.startX)
    );
    const double traversed = std::clamp(
        m_worldModel.direction * (furthest - m_worldModel.startX),
        0.0,
        total
    );
    m_solution.simulatedCompletion = 100.0 * traversed / total;
    if (m_worldModel.direction * (m_worldModel.endX - furthest) <= 16.0) {
        m_solution.simulatedCompletion = 100.0;
    }

    m_solution.simulationPassed = m_solution.simulatedCompletion >= 99.999
        && m_solution.fatalCollisions == 0;
    m_solution.verified = m_solution.simulationPassed
        && m_solution.unmodeledMechanics == 0
        && m_solution.unresolvedBranches == 0;
    if (!m_solution.verified) {
        m_solution.reason = "PRE-RUN VERIFICATION GATE FAILED";
        return false;
    }
    m_solution.reason = "PRE-RUN VERIFIED SOLUTION";
    return true;
}

bool PreRunSolver::prepare(
    core::GameSnapshot const& initialSnapshot,
    world::StaticWorld const& source,
    world::CollisionWorld const& collisionWorld,
    world::TriggerWorldModel const* triggerWorld,
    StageCallback const& onStage
) {
    reset();
    setStage(PreRunStage::ParsingLevel, onStage);
    if (!initialSnapshot.valid || !source.parsed) {
        m_solution.reason = "FULL LEVEL PARSE NOT AVAILABLE";
        setStage(PreRunStage::NotReady, onStage);
        return false;
    }

    setStage(PreRunStage::ModelingWorld, onStage);
    if (!buildWorldModel(initialSnapshot, source, collisionWorld)) {
        m_solution.unmodeledMechanics = m_worldModel.unmodeledMechanics;
        m_solution.reason = "FULL WORLD MODEL INCOMPLETE OR HAS UNMODELED GAMEPLAY";
        setStage(PreRunStage::NotReady, onStage);
        return false;
    }

    setStage(PreRunStage::ResolvingTriggers, onStage);
    m_triggerGraph = buildTriggerDependencyGraph(source);
    if (m_triggerGraph.unresolved > 0 || m_triggerGraph.cyclic) {
        m_solution.unresolvedBranches = m_triggerGraph.unresolved + (m_triggerGraph.cyclic ? 1u : 0u);
        m_solution.reason = "TRIGGER DEPENDENCY GRAPH NOT FULLY RESOLVED";
        setStage(PreRunStage::NotReady, onStage);
        return false;
    }
    if (m_worldModel.gameplayTriggers > 0 && (!triggerWorld || !triggerWorld->ready())) {
        m_solution.unresolvedBranches = 1;
        m_solution.reason = "GAMEPLAY TRIGGERS PRESENT WITHOUT CAUSAL TRIGGER WORLD";
        setStage(PreRunStage::NotReady, onStage);
        return false;
    }

    setStage(PreRunStage::Searching, onStage);
    if (!searchSolution(initialSnapshot, collisionWorld, triggerWorld)) {
        setStage(PreRunStage::NotReady, onStage);
        return false;
    }

    setStage(PreRunStage::Simulating, onStage);
    // Search stores the exact non-fatal simulated trajectory selected for each
    // hierarchical segment. The replay gate below checks the entire assembled
    // 0%-100% policy, not only the next realtime horizon.
    setStage(PreRunStage::Verifying, onStage);
    if (!simulateAndVerify(initialSnapshot)) {
        setStage(PreRunStage::NotReady, onStage);
        return false;
    }

    m_solution.ready = true;
    setStage(PreRunStage::Ready, onStage);
    return true;
}

PolicyDecision PreRunSolver::policyFor(
    core::GameSnapshot const& snapshot,
    bool p1Holding,
    bool p2Holding
) const {
    PolicyDecision result{};
    result.ready = ready();
    if (!result.ready || !snapshot.valid || snapshot.player.dead) {
        result.reason = "PRE-RUN NOT READY";
        return result;
    }

    const double x = snapshot.player.x;
    for (std::size_t i = 0; i < m_solution.nodes.size(); ++i) {
        auto const& node = m_solution.nodes[i];
        const double minX = std::min(node.startX, node.endX) - node.tolerance.x;
        const double maxX = std::max(node.startX, node.endX) + node.tolerance.x;
        if (x < minX || x > maxX) continue;

        const double span = node.endX - node.startX;
        const double t = std::abs(span) < 0.001
            ? 0.0
            : std::clamp((x - node.startX) / span, 0.0, 1.0);
        auto expected = node.expectedInitial;
        expected.x = x;
        expected.y = node.expectedInitial.y + (node.expectedFinal.y - node.expectedInitial.y) * t;
        expected.vx = node.expectedInitial.vx + (node.expectedFinal.vx - node.expectedInitial.vx) * t;
        expected.vy = node.expectedInitial.vy + (node.expectedFinal.vy - node.expectedInitial.vy) * t;
        for (auto const& checkpoint : node.modeCheckpoints) {
            const bool passed = m_worldModel.direction >= 0.0
                ? x >= checkpoint.x - 1.0
                : x <= checkpoint.x + 1.0;
            if (passed) expected.mode = checkpoint.mode;
        }
        if (!stateWithin(snapshot.player, expected, node.tolerance)) {
            result.desync = true;
            result.nodeIndex = i;
            result.reason = "PRE-RUN POLICY STATE DESYNC";
            return result;
        }
        if (node.dual && snapshot.player2Valid) {
            auto expectedP2 = node.expectedP2Initial;
            expectedP2.x = snapshot.player2.x;
            expectedP2.y = node.expectedP2Initial.y
                + (node.expectedP2Final.y - node.expectedP2Initial.y) * t;
            expectedP2.vx = node.expectedP2Initial.vx
                + (node.expectedP2Final.vx - node.expectedP2Initial.vx) * t;
            expectedP2.vy = node.expectedP2Initial.vy
                + (node.expectedP2Final.vy - node.expectedP2Initial.vy) * t;
            expectedP2.mode = t < 0.5 ? node.expectedP2Initial.mode : node.expectedP2Final.mode;
            if (!stateWithin(snapshot.player2, expectedP2, node.tolerance)) {
                result.desync = true;
                result.nodeIndex = i;
                result.reason = "PRE-RUN DUAL P2 STATE DESYNC";
                return result;
            }
        }

        const auto tick = policyTickFor(node, x);
        result.p1 = actionFor(node.p1Policy, tick, p1Holding);
        result.p2 = node.dual
            ? actionFor(node.p2Policy, tick, p2Holding)
            : control::InputAction::NoPress;
        result.matched = true;
        result.nodeIndex = i;
        result.reason = "PRE-RUN VERIFIED POLICY";
        return result;
    }

    const double completed = m_worldModel.direction * (x - m_worldModel.endX);
    if (completed >= -16.0) {
        result.matched = true;
        result.p1 = p1Holding ? control::InputAction::Release : control::InputAction::NoPress;
        result.p2 = p2Holding ? control::InputAction::Release : control::InputAction::NoPress;
        result.reason = "PRE-RUN POLICY COMPLETE";
        return result;
    }

    result.desync = true;
    result.reason = "PRE-RUN POLICY HAS NO NODE FOR CURRENT STATE";
    return result;
}

} // namespace autobot::presolve
