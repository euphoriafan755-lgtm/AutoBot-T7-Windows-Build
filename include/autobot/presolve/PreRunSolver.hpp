#pragma once

#include "autobot/control/InputController.hpp"
#include "autobot/core/GameSnapshot.hpp"
#include "autobot/solver/PhysicsValidationHarness.hpp"
#include "autobot/solver/RealtimePlanner.hpp"
#include "autobot/solver/SolverTypes.hpp"
#include "autobot/world/CollisionWorld.hpp"
#include "autobot/world/DynamicWorldModel.hpp"
#include "autobot/world/TriggerWorldModel.hpp"
#include "autobot/world/WorldObject.hpp"

#include <cstddef>
#include <functional>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

namespace autobot::presolve {

enum class PreRunStage {
    Idle,
    ParsingLevel,
    ModelingWorld,
    ResolvingTriggers,
    Searching,
    Simulating,
    Verifying,
    Ready,
    NotReady,
};

[[nodiscard]] constexpr std::string_view toString(PreRunStage stage) {
    switch (stage) {
        case PreRunStage::Idle: return "IDLE";
        case PreRunStage::ParsingLevel: return "PARSING LEVEL";
        case PreRunStage::ModelingWorld: return "MODELING WORLD";
        case PreRunStage::ResolvingTriggers: return "RESOLVING TRIGGERS";
        case PreRunStage::Searching: return "SEARCHING";
        case PreRunStage::Simulating: return "SIMULATING";
        case PreRunStage::Verifying: return "VERIFYING";
        case PreRunStage::Ready: return "READY";
        case PreRunStage::NotReady: return "NOT READY";
    }
    return "NOT READY";
}

struct TriggerDependencyEdge {
    std::size_t from = 0;
    std::size_t to = 0;
    int groupID = 0;
};

struct TriggerDependencyGraph {
    std::size_t nodes = 0;
    std::vector<TriggerDependencyEdge> edges;
    std::size_t unresolved = 0;
    bool cyclic = false;
};

struct LevelWorldModel {
    bool complete = false;
    double startX = 0.0;
    double endX = 0.0;
    double direction = 1.0;
    std::size_t sourceObjects = 0;
    std::size_t collisionPrimitives = 0;
    std::size_t hazards = 0;
    std::size_t portals = 0;
    std::size_t gameplayTriggers = 0;
    std::size_t unmodeledMechanics = 0;
};

struct StateTolerance {
    double x = 36.0;
    double y = 48.0;
    double vx = 3.0;
    double vy = 8.0;
};

struct ModeCheckpoint {
    double x = 0.0;
    core::GameMode mode = core::GameMode::Unknown;
};

struct PlanNode {
    solver::SimState expectedInitial{};
    solver::SimState expectedFinal{};
    solver::SimState expectedP2Initial{};
    solver::SimState expectedP2Final{};
    StateTolerance tolerance{};
    solver::ActionCandidate p1Policy{};
    solver::ActionCandidate p2Policy{};
    std::vector<core::GameMode> modesVisited;
    std::vector<ModeCheckpoint> modeCheckpoints;
    bool dual = false;
    std::size_t simulatedTicks = 0;
    double startX = 0.0;
    double endX = 0.0;
    std::size_t nextNode = std::numeric_limits<std::size_t>::max();
};

struct PreRunSolution {
    bool routeFound = false;
    bool simulationPassed = false;
    bool verified = false;
    bool ready = false;
    bool dual = false;
    double simulatedCompletion = 0.0;
    std::size_t fatalCollisions = 0;
    std::size_t unresolvedBranches = 0;
    std::size_t unmodeledMechanics = 0;
    std::vector<PlanNode> nodes;
    std::string reason = "NOT PREPARED";
};

struct PolicyDecision {
    bool ready = false;
    bool matched = false;
    bool desync = false;
    control::InputAction p1 = control::InputAction::SafeStop;
    control::InputAction p2 = control::InputAction::SafeStop;
    std::size_t nodeIndex = std::numeric_limits<std::size_t>::max();
    std::string reason = "PRE-RUN NOT READY";
};

class PreRunSolver final {
public:
    using StageCallback = std::function<void(PreRunStage)>;

    void reset();

    [[nodiscard]] bool prepare(
        core::GameSnapshot const& initialSnapshot,
        world::StaticWorld const& source,
        world::CollisionWorld const& collisionWorld,
        world::TriggerWorldModel const* triggerWorld,
        StageCallback const& onStage = {}
    );

    [[nodiscard]] PolicyDecision policyFor(
        core::GameSnapshot const& snapshot,
        bool p1Holding,
        bool p2Holding
    ) const;

    [[nodiscard]] PreRunStage stage() const { return m_stage; }
    [[nodiscard]] LevelWorldModel const& worldModel() const { return m_worldModel; }
    [[nodiscard]] TriggerDependencyGraph const& triggerGraph() const { return m_triggerGraph; }
    [[nodiscard]] PreRunSolution const& solution() const { return m_solution; }
    [[nodiscard]] bool ready() const { return m_stage == PreRunStage::Ready && m_solution.ready; }

    [[nodiscard]] static TriggerDependencyGraph buildTriggerDependencyGraph(
        world::StaticWorld const& source
    );

private:
    static solver::PhysicsValidationHarness bootstrapValidation(core::GameSnapshot const& snapshot);
    static void ensureModeCalibration(
        solver::PhysicsValidationHarness& validation,
        core::GameSnapshot const& snapshot
    );
    static solver::SimState stateFromSnapshot(core::GameSnapshot const& snapshot);
    static core::GameSnapshot snapshotFromTrajectory(
        core::GameSnapshot const& previous,
        solver::TrajectoryResult const& trajectory,
        solver::ActionCandidate const& policy,
        solver::PhysicsValidationHarness const& validation
    );
    static core::GameSnapshot player2Snapshot(core::GameSnapshot const& joint);
    static control::InputAction actionFor(
        solver::ActionCandidate const& policy,
        std::size_t tick,
        bool holding
    );
    static bool stateWithin(
        core::PlayerState const& actual,
        solver::SimState const& expected,
        StateTolerance const& tolerance
    );
    static std::size_t policyTickFor(PlanNode const& node, double x);
    static double worldEndX(world::CollisionWorld const& collisionWorld, double direction);
    static std::size_t countUnmodeled(
        world::StaticWorld const& source,
        world::CollisionWorld const& collisionWorld
    );

    void setStage(PreRunStage stage, StageCallback const& callback);
    bool buildWorldModel(
        core::GameSnapshot const& initialSnapshot,
        world::StaticWorld const& source,
        world::CollisionWorld const& collisionWorld
    );
    bool searchSolution(
        core::GameSnapshot const& initialSnapshot,
        world::CollisionWorld const& collisionWorld,
        world::TriggerWorldModel const* triggerWorld
    );
    bool simulateAndVerify(core::GameSnapshot const& initialSnapshot);

    PreRunStage m_stage = PreRunStage::Idle;
    LevelWorldModel m_worldModel{};
    TriggerDependencyGraph m_triggerGraph{};
    PreRunSolution m_solution{};
    solver::PhysicsValidationHarness m_validation{};
    solver::PhysicsValidationHarness m_validationP2{};
    solver::RealtimePlanner m_planner{};
    world::DynamicWorldModel m_dynamicWorld{};
};

} // namespace autobot::presolve
