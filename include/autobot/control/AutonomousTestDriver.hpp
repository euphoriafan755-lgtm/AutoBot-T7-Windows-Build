#pragma once

#include "autobot/control/InputController.hpp"
#include "autobot/core/GameSnapshot.hpp"
#include "autobot/presolve/PreRunSolver.hpp"
#include "autobot/solver/PhysicsValidationHarness.hpp"
#include "autobot/solver/RealtimePlanner.hpp"
#include "autobot/world/CollisionWorld.hpp"
#include "autobot/world/DynamicWorldModel.hpp"
#include "autobot/world/TriggerWorldModel.hpp"
#include "autobot/world/WorldObject.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace autobot::control {

struct CandidateModelTruthTrace {
    std::string label;
    solver::TrajectoryClass classification = solver::TrajectoryClass::Unknown;
    bool fatalCollision = false;
    bool hazardCollision = false;
    bool horizonConclusive = false;
    bool portalCrossed = false;
    bool uncertainGeometry = false;
    bool nearestHazardSeen = false;
    std::size_t collisionPrimitive = world::kInvalidPrimitiveIndex;
    std::size_t collisionTick = std::numeric_limits<std::size_t>::max();
    double minimumClearance = 0.0;
    double progress = 0.0;
    double confidence = 0.0;
    double score = 0.0;
    double finalX = 0.0;
    double finalY = 0.0;
};

struct SolverModelTruthSample {
    std::uint64_t solverSampleID = 0;
    double levelTime = 0.0;
    core::PlayerState player{};
    solver::ModelTruthDiagnostics diagnostics{};
    std::vector<CandidateModelTruthTrace> candidates;
    std::size_t selectedTrajectory = std::numeric_limits<std::size_t>::max();
    InputAction selectedInput = InputAction::SafeStop;
    std::string selectedLabel = "NONE";
    bool predictedDeath = false;
    bool selectedConclusive = false;
    bool dualMode = false;
    std::vector<CandidateModelTruthTrace> p2Candidates;
    std::size_t selectedP2Trajectory = std::numeric_limits<std::size_t>::max();
};

struct DeathCausalSnapshot {
    bool valid = false;
    bool falseSafe = false;
    std::uint64_t deathSampleID = 0;
    InputAction lastSelectedInput = InputAction::SafeStop;
    std::string lastSelectedLabel = "NONE";
    solver::TrajectoryClass lastSelectedClass = solver::TrajectoryClass::Unknown;
    bool predictedDeath = false;
    std::vector<SolverModelTruthSample> history;
};

struct ActionCountdownTrace {
    bool active = false;
    bool fired = false;
    std::uint64_t targetSampleID = 0;
    std::size_t dueIn = 0;
    std::string label = "NONE";
};

struct AutonomousDecision {
    bool enabled = false;
    bool active = false;
    InputOwnership ownership = InputOwnership::User;
    InputAction action = InputAction::NoPress;
    std::string reason = "AUTOBOT DISABLED";
    std::size_t targetPrimitiveIndex = world::kInvalidPrimitiveIndex;
    int targetObjectID = 0;
    double targetDistance = 0.0;

    std::size_t falseSafeTotal = 0;
    bool falseSafeDetected = false;
    DeathCausalSnapshot deathSnapshot{};
    ActionCountdownTrace countdown{};
    ActionCountdownTrace p2Countdown{};
    InputAction p2Action = InputAction::NoPress;

    presolve::PreRunStage preRunStage = presolve::PreRunStage::Idle;
    std::string preRunReason = "NOT PREPARED";
    bool fullPolicyReplayPassed = false;
    double simulatedCompletion = 0.0;

    double liveSearchElapsed = 0.0;
    double liveSearchExpansionsPerSecond = 0.0;
    std::size_t liveSearchExpansions = 0;
    std::size_t liveSearchEngineSteps = 0;
    std::size_t liveSearchFrontier = 0;
    std::size_t liveSearchReroots = 0;
    double liveSearchBestProgress = 0.0;
    bool liveGlobalPolicyAvailable = false;

    solver::PlanDecision plan{};
};

class AutonomousTestDriver final {
public:
    [[nodiscard]] AutonomousDecision decide(
        core::GameSnapshot const& snapshot,
        world::CollisionWorld const& collisionWorld,
        world::CollisionQueryResult const& query,
        bool enabled,
        bool botHolding,
        bool botHoldingP2 = false,
        std::optional<bool> globalDesiredHold = std::nullopt
    );

    static bool applyGlobalPolicyGuidance(
        solver::PlanDecision& plan,
        bool globalDesiredHold,
        bool botHolding
    );

    bool configureWorld(
        world::StaticWorld const& source,
        world::CollisionWorld const& collisionWorld
    );

    bool preparePreRun(
        core::GameSnapshot const& initialSnapshot,
        world::StaticWorld const& source,
        world::CollisionWorld const& collisionWorld,
        presolve::PreRunSolver::StageCallback const& onStage = {}
    );

    [[nodiscard]] presolve::PreRunSolver const& preRunSolver() const { return m_preRun; }

    [[nodiscard]] solver::PhysicsValidationHarness const& validation() const {
        return m_validation;
    }
    [[nodiscard]] std::size_t falseSafeTotal() const { return m_falseSafeTotal; }

private:
    static SolverModelTruthSample makeTruthSample(
        core::GameSnapshot const& snapshot,
        solver::PlanDecision const& plan
    );
    DeathCausalSnapshot buildDeathSnapshot(core::GameSnapshot const& snapshot);
    void pushTruthSample(SolverModelTruthSample sample);

    struct PendingActionCountdown {
        bool active = false;
        std::uint64_t targetSampleID = 0;
        bool targetHold = false;
        std::string originLabel = "NONE";
    };

    struct HoldTransition {
        bool valid = false;
        std::size_t delay = 0;
        bool targetHold = false;
    };

    [[nodiscard]] static HoldTransition firstHoldTransition(
        solver::ActionCandidate const& candidate,
        bool botHolding
    );
    [[nodiscard]] static std::size_t findCountdownTrajectory(
        solver::PlanDecision const& plan,
        bool botHolding,
        std::size_t dueIn,
        bool targetHold
    );
    static void selectTrajectoryForCurrentSample(
        solver::PlanDecision& plan,
        std::size_t trajectoryIndex,
        bool botHolding
    );
    void applyActionCountdown(
        core::GameSnapshot const& snapshot,
        bool botHolding,
        AutonomousDecision& decision
    );
    void clearActionCountdown();
    void applyP2ActionCountdown(
        core::GameSnapshot const& snapshot,
        bool botHoldingP2,
        AutonomousDecision& decision
    );

    solver::PhysicsValidationHarness m_validation{};
    solver::PhysicsValidationHarness m_validationP2{};
    solver::RealtimePlanner m_planner{};
    presolve::PreRunSolver m_preRun{};
    world::DynamicWorldModel m_dynamicWorld{};
    world::TriggerWorldModel m_triggerWorld{};

    InputAction m_previousAction = InputAction::SafeStop;
    bool m_previousDesiredHold = false;
    InputAction m_previousActionP2 = InputAction::SafeStop;
    bool m_previousDesiredHoldP2 = false;
    bool m_hasPredictedNextState = false;
    solver::SimState m_predictedNextState{};
    bool m_hasPredictedNextStateP2 = false;
    solver::SimState m_predictedNextStateP2{};

    std::deque<SolverModelTruthSample> m_truthHistory{};
    std::size_t m_falseSafeTotal = 0;
    bool m_previousSnapshotValid = false;
    bool m_previousDead = false;
    PendingActionCountdown m_actionCountdown{};
    PendingActionCountdown m_actionCountdownP2{};
};

[[nodiscard]] char const* toString(InputOwnership value);
[[nodiscard]] char const* toString(InputAction value);

} // namespace autobot::control
