#pragma once

#include "autobot/control/InputController.hpp"
#include "autobot/core/GameSnapshot.hpp"
#include "autobot/solver/PhysicsValidationHarness.hpp"
#include "autobot/solver/RealtimePlanner.hpp"
#include "autobot/world/CollisionWorld.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
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

    solver::PlanDecision plan{};
};

class AutonomousTestDriver final {
public:
    [[nodiscard]] AutonomousDecision decide(
        core::GameSnapshot const& snapshot,
        world::CollisionWorld const& collisionWorld,
        world::CollisionQueryResult const& query,
        bool enabled,
        bool botHolding
    );

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

    solver::PhysicsValidationHarness m_validation{};
    solver::RealtimePlanner m_planner{};

    InputAction m_previousAction = InputAction::SafeStop;
    bool m_previousDesiredHold = false;
    bool m_hasPredictedNextState = false;
    solver::SimState m_predictedNextState{};

    std::deque<SolverModelTruthSample> m_truthHistory{};
    std::size_t m_falseSafeTotal = 0;
    bool m_previousSnapshotValid = false;
    bool m_previousDead = false;
    PendingActionCountdown m_actionCountdown{};
};

[[nodiscard]] char const* toString(InputOwnership value);
[[nodiscard]] char const* toString(InputAction value);

} // namespace autobot::control
