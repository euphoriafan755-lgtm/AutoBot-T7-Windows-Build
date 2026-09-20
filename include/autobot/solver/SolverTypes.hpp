#pragma once

#include "autobot/control/InputController.hpp"
#include "autobot/core/GameSnapshot.hpp"
#include "autobot/world/CollisionTypes.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

namespace autobot::solver {

enum class SolverStatus {
    Waiting,
    Active,
    NoSafePath,
    Stopped,
};

enum class PhysicsModelStatus {
    Provisional,
    Calibrating,
    Verified,
};

inline constexpr std::string_view toString(PhysicsModelStatus value) {
    switch (value) {
        case PhysicsModelStatus::Provisional: return "PROVISIONAL";
        case PhysicsModelStatus::Calibrating: return "CALIBRATING";
        case PhysicsModelStatus::Verified: return "VERIFIED";
    }
    return "PROVISIONAL";
}

enum class TrajectoryClass {
    Safe,
    Risky,
    Collision,
    HorizonInconclusive,
    Unknown,
};

struct ActionSegment {
    bool hold = false;
    std::uint16_t ticks = 1;
};

struct ActionCandidate {
    std::string label;
    std::vector<ActionSegment> segments;

    [[nodiscard]] bool desiredHoldAt(std::size_t tick) const {
        std::size_t cursor = 0;
        for (auto const& segment : segments) {
            const auto end = cursor + static_cast<std::size_t>(segment.ticks);
            if (tick < end) return segment.hold;
            cursor = end;
        }
        return segments.empty() ? false : segments.back().hold;
    }

    [[nodiscard]] std::size_t transitionCount() const {
        if (segments.empty()) return 0;
        std::size_t count = 0;
        bool previous = segments.front().hold;
        for (std::size_t i = 1; i < segments.size(); ++i) {
            if (segments[i].hold != previous) ++count;
            previous = segments[i].hold;
        }
        return count;
    }
};

struct SimState {
    double x = 0.0;
    double y = 0.0;
    double vx = 0.0;
    double vy = 0.0;

    core::GameMode mode = core::GameMode::Unknown;
    bool mini = false;
    bool grounded = false;
    bool upsideDown = false;
    bool holding = false;
    bool alive = true;

    double objectWidth = 30.0;
    double objectHeight = 30.0;

    // sampleDt is normalized GD physics time, not wall-clock seconds.
    double sampleDt = 0.0;
    double verticalPositionScale = 0.9;
    double rawGravity = 0.0;
    double gravityModifier = 1.0;
    double jumpVelocity = 0.0;
    double speedScalar = 0.0;
};

struct TrajectoryPoint {
    double x = 0.0;
    double y = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    core::GameMode mode = core::GameMode::Unknown;

    bool collision = false;
    bool hazard = false;
    bool landing = false;
    bool portal = false;
    bool modeChange = false;

    world::WorldRect playerBounds{};
    world::WorldRect nearestHazardBounds{};
    double nearestHazardDistance = std::numeric_limits<double>::infinity();
    bool nearestHazardIntersects = false;
};

struct TrajectoryResult {
    ActionCandidate candidate;
    std::vector<TrajectoryPoint> points;

    TrajectoryClass classification = TrajectoryClass::Unknown;
    bool fatalCollision = false;
    bool hazardCollision = false;
    bool landed = false;
    bool portalCrossed = false;
    bool modeChanged = false;
    bool uncertainGeometry = false;
    bool horizonConclusive = false;
    bool relevantEventReached = false;
    bool nearestHazardSeen = false;

    std::size_t collisionPrimitive = world::kInvalidPrimitiveIndex;
    std::size_t collisionTick = std::numeric_limits<std::size_t>::max();
    std::size_t portalPrimitive = world::kInvalidPrimitiveIndex;
    std::size_t nearestHazardPrimitive = world::kInvalidPrimitiveIndex;
    core::GameMode finalMode = core::GameMode::Unknown;

    std::size_t horizonTicks = 0;
    std::size_t simulatedTicks = 0;
    double requiredForwardDistance = 0.0;
    double progress = 0.0;
    double minimumClearance = std::numeric_limits<double>::infinity();
    double predictedFinalX = 0.0;
    double predictedFinalY = 0.0;
    double score = -std::numeric_limits<double>::infinity();
    double confidence = 0.0;

    // Global-route compatibility is evaluated by RealtimePlanner after the
    // trajectory is simulated. It is a feasibility signal, not a level-
    // specific score bonus.
    bool globalRouteCompatible = false;
    double globalRouteErrorY = std::numeric_limits<double>::infinity();
};

struct LocalWorldObject {
    std::size_t primitiveIndex = world::kInvalidPrimitiveIndex;
    world::GameplayObjectType classification = world::GameplayObjectType::Unknown;
    world::WorldRect bounds{};

    double forwardDistance = 0.0;
    double verticalDistance = 0.0;
    bool potentiallyDynamic = false;
    bool verifiedGeometry = false;
};

struct LocalWorldView {
    world::WorldRect queryRect{};
    std::vector<LocalWorldObject> solids;
    std::vector<LocalWorldObject> hazards;
    std::vector<LocalWorldObject> portals;
    std::vector<LocalWorldObject> unknown;

    double horizonX = 0.0;
    double horizonY = 0.0;
    bool complete = false;
};

struct ModelError {
    double dx = 0.0;
    double dy = 0.0;
    double dvx = 0.0;
    double dvy = 0.0;
    bool modeMatched = true;
    bool landingMatched = true;
    bool collisionMatched = true;

    [[nodiscard]] double magnitude() const {
        return std::abs(dx) + std::abs(dy) + std::abs(dvx) + std::abs(dvy);
    }
};

struct ModelTruthDiagnostics {
    world::WorldRect localQueryRect{};

    double realDtSeconds = 0.0;
    double simDtNormalized = 0.0;
    double physicsTicksPerSecond = 0.0;
    double rawVelocityX = 0.0;
    double observedWorldVelocityX = 0.0;
    double observedWorldVelocityY = 0.0;
    double verticalPositionScale = 0.0;

    std::size_t horizonTicks = 0;
    double predictedHorizonSeconds = 0.0;
    double requiredForwardDistance = 0.0;

    std::size_t nearestHazardPrimitive = world::kInvalidPrimitiveIndex;
    std::size_t nearestHazardSourceIndex = world::kInvalidPrimitiveIndex;
    int nearestHazardObjectID = 0;
    int nearestHazardUniqueID = 0;
    world::WorldRect nearestHazardBounds{};
    world::GeometryVerification nearestHazardGeometry = world::GeometryVerification::NotSupported;
    double nearestHazardDistance = std::numeric_limits<double>::infinity();
    double timeToHazardSeconds = std::numeric_limits<double>::infinity();
    double samplesToHazard = std::numeric_limits<double>::infinity();

    bool hazardExistsInCollisionWorld = false;
    bool hazardClassifiedHazard = false;
    bool hazardPrimitiveExists = false;
    bool hazardHashIndexed = false;
    bool hazardInsideLocalQueryRect = false;
    bool hazardLocalWorldContains = false;
    bool hazardBruteForceContains = false;
    bool hazardEnabled = false;
    bool hazardIndexable = false;
    bool hazardNoTouch = false;
    std::size_t hazardHashCellCount = 0;
};


struct GlobalPlanDiagnostics {
    bool valid = false;
    bool uncertain = false;
    double targetX = 0.0;
    double targetY = 0.0;
    double targetYMin = 0.0;
    double targetYMax = 0.0;
    double plannedForwardDistance = 0.0;
    double minimumCorridorClearance = 0.0;
    double observedComplexity = 0.0;
    double globalLookaheadX = 0.0;
    double globalLookaheadY = 0.0;
    std::size_t routeSteps = 0;
    std::size_t exploredStates = 0;
    std::size_t branchesConsidered = 0;
    std::size_t nextPortalPrimitive = world::kInvalidPrimitiveIndex;
};

struct PlanDecision {
    SolverStatus status = SolverStatus::Waiting;
    control::InputAction inputAction = control::InputAction::SafeStop;

    bool gameStateReady = false;
    bool worldReady = false;
    bool physicsReady = false;
    PhysicsModelStatus physicsModelStatus = PhysicsModelStatus::Provisional;
    bool plannerReady = false;
    bool active = false;

    std::string reason = "AUTOBOT WAITING FOR READY";
    double confidence = 0.0;

    std::size_t targetPrimitiveIndex = world::kInvalidPrimitiveIndex;
    int targetObjectID = 0;
    double targetDistance = 0.0;

    std::vector<TrajectoryResult> trajectories;
    std::size_t selectedTrajectory = std::numeric_limits<std::size_t>::max();

    // Joint Dual planning keeps both players in the same PlanDecision. P1 uses
    // the existing fields; these fields carry the independently simulated P2
    // trajectory and legal input selected by the joint search.
    bool dualMode = false;
    control::InputAction p2InputAction = control::InputAction::NoPress;
    std::vector<TrajectoryResult> p2Trajectories;
    std::size_t selectedP2Trajectory = std::numeric_limits<std::size_t>::max();
    std::size_t jointPairsEvaluated = 0;

    ModelError lastModelError{};
    ModelTruthDiagnostics modelTruth{};
    GlobalPlanDiagnostics globalPlan{};
    bool hasPredictedNextState = false;
    SimState predictedNextState{};
    bool hasPredictedNextStateP2 = false;
    SimState predictedNextStateP2{};

    double candidateGenerationMs = 0.0;
    double physicsSimulationMs = 0.0;
    double trajectoryScoringMs = 0.0;
    double plannerDurationMs = 0.0;
};

inline constexpr std::string_view toString(SolverStatus value) {
    switch (value) {
        case SolverStatus::Waiting: return "WAITING";
        case SolverStatus::Active: return "ACTIVE";
        case SolverStatus::NoSafePath: return "NO SAFE PATH";
        case SolverStatus::Stopped: return "STOPPED";
    }
    return "STOPPED";
}

inline constexpr std::string_view toString(TrajectoryClass value) {
    switch (value) {
        case TrajectoryClass::Safe: return "SAFE";
        case TrajectoryClass::Risky: return "RISKY";
        case TrajectoryClass::Collision: return "COLLISION";
        case TrajectoryClass::HorizonInconclusive: return "HORIZON INCONCLUSIVE";
        case TrajectoryClass::Unknown: return "UNKNOWN";
    }
    return "UNKNOWN";
}

} // namespace autobot::solver
