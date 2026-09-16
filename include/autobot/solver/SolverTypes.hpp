#pragma once

#include "autobot/control/InputController.hpp"
#include "autobot/core/GameSnapshot.hpp"
#include "autobot/world/CollisionTypes.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <cmath>
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

enum class TrajectoryClass {
    Safe,
    Risky,
    Collision,
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

    double sampleDt = 0.0;
    double rawGravity = 0.0;
    double gravityModifier = 1.0;
    double jumpAcceleration = 0.0;
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

    std::size_t collisionPrimitive = world::kInvalidPrimitiveIndex;
    std::size_t portalPrimitive = world::kInvalidPrimitiveIndex;
    core::GameMode finalMode = core::GameMode::Unknown;

    double progress = 0.0;
    double minimumClearance = std::numeric_limits<double>::infinity();
    double score = -std::numeric_limits<double>::infinity();
    double confidence = 0.0;
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

struct PlanDecision {
    SolverStatus status = SolverStatus::Waiting;
    control::InputAction inputAction = control::InputAction::SafeStop;

    bool gameStateReady = false;
    bool worldReady = false;
    bool physicsReady = false;
    bool plannerReady = false;
    bool active = false;

    std::string reason = "AUTOBOT WAITING FOR READY";
    double confidence = 0.0;

    std::size_t targetPrimitiveIndex = world::kInvalidPrimitiveIndex;
    int targetObjectID = 0;
    double targetDistance = 0.0;

    std::vector<TrajectoryResult> trajectories;
    std::size_t selectedTrajectory = std::numeric_limits<std::size_t>::max();

    ModelError lastModelError{};
    bool hasPredictedNextState = false;
    SimState predictedNextState{};

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
        case TrajectoryClass::Unknown: return "UNKNOWN";
    }
    return "UNKNOWN";
}

} // namespace autobot::solver
