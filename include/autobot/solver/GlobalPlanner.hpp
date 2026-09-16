#pragma once

#include "autobot/core/GameSnapshot.hpp"
#include "autobot/solver/AdaptiveSearchBudget.hpp"
#include "autobot/solver/GeneralWorldModel.hpp"
#include "autobot/world/CollisionWorld.hpp"

#include <cstddef>
#include <vector>

namespace autobot::solver {

struct GlobalRouteStep {
    double xNear = 0.0;
    double xFar = 0.0;
    double yMin = 0.0;
    double yMax = 0.0;
    double centerY = 0.0;
    double clearance = 0.0;
};

struct GlobalRoutePlan {
    bool valid = false;
    bool uncertain = false;
    double direction = 1.0;
    double targetX = 0.0;
    double targetY = 0.0;
    double targetYMin = 0.0;
    double targetYMax = 0.0;
    double plannedForwardDistance = 0.0;
    double minimumCorridorClearance = 0.0;
    std::size_t nextPortalPrimitive = world::kInvalidPrimitiveIndex;
    std::size_t exploredStates = 0;
    std::size_t branchesConsidered = 0;
    std::vector<GlobalRouteStep> steps;
};

class GlobalPlanner final {
public:
    [[nodiscard]] GlobalRoutePlan plan(
        core::GameSnapshot const& snapshot,
        GeneralWorldView const& worldView,
        world::CollisionWorld const& collisionWorld,
        SearchBudget const& budget
    ) const;
};

} // namespace autobot::solver
