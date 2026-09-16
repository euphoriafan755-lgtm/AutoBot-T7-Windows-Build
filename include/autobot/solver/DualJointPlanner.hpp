#pragma once

#include "autobot/core/GameSnapshot.hpp"
#include "autobot/solver/ActionGenerator.hpp"
#include "autobot/solver/AdaptiveSearchBudget.hpp"
#include "autobot/solver/GeneralWorldModel.hpp"
#include "autobot/solver/GlobalPlanner.hpp"
#include "autobot/solver/LocalWorldView.hpp"
#include "autobot/solver/PhysicsValidationHarness.hpp"
#include "autobot/solver/TrajectoryScorer.hpp"
#include "autobot/solver/TrajectorySimulator.hpp"
#include "autobot/world/CollisionWorld.hpp"
#include "autobot/world/DynamicWorldModel.hpp"
#include "autobot/world/TriggerWorldModel.hpp"

#include <cstddef>
#include <limits>
#include <vector>

namespace autobot::solver {

struct JointTrajectoryPair {
    std::size_t p1Index = std::numeric_limits<std::size_t>::max();
    std::size_t p2Index = std::numeric_limits<std::size_t>::max();
    bool fatalCollision = true;
    bool horizonConclusive = false;
    bool globalCompatible = false;
    int safetyTier = 0;
    double score = -std::numeric_limits<double>::infinity();
};

struct DualPlanResult {
    bool ready = false;
    bool active = false;
    control::InputAction p1Action = control::InputAction::SafeStop;
    control::InputAction p2Action = control::InputAction::SafeStop;
    std::vector<TrajectoryResult> p1Trajectories;
    std::vector<TrajectoryResult> p2Trajectories;
    std::vector<JointTrajectoryPair> pairs;
    std::size_t selectedPair = std::numeric_limits<std::size_t>::max();
    std::size_t selectedP1 = std::numeric_limits<std::size_t>::max();
    std::size_t selectedP2 = std::numeric_limits<std::size_t>::max();
    bool hasPredictedP1 = false;
    bool hasPredictedP2 = false;
    SimState predictedP1{};
    SimState predictedP2{};
};

class DualJointPlanner final {
public:
    [[nodiscard]] DualPlanResult plan(
        core::GameSnapshot const& snapshot,
        world::CollisionWorld const& collisionWorld,
        PhysicsValidationHarness const& p1Validation,
        PhysicsValidationHarness const& p2Validation,
        bool p1Holding,
        bool p2Holding,
        world::DynamicWorldModel const& dynamicWorld,
        world::TriggerWorldModel const* triggerWorld,
        SearchBudget const& budget
    ) const;

    [[nodiscard]] static int safetyTier(
        TrajectoryResult const& p1,
        TrajectoryResult const& p2,
        bool globalCompatible
    );

    [[nodiscard]] static JointTrajectoryPair selectBestPair(
        std::vector<TrajectoryResult> const& p1,
        std::vector<TrajectoryResult> const& p2
    );

private:
    [[nodiscard]] static control::InputAction firstInput(
        ActionCandidate const& candidate,
        bool holding
    );
    [[nodiscard]] static core::GameSnapshot playerSnapshot(
        core::GameSnapshot const& joint,
        bool player2
    );
    [[nodiscard]] static double requiredDistance(
        core::GameSnapshot const& snapshot,
        LocalWorldView const& local,
        ModeCalibration const& calibration,
        std::size_t horizon
    );
    [[nodiscard]] static bool routeCompatible(
        TrajectoryResult& trajectory,
        GlobalRoutePlan const& route
    );

    LocalWorldViewBuilder m_localWorldBuilder{};
    GeneralWorldModelBuilder m_generalWorldBuilder{};
    GlobalPlanner m_globalPlanner{};
    ModeActionGenerator m_actionGenerator{};
    TrajectorySimulator m_simulator{};
    TrajectoryScorer m_scorer{};
};

} // namespace autobot::solver
