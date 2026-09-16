#include "autobot/control/AutonomousTestDriver.hpp"

namespace autobot::control {

char const* toString(InputOwnership value) {
    switch (value) {
        case InputOwnership::User: return "USER";
        case InputOwnership::Bot: return "BOT";
        case InputOwnership::None: return "NONE";
    }
    return "NONE";
}

char const* toString(InputAction value) {
    switch (value) {
        case InputAction::NoPress: return "NO PRESS";
        case InputAction::Press: return "PRESS";
        case InputAction::Hold: return "HOLD";
        case InputAction::Release: return "RELEASE";
        case InputAction::SafeStop: return "SAFE STOP";
    }
    return "SAFE STOP";
}

AutonomousDecision AutonomousTestDriver::decide(
    core::GameSnapshot const& snapshot,
    world::CollisionWorld const& collisionWorld,
    world::CollisionQueryResult const& query,
    bool enabled,
    bool botHolding
) {
    (void)query;

    if (m_hasPredictedNextState && snapshot.valid) {
        m_validation.recordPrediction(m_predictedNextState, snapshot);
    }

    m_validation.observe(snapshot, m_previousAction, m_previousDesiredHold);

    AutonomousDecision decision{};
    decision.enabled = enabled;

    if (!enabled) {
        decision.active = false;
        decision.ownership = InputOwnership::User;
        decision.action = InputAction::SafeStop;
        decision.reason = "AUTOBOT DISABLED";
        m_previousAction = InputAction::SafeStop;
        m_previousDesiredHold = false;
        m_hasPredictedNextState = false;
        return decision;
    }

    decision.plan = m_planner.plan(
        snapshot,
        collisionWorld,
        m_validation,
        botHolding
    );

    decision.action = decision.plan.inputAction;
    decision.reason = decision.plan.reason;
    decision.targetPrimitiveIndex = decision.plan.targetPrimitiveIndex;
    decision.targetObjectID = decision.plan.targetObjectID;
    decision.targetDistance = decision.plan.targetDistance;
    decision.active = decision.plan.active;
    decision.ownership = decision.active ? InputOwnership::Bot : InputOwnership::None;

    m_previousAction = decision.action;
    m_previousDesiredHold = decision.action == InputAction::Press
        || decision.action == InputAction::Hold;

    if (decision.plan.hasPredictedNextState) {
        m_predictedNextState = decision.plan.predictedNextState;
        m_hasPredictedNextState = true;
    } else {
        m_hasPredictedNextState = false;
    }

    if (snapshot.player.dead) {
        m_previousAction = InputAction::SafeStop;
        m_previousDesiredHold = false;
        m_hasPredictedNextState = false;
    }

    return decision;
}

} // namespace autobot::control
