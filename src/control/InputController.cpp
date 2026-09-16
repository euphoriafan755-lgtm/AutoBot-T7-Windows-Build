#include "autobot/control/InputController.hpp"

#include <Geode/Geode.hpp>

using namespace geode::prelude;

namespace autobot::control {

bool InputController::queueJump(
    PlayLayer* playLayer,
    bool push,
    bool player2,
    double timestamp
) {
    if (!playLayer) return false;
    playLayer->queueButton(
        static_cast<int>(PlayerButton::Jump),
        push,
        player2,
        timestamp
    );
    return true;
}

void InputController::setBotControl(bool enabled, PlayLayer* playLayer, double timestamp) {
    if (!enabled) {
        releasePlayer(false, playLayer, timestamp);
        releasePlayer(true, playLayer, timestamp);
        m_ownership = InputOwnership::User;
        return;
    }
    m_ownership = InputOwnership::Bot;
}

bool InputController::applyInternal(
    InputAction action,
    bool player2,
    PlayLayer* playLayer,
    double timestamp
) {
    auto& holding = player2 ? m_botHoldingP2 : m_botHoldingP1;
    auto& lastTransition = player2 ? m_lastTransitionP2 : m_lastTransitionP1;
    auto& lastQueueInvoked = player2 ? m_lastQueueInvokedP2 : m_lastQueueInvokedP1;
    auto& lastQueueSucceeded = player2 ? m_lastQueueSucceededP2 : m_lastQueueSucceededP1;
    auto& lastQueuePush = player2 ? m_lastQueuePushP2 : m_lastQueuePushP1;

    lastTransition = transition(holding, action);
    lastQueueInvoked = false;
    lastQueueSucceeded = true;
    lastQueuePush = false;

    if (m_ownership != InputOwnership::Bot || !playLayer) {
        lastQueueSucceeded = false;
        return false;
    }

    bool ok = true;
    if (lastTransition.emitPress) {
        lastQueueInvoked = true;
        lastQueuePush = true;
        ok = queueJump(playLayer, true, player2, timestamp) && ok;
    }
    if (lastTransition.emitRelease) {
        lastQueueInvoked = true;
        lastQueuePush = false;
        ok = queueJump(playLayer, false, player2, timestamp) && ok;
    }
    lastQueueSucceeded = ok;
    holding = lastTransition.nextHolding;
    return ok;
}

bool InputController::apply(InputAction action, PlayLayer* playLayer, double timestamp) {
    return applyInternal(action, false, playLayer, timestamp);
}

bool InputController::applyForPlayer(
    InputAction action,
    bool player2,
    PlayLayer* playLayer,
    double timestamp
) {
    return applyInternal(action, player2, playLayer, timestamp);
}

bool InputController::applyJoint(
    InputAction p1,
    InputAction p2,
    PlayLayer* playLayer,
    double timestamp
) {
    if (m_ownership != InputOwnership::Bot || !playLayer) return false;
    const bool ok1 = applyInternal(p1, false, playLayer, timestamp);
    const bool ok2 = applyInternal(p2, true, playLayer, timestamp);
    if (p1 == InputAction::SafeStop && p2 == InputAction::SafeStop) {
        m_ownership = InputOwnership::None;
    }
    return ok1 && ok2;
}

void InputController::releasePlayer(bool player2, PlayLayer* playLayer, double timestamp) {
    auto& holding = player2 ? m_botHoldingP2 : m_botHoldingP1;
    auto& lastTransition = player2 ? m_lastTransitionP2 : m_lastTransitionP1;
    auto& lastQueueInvoked = player2 ? m_lastQueueInvokedP2 : m_lastQueueInvokedP1;
    auto& lastQueueSucceeded = player2 ? m_lastQueueSucceededP2 : m_lastQueueSucceededP1;
    auto& lastQueuePush = player2 ? m_lastQueuePushP2 : m_lastQueuePushP1;

    lastTransition = transition(holding, InputAction::Release);
    lastQueueInvoked = false;
    lastQueueSucceeded = true;
    lastQueuePush = false;
    if (holding && playLayer) {
        lastQueueInvoked = true;
        lastQueueSucceeded = queueJump(playLayer, false, player2, timestamp);
    }
    holding = false;
}

void InputController::release(PlayLayer* playLayer, double timestamp) {
    releasePlayer(false, playLayer, timestamp);
}

} // namespace autobot::control
