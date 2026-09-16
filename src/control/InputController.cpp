#include "autobot/control/InputController.hpp"

#include <Geode/Geode.hpp>

using namespace geode::prelude;

namespace autobot::control {

bool InputController::queueJump(PlayLayer* playLayer, bool push, double timestamp) {
    if (!playLayer) return false;

    playLayer->queueButton(
        static_cast<int>(PlayerButton::Jump),
        push,
        false,
        timestamp
    );
    return true;
}

void InputController::setBotControl(bool enabled, PlayLayer* playLayer, double timestamp) {
    if (!enabled) {
        if (m_botHolding) release(playLayer, timestamp);
        m_ownership = InputOwnership::User;
        return;
    }
    m_ownership = InputOwnership::Bot;
}

bool InputController::apply(InputAction action, PlayLayer* playLayer, double timestamp) {
    m_lastTransition = transition(m_botHolding, action);
    m_lastQueueInvoked = false;
    m_lastQueueSucceeded = true;
    m_lastQueuePush = false;

    if (m_ownership != InputOwnership::Bot || !playLayer) {
        m_lastQueueSucceeded = false;
        return false;
    }

    bool ok = true;
    if (m_lastTransition.emitPress) {
        m_lastQueueInvoked = true;
        m_lastQueuePush = true;
        ok = queueJump(playLayer, true, timestamp) && ok;
    }
    if (m_lastTransition.emitRelease) {
        m_lastQueueInvoked = true;
        m_lastQueuePush = false;
        ok = queueJump(playLayer, false, timestamp) && ok;
    }
    m_lastQueueSucceeded = ok;
    m_botHolding = m_lastTransition.nextHolding;

    if (action == InputAction::SafeStop) {
        m_ownership = InputOwnership::None;
    }
    return ok;
}

void InputController::release(PlayLayer* playLayer, double timestamp) {
    m_lastTransition = transition(m_botHolding, InputAction::Release);
    m_lastQueueInvoked = false;
    m_lastQueueSucceeded = true;
    m_lastQueuePush = false;
    if (m_botHolding && playLayer) {
        m_lastQueueInvoked = true;
        m_lastQueueSucceeded = queueJump(playLayer, false, timestamp);
    }
    m_botHolding = false;
}

} // namespace autobot::control
