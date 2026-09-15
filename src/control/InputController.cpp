#include "autobot/control/InputController.hpp"

#include <Geode/Geode.hpp>

using namespace geode::prelude;

namespace autobot::control {

bool InputController::queueJump(PlayLayer* playLayer, bool push, double timestamp) {
    if (!playLayer) return false;

    // Geometry Dash's native gameplay input queue. No OS input injection,
    // Sleep(), macro playback, position mutation, noclip, or physics edits.
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
    if (m_ownership != InputOwnership::Bot || !playLayer) return false;

    const auto t = transition(m_botHolding, action);
    bool ok = true;
    if (t.emitPress) ok = queueJump(playLayer, true, timestamp) && ok;
    if (t.emitRelease) ok = queueJump(playLayer, false, timestamp) && ok;
    m_botHolding = t.nextHolding;

    if (action == InputAction::SafeStop) {
        m_ownership = InputOwnership::None;
    }
    return ok;
}

void InputController::release(PlayLayer* playLayer, double timestamp) {
    if (m_botHolding && playLayer) {
        queueJump(playLayer, false, timestamp);
    }
    m_botHolding = false;
}

} // namespace autobot::control
