#pragma once

class PlayLayer;

namespace autobot::control {

enum class InputOwnership {
    User,
    Bot,
    None,
};

enum class InputAction {
    NoPress,
    Press,
    Hold,
    Release,
    SafeStop,
};

struct InputStateTransition {
    InputAction effectiveAction = InputAction::NoPress;
    bool emitPress = false;
    bool emitRelease = false;
    bool nextHolding = false;
};

class InputController final {
public:
    [[nodiscard]] static constexpr InputStateTransition transition(
        bool currentlyHolding,
        InputAction requested
    ) {
        InputStateTransition t{};
        t.effectiveAction = requested;
        t.nextHolding = currentlyHolding;

        switch (requested) {
            case InputAction::Press:
                if (!currentlyHolding) {
                    t.emitPress = true;
                    t.nextHolding = true;
                } else {
                    t.effectiveAction = InputAction::Hold;
                }
                break;
            case InputAction::Hold:
            case InputAction::NoPress:
                break;
            case InputAction::Release:
                if (currentlyHolding) t.emitRelease = true;
                t.nextHolding = false;
                break;
            case InputAction::SafeStop:
                if (currentlyHolding) t.emitRelease = true;
                t.nextHolding = false;
                break;
        }
        return t;
    }

    void setBotControl(bool enabled, PlayLayer* playLayer, double timestamp);
    bool apply(InputAction action, PlayLayer* playLayer, double timestamp);
    bool applyForPlayer(InputAction action, bool player2, PlayLayer* playLayer, double timestamp);
    bool applyJoint(InputAction p1, InputAction p2, PlayLayer* playLayer, double timestamp);
    void release(PlayLayer* playLayer, double timestamp);
    void releasePlayer(bool player2, PlayLayer* playLayer, double timestamp);

    [[nodiscard]] bool botHolding() const { return m_botHoldingP1; }
    [[nodiscard]] bool botHoldingP2() const { return m_botHoldingP2; }
    [[nodiscard]] InputOwnership ownership() const { return m_ownership; }
    [[nodiscard]] InputStateTransition const& lastTransition() const { return m_lastTransitionP1; }
    [[nodiscard]] InputStateTransition const& lastTransitionP2() const { return m_lastTransitionP2; }
    [[nodiscard]] bool lastQueueInvoked() const { return m_lastQueueInvokedP1; }
    [[nodiscard]] bool lastQueueSucceeded() const { return m_lastQueueSucceededP1; }
    [[nodiscard]] bool lastQueuePush() const { return m_lastQueuePushP1; }
    [[nodiscard]] bool lastQueueInvokedP2() const { return m_lastQueueInvokedP2; }
    [[nodiscard]] bool lastQueueSucceededP2() const { return m_lastQueueSucceededP2; }
    [[nodiscard]] bool lastQueuePushP2() const { return m_lastQueuePushP2; }

private:
    bool queueJump(PlayLayer* playLayer, bool push, bool player2, double timestamp);
    bool applyInternal(InputAction action, bool player2, PlayLayer* playLayer, double timestamp);

    bool m_botHoldingP1 = false;
    bool m_botHoldingP2 = false;
    InputOwnership m_ownership = InputOwnership::User;
    InputStateTransition m_lastTransitionP1{};
    InputStateTransition m_lastTransitionP2{};
    bool m_lastQueueInvokedP1 = false;
    bool m_lastQueueSucceededP1 = false;
    bool m_lastQueuePushP1 = false;
    bool m_lastQueueInvokedP2 = false;
    bool m_lastQueueSucceededP2 = false;
    bool m_lastQueuePushP2 = false;
};

} // namespace autobot::control
