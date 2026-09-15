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
    void release(PlayLayer* playLayer, double timestamp);

    [[nodiscard]] bool botHolding() const { return m_botHolding; }
    [[nodiscard]] InputOwnership ownership() const { return m_ownership; }

private:
    bool queueJump(PlayLayer* playLayer, bool push, double timestamp);

    bool m_botHolding = false;
    InputOwnership m_ownership = InputOwnership::User;
};

} // namespace autobot::control
