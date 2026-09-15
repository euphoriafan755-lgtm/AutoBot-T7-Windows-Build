#pragma once

#include "autobot/control/AutonomousTestDriver.hpp"
#include "autobot/core/GameSnapshot.hpp"

class PlayLayer;

namespace cocos2d {
class CCLabelBMFont;
}

namespace autobot::ui {

class AutonomousHUD final {
public:
    bool attach(PlayLayer* playLayer);
    void update(
        core::GameSnapshot const& snapshot,
        control::AutonomousDecision const& decision
    );

    [[nodiscard]] bool attached() const { return m_label != nullptr; }

private:
    cocos2d::CCLabelBMFont* m_label = nullptr;
};

} // namespace autobot::ui
