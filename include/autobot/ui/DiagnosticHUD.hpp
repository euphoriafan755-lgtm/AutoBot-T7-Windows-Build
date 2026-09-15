#pragma once

#include "autobot/core/GameSnapshot.hpp"
#include "autobot/world/WorldObject.hpp"

class PlayLayer;

namespace cocos2d {
class CCLabelBMFont;
}

namespace autobot::ui {

class DiagnosticHUD final {
public:
    bool attach(PlayLayer* playLayer);
    void update(core::GameSnapshot const& snapshot, world::StaticWorld const& world);
    [[nodiscard]] bool attached() const { return m_label != nullptr; }

private:
    cocos2d::CCLabelBMFont* m_label = nullptr;
};

} // namespace autobot::ui
