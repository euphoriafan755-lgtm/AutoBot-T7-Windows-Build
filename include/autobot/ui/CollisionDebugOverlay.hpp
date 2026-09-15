#pragma once

#include "autobot/core/GameSnapshot.hpp"
#include "autobot/world/CollisionWorld.hpp"

class PlayLayer;

namespace cocos2d {
class CCDrawNode;
class CCLabelBMFont;
struct _ccColor4F;
}

namespace autobot::ui {

class CollisionDebugOverlay final {
public:
    bool attach(PlayLayer* playLayer);
    void setEnabled(bool enabled);
    void update(
        core::GameSnapshot const& snapshot,
        world::CollisionWorld const& collisionWorld,
        world::CollisionQueryResult const& query
    );

    [[nodiscard]] bool attached() const { return m_drawNode != nullptr && m_label != nullptr; }
    [[nodiscard]] bool enabled() const { return m_enabled; }

private:
    void drawPrimitive(world::CollisionPrimitive const& primitive);
    void drawBounds(world::WorldRect const& rect, cocos2d::_ccColor4F const& color, float thickness);

    cocos2d::CCDrawNode* m_drawNode = nullptr;
    cocos2d::CCLabelBMFont* m_label = nullptr;
    bool m_enabled = false;
};

} // namespace autobot::ui
