#pragma once

#include "autobot/core/GameSnapshot.hpp"
#include "autobot/world/CollisionTrace.hpp"
#include "autobot/world/CollisionWorld.hpp"

#include <cstddef>
#include <vector>

class PlayLayer;

namespace cocos2d {
class CCDrawNode;
class CCLabelBMFont;
struct _ccColor4F;
}

namespace autobot::ui {

struct CollisionRenderStats {
    world::WorldRect visibleWorldRect{};
    std::size_t queryReceived = 0;
    std::size_t queryDrawn = 0;
    std::size_t traceReceived = 0;
    std::size_t traceDrawn = 0;
    std::size_t fullWorldReceived = 0;
    std::size_t fullWorldDrawn = 0;
};

class CollisionDebugOverlay final {
public:
    bool attach(PlayLayer* playLayer);
    void setEnabled(bool enabled);
    void setObjectLabelsEnabled(bool enabled);
    void setFullWorldEnabled(bool enabled) { m_fullWorldEnabled = enabled; }
    void update(
        core::GameSnapshot const& snapshot,
        world::CollisionWorld const& collisionWorld,
        world::CollisionQueryResult const& query,
        world::CollisionTraceQuery* trace
    );

    [[nodiscard]] bool attached() const { return m_drawNode != nullptr && m_label != nullptr; }
    [[nodiscard]] bool enabled() const { return m_enabled; }
    [[nodiscard]] bool fullWorldEnabled() const { return m_fullWorldEnabled; }
    [[nodiscard]] CollisionRenderStats const& lastStats() const { return m_lastStats; }

private:
    void drawPrimitive(world::CollisionPrimitive const& primitive);
    void drawTraceRecord(world::CollisionTraceRecord& record);
    void drawBounds(world::WorldRect const& rect, cocos2d::_ccColor4F const& color, float thickness);
    void drawCross(float x, float y, cocos2d::_ccColor4F const& color, float radius);
    void hideObjectLabels();
    cocos2d::CCLabelBMFont* ensureObjectLabel(std::size_t index);
    [[nodiscard]] world::WorldRect visibleWorldRect() const;

    PlayLayer* m_playLayer = nullptr;
    cocos2d::CCDrawNode* m_drawNode = nullptr;
    cocos2d::CCLabelBMFont* m_label = nullptr;
    std::vector<cocos2d::CCLabelBMFont*> m_objectLabels;
    bool m_enabled = false;
    bool m_objectLabelsEnabled = false;
    bool m_fullWorldEnabled = false;
    CollisionRenderStats m_lastStats{};
};

} // namespace autobot::ui
