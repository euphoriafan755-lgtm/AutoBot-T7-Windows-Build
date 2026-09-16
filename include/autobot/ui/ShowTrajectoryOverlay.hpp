#pragma once

#include "autobot/solver/SolverTypes.hpp"

class PlayLayer;

namespace cocos2d {
class CCDrawNode;
class CCLabelBMFont;
}

namespace autobot::ui {

class ShowTrajectoryOverlay final {
public:
    bool attach(PlayLayer* playLayer);
    void setEnabled(bool enabled);
    void update(solver::PlanDecision const& decision);

    [[nodiscard]] bool attached() const { return m_drawNode != nullptr; }

private:
    void drawTrajectory(
        solver::TrajectoryResult const& trajectory,
        bool selected
    );

    cocos2d::CCDrawNode* m_drawNode = nullptr;
    cocos2d::CCLabelBMFont* m_label = nullptr;
    bool m_enabled = true;
};

} // namespace autobot::ui
