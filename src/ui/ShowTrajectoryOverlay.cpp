#include "autobot/ui/ShowTrajectoryOverlay.hpp"

#include <Geode/Geode.hpp>
#include <fmt/format.h>

#include <algorithm>

using namespace geode::prelude;

namespace autobot::ui {
namespace {

ccColor4F pathColor(bool selected, solver::TrajectoryClass cls) {
    if (selected) return {1.0f, 1.0f, 1.0f, 0.95f};
    switch (cls) {
        case solver::TrajectoryClass::Safe: return {0.2f, 1.0f, 0.35f, 0.55f};
        case solver::TrajectoryClass::Risky: return {1.0f, 0.75f, 0.15f, 0.55f};
        case solver::TrajectoryClass::Collision: return {1.0f, 0.2f, 0.2f, 0.55f};
        case solver::TrajectoryClass::Unknown: return {0.75f, 0.35f, 1.0f, 0.55f};
    }
    return {1.0f, 1.0f, 1.0f, 0.55f};
}

void drawCross(CCDrawNode* node, float x, float y, float size, ccColor4F const& color) {
    if (!node) return;
    node->drawSegment({x - size, y}, {x + size, y}, 0.8f, color);
    node->drawSegment({x, y - size}, {x, y + size}, 0.8f, color);
}

} // namespace

bool ShowTrajectoryOverlay::attach(PlayLayer* playLayer) {
    if (attached()) return true;
    if (!playLayer || !playLayer->m_objectLayer) return false;

    m_drawNode = CCDrawNode::create();
    if (!m_drawNode) return false;
    m_drawNode->setID("autobot-show-trajectory"_spr);
    m_drawNode->setZOrder(100100);
    playLayer->m_objectLayer->addChild(m_drawNode, 100100);

    m_label = CCLabelBMFont::create("SHOW TRAJECTORY: WAITING", "chatFont.fnt");
    if (m_label) {
        const auto win = CCDirector::get()->getWinSize();
        m_label->setAnchorPoint({0.5f, 0.0f});
        m_label->setPosition({win.width * 0.5f, 8.0f});
        m_label->setScale(0.28f);
        m_label->setZOrder(1000000);
        m_label->setOpacity(225);
        m_label->setID("autobot-show-trajectory-label"_spr);
        playLayer->addChild(m_label, 1000000);
    }

    setEnabled(m_enabled);
    return true;
}

void ShowTrajectoryOverlay::setEnabled(bool enabled) {
    m_enabled = enabled;
    if (m_drawNode) {
        m_drawNode->setVisible(enabled);
        if (!enabled) m_drawNode->clear();
    }
    if (m_label) m_label->setVisible(enabled);
}

void ShowTrajectoryOverlay::drawTrajectory(
    solver::TrajectoryResult const& trajectory,
    bool selected
) {
    if (!m_drawNode || trajectory.points.size() < 2) return;
    const auto color = pathColor(selected, trajectory.classification);
    const float thickness = selected ? 1.25f : 0.45f;

    for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
        auto const& a = trajectory.points[i - 1];
        auto const& b = trajectory.points[i];
        m_drawNode->drawSegment(
            {static_cast<float>(a.x), static_cast<float>(a.y)},
            {static_cast<float>(b.x), static_cast<float>(b.y)},
            thickness,
            color
        );

        if (b.collision) {
            drawCross(
                m_drawNode,
                static_cast<float>(b.x),
                static_cast<float>(b.y),
                selected ? 5.0f : 3.0f,
                {1.0f, 0.1f, 0.1f, 0.95f}
            );
        } else if (b.landing) {
            drawCross(
                m_drawNode,
                static_cast<float>(b.x),
                static_cast<float>(b.y),
                selected ? 4.0f : 2.5f,
                {0.2f, 1.0f, 0.9f, 0.9f}
            );
        } else if (b.portal || b.modeChange) {
            drawCross(
                m_drawNode,
                static_cast<float>(b.x),
                static_cast<float>(b.y),
                selected ? 4.5f : 2.5f,
                {0.9f, 0.35f, 1.0f, 0.9f}
            );
        }
    }

    if (selected && !trajectory.points.empty()) {
        auto const& end = trajectory.points.back();
        drawCross(
            m_drawNode,
            static_cast<float>(end.x),
            static_cast<float>(end.y),
            5.0f,
            {1.0f, 1.0f, 1.0f, 0.95f}
        );
    }
}

void ShowTrajectoryOverlay::update(solver::PlanDecision const& decision) {
    if (!m_enabled || !m_drawNode) return;
    m_drawNode->clear();

    for (std::size_t i = 0; i < decision.trajectories.size(); ++i) {
        drawTrajectory(
            decision.trajectories[i],
            i == decision.selectedTrajectory
        );
    }

    if (!m_label) return;
    if (decision.selectedTrajectory < decision.trajectories.size()) {
        auto const& selected = decision.trajectories[decision.selectedTrajectory];
        const auto text = fmt::format(
            "SELECTED: {} | {} | score {:.1f} | conf {:.2f} | clearance {:.2f}",
            selected.candidate.label,
            solver::toString(selected.classification),
            selected.score,
            selected.confidence,
            selected.minimumClearance
        );
        m_label->setString(text.c_str());
    } else {
        m_label->setString("SHOW TRAJECTORY: NO SELECTED PATH");
    }
}

} // namespace autobot::ui
