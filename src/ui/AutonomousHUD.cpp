#include "autobot/ui/AutonomousHUD.hpp"

#include <Geode/Geode.hpp>
#include <fmt/format.h>

using namespace geode::prelude;

namespace autobot::ui {

bool AutonomousHUD::attach(PlayLayer* playLayer) {
    if (m_label) return true;
    if (!playLayer) return false;

    m_label = CCLabelBMFont::create("AUTOBOT: WAITING", "chatFont.fnt");
    if (!m_label) return false;

    const auto winSize = CCDirector::get()->getWinSize();
    m_label->setAnchorPoint({0.5f, 1.0f});
    m_label->setPosition({winSize.width * 0.5f, winSize.height - 8.0f});
    m_label->setScale(0.36f);
    m_label->setZOrder(1000000);
    m_label->setOpacity(235);
    m_label->setID("autobot-control-hud"_spr);
    playLayer->addChild(m_label, 1000000);
    return true;
}

void AutonomousHUD::update(
    core::GameSnapshot const& snapshot,
    control::AutonomousDecision const& decision
) {
    if (!m_label) return;

    const auto mode = snapshot.valid ? core::toString(snapshot.player.mode) : "UNKNOWN";
    std::string targetLine = "TARGET: NONE";
    if (decision.targetPrimitiveIndex != world::kInvalidPrimitiveIndex) {
        targetLine = fmt::format(
            "TARGET: PRIM {} ID {} DIST {:.2f}",
            decision.targetPrimitiveIndex,
            decision.targetObjectID,
            decision.targetDistance
        );
    }

    const auto text = fmt::format(
        "AUTOBOT: {}\n"
        "CONTROL: {}\n"
        "MODE: {}\n"
        "ACTION: {}\n"
        "REASON: {}\n"
        "{}",
        decision.active ? "ACTIVE" : "STOPPED",
        control::toString(decision.ownership),
        mode,
        control::toString(decision.action),
        decision.reason,
        targetLine
    );
    m_label->setString(text.c_str());
}

} // namespace autobot::ui
