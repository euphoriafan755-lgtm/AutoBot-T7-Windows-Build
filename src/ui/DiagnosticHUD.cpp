#include "autobot/ui/DiagnosticHUD.hpp"

#include <Geode/Geode.hpp>
#include <fmt/format.h>

using namespace geode::prelude;

namespace autobot::ui {

bool DiagnosticHUD::attach(PlayLayer* playLayer) {
    if (m_label) return true;
    if (!playLayer) return false;

    m_label = CCLabelBMFont::create("AUTOBOT T7 V0.1", "chatFont.fnt");
    if (!m_label) return false;

    const auto winSize = CCDirector::get()->getWinSize();
    m_label->setAnchorPoint({0.0f, 1.0f});
    m_label->setPosition({8.0f, winSize.height - 8.0f});
    m_label->setScale(0.42f);
    m_label->setZOrder(1000000);
    m_label->setOpacity(230);
    m_label->setID("diagnostic-hud"_spr);

    playLayer->addChild(m_label, 1000000);
    return true;
}

void DiagnosticHUD::update(core::GameSnapshot const& snapshot, world::StaticWorld const& world) {
    if (!m_label) return;

    if (!snapshot.valid) {
        m_label->setString("AUTOBOT T7 V0.1\nSTATE: INVALID");
        return;
    }

    const auto& p = snapshot.player;
    const auto text = fmt::format(
        "AUTOBOT T7 V0.1 - ZERO-SHOT CORE\n"
        "STATE READER: ACTIVE\n"
        "MODE: {}\n"
        "POS: {:.3f}, {:.3f}\n"
        "VEL: {:.6f}, {:.6f}\n"
        "GRAVITY: {:.6f}  MOD: {:.3f}\n"
        "SPEED: {:.6f}\n"
        "MINI:{} GROUND:{} UPSIDE:{} HOLD:{} DEAD:{}\n"
        "SAMPLE TICK: {} (EXACT GD TICK: NOT VERIFIED)\n"
        "LEVEL TIME: {:.6f}\n"
        "PROGRESS: {:.3f}%\n"
        "LEVEL PARSER: {}\n"
        "OBJECTS:{} SOLID:{} HAZ:{} ORB:{} PAD:{} PORTAL:{} UNKNOWN:{}\n"
        "UNSUPPORTED GAMEPLAY OBJECTS: {}",
        core::toString(p.mode),
        p.x, p.y,
        p.velocityX, p.velocityY,
        p.gravity, p.gravityModifier,
        p.speed,
        p.mini ? 1 : 0,
        p.grounded ? 1 : 0,
        p.upsideDown ? 1 : 0,
        p.holding ? 1 : 0,
        p.dead ? 1 : 0,
        snapshot.gameTick,
        snapshot.levelTime,
        snapshot.levelProgress,
        world.parsed ? "PARSED" : "WAITING",
        world.objects.size(),
        world.solids,
        world.hazards,
        world.orbs,
        world.pads,
        world.portals,
        world.unknown,
        world.unsupportedGameplay
    );

    m_label->setString(text.c_str());
}

} // namespace autobot::ui
