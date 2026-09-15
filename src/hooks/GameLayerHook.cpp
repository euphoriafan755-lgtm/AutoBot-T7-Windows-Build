#include <Geode/Geode.hpp>
#include <Geode/modify/PlayLayer.hpp>

#include "autobot/core/GameStateReader.hpp"
#include "autobot/ui/DiagnosticHUD.hpp"
#include "autobot/world/LevelParser.hpp"

using namespace geode::prelude;

class $modify(AutoBotT7GameLayerHook, PlayLayer) {
    struct Fields {
        std::uint64_t tick = 0;
        bool parserAttempted = false;
        autobot::world::StaticWorld world{};
        autobot::ui::DiagnosticHUD hud{};
    };

    void postUpdate(float dt) {
        PlayLayer::postUpdate(dt);

        auto* playLayer = PlayLayer::get();
        if (!playLayer || playLayer != this) {
            return;
        }

        ++m_fields->tick;

        if (!m_fields->hud.attached()) {
            if (!m_fields->hud.attach(playLayer)) {
                log::warn("AutoBot T7: diagnostic HUD could not be attached yet");
            }
        }

        if (!m_fields->parserAttempted && playLayer->m_objects && playLayer->m_objects->count() > 0) {
            m_fields->world = autobot::world::LevelParser::parse(playLayer);
            m_fields->parserAttempted = true;

            if (m_fields->world.parsed) {
                log::info(
                    "AutoBot T7: parsed {} objects (solid={}, hazard={}, orb={}, pad={}, portal={}, decoration={}, unknown={})",
                    m_fields->world.objects.size(),
                    m_fields->world.solids,
                    m_fields->world.hazards,
                    m_fields->world.orbs,
                    m_fields->world.pads,
                    m_fields->world.portals,
                    m_fields->world.decorations,
                    m_fields->world.unknown
                );
            } else {
                log::error("AutoBot T7: LevelParser failed: {}", m_fields->world.error);
            }
        }

        const auto snapshot = autobot::core::GameStateReader::capture(playLayer, m_fields->tick);
        m_fields->hud.update(snapshot, m_fields->world);
    }
};
