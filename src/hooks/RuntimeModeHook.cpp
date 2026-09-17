#include <Geode/Geode.hpp>
#include <Geode/modify/PlayLayer.hpp>
#include <Geode/modify/PlayerObject.hpp>

#include "autobot/core/GameStateReader.hpp"

using namespace geode::prelude;

namespace {

autobot::core::GameMode modeFromPortalType(GameObjectType type) {
    switch (type) {
        case GameObjectType::CubePortal: return autobot::core::GameMode::Cube;
        case GameObjectType::ShipPortal: return autobot::core::GameMode::Ship;
        case GameObjectType::BallPortal: return autobot::core::GameMode::Ball;
        case GameObjectType::UfoPortal: return autobot::core::GameMode::Ufo;
        case GameObjectType::WavePortal: return autobot::core::GameMode::Wave;
        case GameObjectType::RobotPortal: return autobot::core::GameMode::Robot;
        case GameObjectType::SpiderPortal: return autobot::core::GameMode::Spider;
        case GameObjectType::SwingPortal: return autobot::core::GameMode::Swing;
        default: return autobot::core::GameMode::Unknown;
    }
}

autobot::core::GameMode modeFromStartMode(int mode) {
    switch (mode) {
        case 0: return autobot::core::GameMode::Cube;
        case 1: return autobot::core::GameMode::Ship;
        case 2: return autobot::core::GameMode::Ball;
        case 3: return autobot::core::GameMode::Ufo;
        case 4: return autobot::core::GameMode::Wave;
        case 5: return autobot::core::GameMode::Robot;
        case 6: return autobot::core::GameMode::Spider;
        case 7: return autobot::core::GameMode::Swing;
        default: return autobot::core::GameMode::Unknown;
    }
}

} // namespace

class $modify(AutoBotT7RuntimeModePlayerHook, PlayerObject) {
    void switchedToMode(GameObjectType type) {
        PlayerObject::switchedToMode(type);

        const auto mode = modeFromPortalType(type);
        if (mode == autobot::core::GameMode::Unknown) return;

        autobot::core::GameStateReader::setRuntimeMode(this, mode);
        log::info(
            "RUNTIME_MODE_TRANSITION player={} portalType={} mode={}",
            static_cast<void*>(this),
            static_cast<int>(type),
            autobot::core::toString(mode)
        );
    }
};

class $modify(AutoBotT7RuntimeModePlayLayerHook, PlayLayer) {
    void startGame() {
        PlayLayer::startGame();

        autobot::core::GameStateReader::resetRuntimeModes();
        const int startModeValue = m_levelSettings ? m_levelSettings->m_startMode : -1;
        const auto startMode = modeFromStartMode(startModeValue);
        if (m_player1 && startMode != autobot::core::GameMode::Unknown) {
            autobot::core::GameStateReader::setRuntimeMode(m_player1, startMode);
        }
        if (m_player2 && startMode != autobot::core::GameMode::Unknown) {
            autobot::core::GameStateReader::setRuntimeMode(m_player2, startMode);
        }

        log::info(
            "RUNTIME_MODE_SEED startMode={} resolved={} p1={} p2={}",
            startModeValue,
            autobot::core::toString(startMode),
            static_cast<void*>(m_player1),
            static_cast<void*>(m_player2)
        );
    }
};
