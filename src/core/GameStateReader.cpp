#include "autobot/core/GameStateReader.hpp"

#include <Geode/Geode.hpp>

#include <cmath>
#include <unordered_map>

using namespace geode::prelude;

namespace autobot::core {
namespace {

std::unordered_map<PlayerObject const*, GameMode> g_runtimeModes;

GameMode fallbackModeFromFlags(PlayerObject const* player) {
    if (!player) return GameMode::Unknown;

    if (player->m_isBird) return GameMode::Ufo;
    if (player->m_isDart) return GameMode::Wave;
    if (player->m_isSwing) return GameMode::Swing;
    if (player->m_isRobot) return GameMode::Robot;
    if (player->m_isSpider) return GameMode::Spider;
    if (player->m_isBall) return GameMode::Ball;
    if (player->m_isShip) return GameMode::Ship;
    return GameMode::Cube;
}

} // namespace

void GameStateReader::resetRuntimeModes() {
    g_runtimeModes.clear();
}

void GameStateReader::setRuntimeMode(PlayerObject const* player, GameMode mode) {
    if (!player || mode == GameMode::Unknown) return;
    g_runtimeModes[player] = mode;
}

GameMode GameStateReader::detectMode(PlayerObject const* player) {
    if (!player) return GameMode::Unknown;

    if (auto const it = g_runtimeModes.find(player); it != g_runtimeModes.end()) {
        return it->second;
    }

    // Fallback only. Normal PlayLayer runtime is seeded at startGame and then
    // updated by PlayerObject::switchedToMode, so solver mode does not depend
    // on direct bool-member reads.
    return fallbackModeFromFlags(player);
}

namespace {

PlayerState capturePlayer(PlayerObject* player) {
    PlayerState state{};
    if (!player) return state;

    const auto position = player->getRealPosition();
    const auto objectRect = player->getObjectRect();

    state.x = static_cast<double>(position.x);
    state.y = static_cast<double>(position.y);

    double rawXVelocity = player->getCurrentXVelocity();
    if (!player->m_isPlatformer && player->m_isGoingLeft) {
        rawXVelocity = -std::abs(rawXVelocity);
    }
    state.velocityX = rawXVelocity;
    state.velocityY = player->getYVelocity();
    state.gravity = player->m_gravity;
    state.gravityModifier = static_cast<double>(player->m_gravityMod);
    state.jumpVelocity = player->m_yStart;
    state.speed = static_cast<double>(player->m_playerSpeed);
    state.speedMultiplier = static_cast<double>(player->m_speedMultiplier);
    state.objectBoundsWidth = static_cast<double>(objectRect.size.width);
    state.objectBoundsHeight = static_cast<double>(objectRect.size.height);
    state.mode = GameStateReader::detectMode(player);
    state.mini = player->m_vehicleSize < 1.0f;
    state.grounded = player->m_isOnGround;
    state.upsideDown = player->m_isUpsideDown;

    // Observation only. The solver uses InputController::botHolding(P1/P2) as
    // the authoritative state for inputs emitted by AutoBot.
    state.holding = player->m_jumpBuffered;
    state.goingLeft = player->m_isGoingLeft;
    state.dead = player->m_isDead;
    return state;
}

} // namespace

GameSnapshot GameStateReader::capture(PlayLayer* playLayer, std::uint64_t gameTick) {
    GameSnapshot snapshot{};
    snapshot.gameTick = gameTick;
    snapshot.solverSampleID = gameTick;

    if (!playLayer || !playLayer->m_player1) return snapshot;

    snapshot.valid = true;
    snapshot.levelTime = playLayer->m_gameState.m_levelTime;
    snapshot.levelProgress = playLayer->getCurrentPercent();
    snapshot.player = capturePlayer(playLayer->m_player1);

    snapshot.dualMode = playLayer->m_gameState.m_isDualMode && playLayer->m_player2;
    snapshot.player2Valid = snapshot.dualMode && playLayer->m_player2;
    if (snapshot.player2Valid) snapshot.player2 = capturePlayer(playLayer->m_player2);

    return snapshot;
}

} // namespace autobot::core
