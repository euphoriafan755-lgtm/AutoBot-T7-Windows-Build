#include "autobot/core/GameStateReader.hpp"

#include <Geode/Geode.hpp>

using namespace geode::prelude;

namespace autobot::core {

GameMode GameStateReader::detectMode(PlayerObject const* player) {
    if (!player) return GameMode::Unknown;

    if (player->m_isShip) return GameMode::Ship;
    if (player->m_isBird) return GameMode::Ufo;
    if (player->m_isBall) return GameMode::Ball;
    if (player->m_isDart) return GameMode::Wave;
    if (player->m_isRobot) return GameMode::Robot;
    if (player->m_isSpider) return GameMode::Spider;
    if (player->m_isSwing) return GameMode::Swing;
    return GameMode::Cube;
}

GameSnapshot GameStateReader::capture(PlayLayer* playLayer, std::uint64_t gameTick) {
    GameSnapshot snapshot{};
    snapshot.gameTick = gameTick;
    snapshot.solverSampleID = gameTick;

    if (!playLayer || !playLayer->m_player1) {
        return snapshot;
    }

    auto* player = playLayer->m_player1;
    const auto position = player->getRealPosition();
    const auto objectRect = player->getObjectRect();

    snapshot.valid = true;
    snapshot.levelTime = playLayer->m_gameState.m_levelTime;
    snapshot.levelProgress = playLayer->getCurrentPercent();

    snapshot.player.x = static_cast<double>(position.x);
    snapshot.player.y = static_cast<double>(position.y);
    snapshot.player.velocityX = player->getCurrentXVelocity();
    snapshot.player.velocityY = player->getYVelocity();
    snapshot.player.gravity = player->m_gravity;
    snapshot.player.gravityModifier = static_cast<double>(player->m_gravityMod);

    // Geode 5.10.1 / GD 2.2081 does not expose the historical
    // PlayerObject::m_jumpAccel binding. Do not guess an offset or reinterpret
    // unrelated fields. A value of 0 explicitly means "not directly bound";
    // PhysicsValidationHarness learns the actual press delta from runtime
    // predicted-vs-actual samples once input occurs.
    snapshot.player.jumpAcceleration = 0.0;

    snapshot.player.speed = static_cast<double>(player->m_playerSpeed);
    snapshot.player.objectBoundsWidth = static_cast<double>(objectRect.size.width);
    snapshot.player.objectBoundsHeight = static_cast<double>(objectRect.size.height);
    snapshot.player.mode = detectMode(player);
    snapshot.player.mini = player->m_vehicleSize < 1.0f;
    snapshot.player.grounded = player->m_isOnGround;
    snapshot.player.upsideDown = player->m_isUpsideDown;
    snapshot.player.holding = player->buttonDown(PlayerButton::Jump);
    snapshot.player.dead = player->m_isDead;

    return snapshot;
}

} // namespace autobot::core
