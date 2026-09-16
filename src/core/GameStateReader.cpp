#include "autobot/core/GameStateReader.hpp"

#include <Geode/Geode.hpp>

#include <cmath>

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

    double rawXVelocity = player->getCurrentXVelocity();
    if (!player->m_isPlatformer && player->m_isGoingLeft) {
        rawXVelocity = -std::abs(rawXVelocity);
    }
    snapshot.player.velocityX = rawXVelocity;
    snapshot.player.velocityY = player->getYVelocity();
    snapshot.player.gravity = player->m_gravity;
    snapshot.player.gravityModifier = static_cast<double>(player->m_gravityMod);

    // Ground-mode jump velocity is exposed as m_yStart in the 2.2081 binding.
    // This is a velocity target in GD's normalized physics units, not an
    // acceleration and not a world-units-per-second value.
    snapshot.player.jumpVelocity = player->m_yStart;

    snapshot.player.speed = static_cast<double>(player->m_playerSpeed);
    snapshot.player.speedMultiplier = static_cast<double>(player->m_speedMultiplier);
    snapshot.player.objectBoundsWidth = static_cast<double>(objectRect.size.width);
    snapshot.player.objectBoundsHeight = static_cast<double>(objectRect.size.height);
    snapshot.player.mode = detectMode(player);
    snapshot.player.mini = player->m_vehicleSize < 1.0f;
    snapshot.player.grounded = player->m_isOnGround;
    snapshot.player.upsideDown = player->m_isUpsideDown;

    // Geode 2.2081's inline buttonDown(Jump) is not a usable hold-state query
    // for this purpose. m_jumpBuffered is the runtime jump/hold state consumed
    // by PlayerObject::updateJump; the planner itself uses InputController's
    // authoritative botHolding state for counterfactual input edges.
    snapshot.player.holding = player->m_jumpBuffered;
    snapshot.player.goingLeft = player->m_isGoingLeft;
    snapshot.player.dead = player->m_isDead;

    return snapshot;
}

} // namespace autobot::core
