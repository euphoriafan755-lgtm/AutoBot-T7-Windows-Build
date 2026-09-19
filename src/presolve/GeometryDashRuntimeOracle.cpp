#include "autobot/presolve/GeometryDashRuntimeOracle.hpp"

#include <Geode/Geode.hpp>
#include <Geode/binding/EffectGameObject.hpp>
#include <Geode/binding/GameToolbox.hpp>
#include <Geode/binding/PlayLayer.hpp>

#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace geode::prelude;

namespace autobot::presolve {
namespace {

constexpr std::uint64_t kFnvOffset1 = 1469598103934665603ULL;
constexpr std::uint64_t kFnvOffset2 = 1099511628211ULL ^ 0x9e3779b97f4a7c15ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

void mix(std::uint64_t& h, std::uint64_t v) {
    for (int i = 0; i < 8; ++i) {
        h ^= static_cast<std::uint8_t>((v >> (i * 8)) & 0xffU);
        h *= kFnvPrime;
    }
}

void mixFloat(std::uint64_t& h, float value) {
    mix(h, std::bit_cast<std::uint32_t>(value));
}

void mixDouble(std::uint64_t& h, double value) {
    mix(h, std::bit_cast<std::uint64_t>(value));
}

bool shouldTrack(GameObject* object) {
    if (!object) return false;
    if (object->m_groupCount > 0 || object->m_isTrigger) return true;
    switch (object->m_objectType) {
        case GameObjectType::Solid:
        case GameObjectType::Hazard:
        case GameObjectType::Slope:
        case GameObjectType::Decoration:
            return false;
        default:
            return true;
    }
}

struct ObjectState {
    GameObject* object = nullptr;
    float positionX = 0.0f;
    float positionY = 0.0f;
    float offsetX = 0.0f;
    float offsetY = 0.0f;
    cocos2d::CCPoint lastPosition{};
    float rotation = 0.0f;
    float scaleX = 1.0f;
    float scaleY = 1.0f;
    bool groupDisabled = false;
    bool disabled = false;
    bool disabled2 = false;
    int enabledGroupsCounter = 0;
    bool activatedP1 = false;
    bool activatedP2 = false;
};

ObjectState captureObject(GameObject* object) {
    ObjectState state{};
    state.object = object;
    if (!object) return state;
    state.positionX = object->m_positionX;
    state.positionY = object->m_positionY;
    state.offsetX = object->m_positionXOffset;
    state.offsetY = object->m_positionYOffset;
    state.lastPosition = object->m_lastPosition;
    state.rotation = object->getRotation();
    state.scaleX = object->getScaleX();
    state.scaleY = object->getScaleY();
    state.groupDisabled = object->m_isGroupDisabled;
    state.disabled = object->m_isDisabled;
    state.disabled2 = object->m_isDisabled2;
    state.enabledGroupsCounter = object->m_enabledGroupsCounter;
    if (auto* effect = typeinfo_cast<EffectGameObject*>(object)) {
        state.activatedP1 = effect->m_activatedByPlayer1;
        state.activatedP2 = effect->m_activatedByPlayer2;
    }
    return state;
}

void restoreObject(PlayLayer* layer, ObjectState const& state) {
    auto* object = state.object;
    if (!layer || !object) return;
    object->m_positionX = state.positionX;
    object->m_positionY = state.positionY;
    object->m_positionXOffset = state.offsetX;
    object->m_positionYOffset = state.offsetY;
    object->setPosition(object->getRealPosition());
    object->m_lastPosition = state.lastPosition;
    object->setRotation(state.rotation);
    object->setScaleX(state.scaleX);
    object->setScaleY(state.scaleY);
    object->m_isGroupDisabled = state.groupDisabled;
    object->m_isDisabled = state.disabled;
    object->m_isDisabled2 = state.disabled2;
    object->m_enabledGroupsCounter = state.enabledGroupsCounter;
    if (auto* effect = typeinfo_cast<EffectGameObject*>(object)) {
        effect->m_activatedByPlayer1 = state.activatedP1;
        effect->m_activatedByPlayer2 = state.activatedP2;
    }
    object->setObjectRectDirty(true);
    object->setOrientedRectDirty(true);
    layer->updateObjectSection(object);
}

void mixPlayer(std::uint64_t& lo, std::uint64_t& hi, PlayerObject* player) {
    if (!player) {
        mix(lo, 0xdeadbeefULL);
        mix(hi, 0xbad0ULL);
        return;
    }
    const auto p = player->getRealPosition();
    mixFloat(lo, p.x); mixFloat(hi, p.y);
    mixDouble(lo, player->getYVelocity());
    mixDouble(hi, player->getCurrentXVelocity());
    mixFloat(lo, player->m_gravityMod);
    mixFloat(hi, player->m_vehicleSize);
    mix(lo, static_cast<std::uint64_t>(player->m_isDead));
    mix(lo, static_cast<std::uint64_t>(player->m_isOnGround) << 1U);
    mix(lo, static_cast<std::uint64_t>(player->m_isUpsideDown) << 2U);
    mix(lo, static_cast<std::uint64_t>(player->m_isShip) << 3U);
    mix(lo, static_cast<std::uint64_t>(player->m_isBall) << 4U);
    mix(lo, static_cast<std::uint64_t>(player->m_isBird) << 5U);
    mix(lo, static_cast<std::uint64_t>(player->m_isDart) << 6U);
    mix(lo, static_cast<std::uint64_t>(player->m_isRobot) << 7U);
    mix(lo, static_cast<std::uint64_t>(player->m_isSpider) << 8U);
    mix(lo, static_cast<std::uint64_t>(player->m_isSwing) << 9U);
    mix(hi, static_cast<std::uint64_t>(player->m_jumpBuffered));
    mix(hi, static_cast<std::uint64_t>(player->m_isDashing) << 1U);
    mix(hi, static_cast<std::uint64_t>(player->m_isGoingLeft) << 2U);
}

} // namespace

struct GeometryDashRuntimeOracle::Impl {
    struct Snapshot {
        Ref<CheckpointObject> checkpoint;
        std::uint64_t randomSeed = 0;
        std::uint64_t replaySeed = 0;
        int attempts = 0;
        bool practiceMode = false;
        float extraDelta = 0.0f;
        float timeWarp = 1.0f;
        UniversalAction action{};
        std::vector<ObjectState> objectStates;
    };

    PlayLayer* layer = nullptr;
    StepCallback callback;
    double dt = 1.0 / 240.0;
    std::string error;
    UniversalToken nextToken = 0;
    std::unordered_map<UniversalToken, std::shared_ptr<Snapshot>> snapshots;
    std::vector<GameObject*> trackedObjects;
    UniversalAction action{};

    explicit Impl(PlayLayer* value, double step) : layer(value), dt(step) {
        if (!layer || !layer->m_objects) return;
        for (auto* object : CCArrayExt<GameObject*>(layer->m_objects)) {
            if (shouldTrack(object)) trackedObjects.push_back(object);
        }
    }

    void setButton(bool desired, bool& current, int button, bool player2) {
        if (!layer || desired == current) return;
        layer->queueButton(button, desired, player2, layer->m_gameState.m_levelTime);
        current = desired;
    }

    void applyAction(UniversalAction desired) {
        setButton(desired.p1Hold, action.p1Hold, static_cast<int>(PlayerButton::Jump), false);
        setButton(desired.p1Left, action.p1Left, static_cast<int>(PlayerButton::Left), false);
        setButton(desired.p1Right, action.p1Right, static_cast<int>(PlayerButton::Right), false);
        if (layer && layer->m_player2) {
            setButton(desired.p2Hold, action.p2Hold, static_cast<int>(PlayerButton::Jump), true);
            setButton(desired.p2Left, action.p2Left, static_cast<int>(PlayerButton::Left), true);
            setButton(desired.p2Right, action.p2Right, static_cast<int>(PlayerButton::Right), true);
        }
    }

    UniversalFingerprint fingerprint() const {
        std::uint64_t lo = kFnvOffset1;
        std::uint64_t hi = kFnvOffset2;
        if (!layer) return {lo, hi};
        mixDouble(lo, layer->m_gameState.m_levelTime);
        mixDouble(hi, static_cast<double>(layer->m_gameState.m_currentProgress));
        mixFloat(lo, layer->m_gameState.m_timeWarp);
        mix(hi, GameToolbox::getfast_srand());
        mix(lo, static_cast<std::uint64_t>(layer->m_gameState.m_isDualMode));
        mix(lo, static_cast<std::uint64_t>(layer->m_isPlatformer) << 1U);
        mixPlayer(lo, hi, layer->m_player1);
        if (layer->m_player2) mixPlayer(hi, lo, layer->m_player2);
        mix(lo, static_cast<std::uint64_t>(action.p1Hold));
        mix(lo, static_cast<std::uint64_t>(action.p1Left) << 1U);
        mix(lo, static_cast<std::uint64_t>(action.p1Right) << 2U);
        mix(hi, static_cast<std::uint64_t>(action.p2Hold));
        mix(hi, static_cast<std::uint64_t>(action.p2Left) << 1U);
        mix(hi, static_cast<std::uint64_t>(action.p2Right) << 2U);
        for (auto* object : trackedObjects) {
            if (!object) continue;
            mix(lo, static_cast<std::uint64_t>(object->m_uniqueID));
            mixFloat(lo, object->m_positionX);
            mixFloat(hi, object->m_positionY);
            mixFloat(lo, object->m_positionXOffset);
            mixFloat(hi, object->m_positionYOffset);
            mixFloat(lo, object->getRotation());
            mixFloat(hi, object->getScaleX());
            mixFloat(hi, object->getScaleY());
            mix(lo, static_cast<std::uint64_t>(object->m_isGroupDisabled));
            mix(lo, static_cast<std::uint64_t>(object->m_isDisabled) << 1U);
            mix(lo, static_cast<std::uint64_t>(object->m_isDisabled2) << 2U);
            mix(hi, static_cast<std::uint64_t>(static_cast<std::uint32_t>(object->m_enabledGroupsCounter)));
            if (auto* effect = typeinfo_cast<EffectGameObject*>(object)) {
                mix(hi, static_cast<std::uint64_t>(effect->m_activatedByPlayer1));
                mix(hi, static_cast<std::uint64_t>(effect->m_activatedByPlayer2) << 1U);
            }
        }
        return {lo, hi};
    }
};

GeometryDashRuntimeOracle::GeometryDashRuntimeOracle(PlayLayer* layer, double stepDt)
  : m_impl(std::make_unique<Impl>(layer, stepDt)) {}

GeometryDashRuntimeOracle::~GeometryDashRuntimeOracle() = default;

void GeometryDashRuntimeOracle::setStepCallback(StepCallback callback) {
    m_impl->callback = std::move(callback);
}

void GeometryDashRuntimeOracle::setStepDt(double value) {
    if (std::isfinite(value) && value > 0.0) m_impl->dt = value;
}

double GeometryDashRuntimeOracle::stepDt() const { return m_impl->dt; }
std::string const& GeometryDashRuntimeOracle::lastError() const { return m_impl->error; }

UniversalObservation GeometryDashRuntimeOracle::observe() const {
    UniversalObservation out{};
    auto* layer = m_impl->layer;
    if (!layer || !layer->m_player1) return out;
    out.valid = true;
    out.dead = layer->m_player1->m_isDead
        || (layer->m_gameState.m_isDualMode && layer->m_player2 && layer->m_player2->m_isDead);
    out.complete = layer->m_hasCompletedLevel || layer->m_levelEndAnimationStarted;
    out.dual = layer->m_gameState.m_isDualMode && layer->m_player2;
    out.platformer = layer->m_isPlatformer;
    out.progress = static_cast<double>(layer->getCurrentPercent());
    out.fingerprint = m_impl->fingerprint();
    return out;
}

std::optional<UniversalToken> GeometryDashRuntimeOracle::capture() {
    auto* layer = m_impl->layer;
    if (!layer) return std::nullopt;
    auto* checkpoint = layer->createCheckpoint();
    if (!checkpoint) {
        m_impl->error = "createCheckpoint returned null";
        return std::nullopt;
    }
    auto snap = std::make_shared<Impl::Snapshot>();
    snap->checkpoint = checkpoint;
    snap->randomSeed = GameToolbox::getfast_srand();
    snap->replaySeed = layer->m_replayRandSeed;
    snap->attempts = layer->m_attempts;
    snap->practiceMode = layer->m_isPracticeMode;
    snap->extraDelta = layer->m_extraDelta;
    snap->timeWarp = layer->m_gameState.m_timeWarp;
    snap->action = m_impl->action;
    snap->objectStates.reserve(m_impl->trackedObjects.size());
    for (auto* object : m_impl->trackedObjects) snap->objectStates.push_back(captureObject(object));
    const auto token = ++m_impl->nextToken;
    m_impl->snapshots.emplace(token, std::move(snap));
    return token;
}

bool GeometryDashRuntimeOracle::restore(UniversalToken token) {
    auto* layer = m_impl->layer;
    auto it = m_impl->snapshots.find(token);
    if (!layer || it == m_impl->snapshots.end() || !it->second || !it->second->checkpoint) {
        m_impl->error = "snapshot token missing";
        return false;
    }
    auto const& snap = *it->second;
    layer->m_queuedButtons.clear();
    if (layer->m_checkpointArray) {
        layer->m_checkpointArray->removeAllObjects();
        layer->m_checkpointArray->addObject(snap.checkpoint.data());
    }
    layer->m_currentCheckpoint = snap.checkpoint.data();
    const bool oldPractice = layer->m_isPracticeMode;
    layer->m_isPracticeMode = true;
    GameToolbox::fast_srand(snap.randomSeed);
    layer->m_replayRandSeed = snap.replaySeed;
    layer->loadFromCheckpoint(snap.checkpoint.data());
    layer->m_isPracticeMode = snap.practiceMode;
    layer->m_attempts = snap.attempts;
    layer->m_extraDelta = snap.extraDelta;
    layer->m_gameState.m_timeWarp = snap.timeWarp;
    for (auto const& object : snap.objectStates) restoreObject(layer, object);
    layer->updatePlayerCollisionBlocks();
    layer->checkSpawnObjects();
    layer->sortSectionVector();
    layer->m_queuedButtons.clear();
    layer->m_isPaused = true;
    m_impl->action = snap.action;
    (void)oldPractice;
    return true;
}

UniversalObservation GeometryDashRuntimeOracle::step(UniversalAction action) {
    auto* layer = m_impl->layer;
    if (!layer || !m_impl->callback) {
        m_impl->error = "engine step callback unavailable";
        return {};
    }
    m_impl->applyAction(action);
    layer->m_isPaused = false;
    m_impl->callback(static_cast<float>(m_impl->dt));
    layer->m_isPaused = true;
    return observe();
}

void GeometryDashRuntimeOracle::discard(UniversalToken token) {
    m_impl->snapshots.erase(token);
}

} // namespace autobot::presolve
