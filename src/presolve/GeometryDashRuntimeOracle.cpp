#include "autobot/presolve/GeometryDashRuntimeOracle.hpp"

#include <Geode/Geode.hpp>
#include <Geode/binding/EffectGameObject.hpp>
#include <Geode/binding/GameToolbox.hpp>
#include <Geode/binding/PlayLayer.hpp>

#include <bit>
#include <cmath>
#include <cstdint>
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

std::uint64_t bits(float value) { return std::bit_cast<std::uint32_t>(value); }
std::uint64_t bits(double value) { return std::bit_cast<std::uint64_t>(value); }

UniversalFingerprint hashWords(std::vector<std::uint64_t> const& words) {
    std::uint64_t lo = kFnvOffset1;
    std::uint64_t hi = kFnvOffset2;
    for (std::size_t i = 0; i < words.size(); ++i) {
        mix(lo, words[i]);
        mix(hi, words[words.size() - 1U - i] ^ static_cast<std::uint64_t>(i));
    }
    return {lo, hi};
}

void appendAction(std::vector<std::uint64_t>& out, UniversalAction const& a) {
    std::uint64_t v = 0;
    v |= static_cast<std::uint64_t>(a.p1Hold) << 0U;
    v |= static_cast<std::uint64_t>(a.p1Left) << 1U;
    v |= static_cast<std::uint64_t>(a.p1Right) << 2U;
    v |= static_cast<std::uint64_t>(a.p2Hold) << 3U;
    v |= static_cast<std::uint64_t>(a.p2Left) << 4U;
    v |= static_cast<std::uint64_t>(a.p2Right) << 5U;
    out.push_back(v);
}

bool actionIdle(UniversalAction const& a) {
    return !a.p1Hold && !a.p1Left && !a.p1Right && !a.p2Hold && !a.p2Left && !a.p2Right;
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

    friend bool operator==(ObjectState const& a, ObjectState const& b) {
        return a.object == b.object
            && a.positionX == b.positionX && a.positionY == b.positionY
            && a.offsetX == b.offsetX && a.offsetY == b.offsetY
            && a.lastPosition.x == b.lastPosition.x && a.lastPosition.y == b.lastPosition.y
            && a.rotation == b.rotation && a.scaleX == b.scaleX && a.scaleY == b.scaleY
            && a.groupDisabled == b.groupDisabled && a.disabled == b.disabled
            && a.disabled2 == b.disabled2 && a.enabledGroupsCounter == b.enabledGroupsCounter
            && a.activatedP1 == b.activatedP1 && a.activatedP2 == b.activatedP2;
    }
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

void appendPlayer(std::vector<std::uint64_t>& out, PlayerObject* player) {
    if (!player) {
        out.push_back(0x504c415945524e55ULL); // PLAYERNU
        return;
    }
    const auto p = player->getRealPosition();
    out.push_back(bits(p.x)); out.push_back(bits(p.y));
    out.push_back(bits(player->getYVelocity()));
    out.push_back(bits(player->getCurrentXVelocity()));
    out.push_back(bits(player->m_gravityMod));
    out.push_back(bits(player->m_vehicleSize));
    out.push_back(bits(player->m_playerSpeed));
    out.push_back(bits(player->m_platformerVelocityRelated));
    out.push_back(bits(static_cast<double>(player->m_dashX)));
    out.push_back(bits(static_cast<double>(player->m_dashY)));
    out.push_back(bits(static_cast<double>(player->m_dashAngle)));
    out.push_back(bits(static_cast<double>(player->m_dashStartTime)));
    std::uint64_t flags = 0;
    flags |= static_cast<std::uint64_t>(player->m_isDead) << 0U;
    flags |= static_cast<std::uint64_t>(player->m_isOnGround) << 1U;
    flags |= static_cast<std::uint64_t>(player->m_isUpsideDown) << 2U;
    flags |= static_cast<std::uint64_t>(player->m_isShip) << 3U;
    flags |= static_cast<std::uint64_t>(player->m_isBall) << 4U;
    flags |= static_cast<std::uint64_t>(player->m_isBird) << 5U;
    flags |= static_cast<std::uint64_t>(player->m_isDart) << 6U;
    flags |= static_cast<std::uint64_t>(player->m_isRobot) << 7U;
    flags |= static_cast<std::uint64_t>(player->m_isSpider) << 8U;
    flags |= static_cast<std::uint64_t>(player->m_isSwing) << 9U;
    flags |= static_cast<std::uint64_t>(player->m_jumpBuffered) << 10U;
    flags |= static_cast<std::uint64_t>(player->m_isDashing) << 11U;
    flags |= static_cast<std::uint64_t>(player->m_isGoingLeft) << 12U;
    flags |= static_cast<std::uint64_t>(player->m_isPlatformer) << 13U;
    out.push_back(flags);
    out.push_back(static_cast<std::uint64_t>(player->m_touchedRings.size()));
    out.push_back(static_cast<std::uint64_t>(player->m_jumpPadRelated.size()));
    out.push_back(static_cast<std::uint64_t>(player->m_holdingButtons.size()));
}

bool playerQuiescent(PlayerObject* p) {
    if (!p) return true;
    return p->m_isOnGround
        && !p->m_isDashing
        && std::abs(p->getYVelocity()) < 1e-12
        && std::abs(p->getCurrentXVelocity()) < 1e-12;
}

} // namespace

struct GeometryDashRuntimeOracle::Impl {
    struct CanonicalSections {
        std::vector<std::uint64_t> core;
        std::vector<std::uint64_t> rng;
        std::vector<std::uint64_t> effect;
        std::vector<std::uint64_t> dynamic;
        std::vector<std::uint64_t> temporal;
    };

    struct Snapshot {
        Ref<CheckpointObject> checkpoint;
        std::uint64_t randomSeed = 0;
        std::uint64_t replaySeed = 0;
        int attempts = 0;
        bool practiceMode = false;
        float extraDelta = 0.0f;
        float timeWarp = 1.0f;
        double levelTime = 0.0;
        float currentProgress = 0.0f;
        UniversalAction action{};
        std::vector<ObjectState> objectStates;
        CanonicalSections sections;
        UniversalCanonicalState canonical;
    };

    PlayLayer* layer = nullptr;
    StepCallback callback;
    double dt = 1.0 / 240.0;
    std::string error;
    UniversalToken nextToken = 0;
    std::unordered_map<UniversalToken, std::shared_ptr<Snapshot>> snapshots;
    std::vector<GameObject*> trackedObjects;
    UniversalAction action{};
    RuntimeOracleValidation validation{};
    bool hasTimedOrTriggeredWorld = false;
    std::optional<CanonicalSections> lastCapturedSections;

    explicit Impl(PlayLayer* value, double step) : layer(value), dt(step) {
        if (!layer || !layer->m_objects) return;
        for (auto* object : CCArrayExt<GameObject*>(layer->m_objects)) {
            if (!object) continue;
            if (object->m_isTrigger || object->m_groupCount > 0) hasTimedOrTriggeredWorld = true;
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

    CanonicalSections buildSections() const {
        CanonicalSections s{};
        if (!layer) return s;

        // Absolute levelTime is intentionally excluded from search identity.
        // Time-dependent future state must instead be represented by concrete
        // pending/effect state; until that projection is proven complete the
        // search marks this representation incomplete and does not transposition-dedup it.
        s.core.push_back(bits(static_cast<double>(layer->m_gameState.m_currentProgress)));
        s.core.push_back(bits(layer->m_gameState.m_timeWarp));
        s.core.push_back(static_cast<std::uint64_t>(layer->m_gameState.m_currentChannel));
        s.core.push_back(static_cast<std::uint64_t>(layer->m_gameState.m_rotateChannel));
        s.core.push_back(static_cast<std::uint64_t>(layer->m_gameState.m_isDualMode));
        s.core.push_back(static_cast<std::uint64_t>(layer->m_isPlatformer));
        s.core.push_back(bits(layer->m_gameState.m_portalY));
        s.core.push_back(static_cast<std::uint64_t>(layer->m_gameState.m_levelFlipping));
        s.core.push_back(static_cast<std::uint64_t>(layer->m_queuedButtons.size()));
        appendPlayer(s.core, layer->m_player1);
        appendPlayer(s.core, layer->m_player2);
        appendAction(s.core, action);

        s.rng.push_back(GameToolbox::getfast_srand());
        s.rng.push_back(layer->m_replayRandSeed);

        s.effect.push_back(static_cast<std::uint64_t>(trackedObjects.size()));
        s.dynamic.push_back(static_cast<std::uint64_t>(trackedObjects.size()));
        for (auto* object : trackedObjects) {
            if (!object) continue;
            const auto state = captureObject(object);
            s.dynamic.push_back(static_cast<std::uint64_t>(object->m_uniqueID));
            s.dynamic.push_back(static_cast<std::uint64_t>(static_cast<std::uint32_t>(object->m_objectID)));
            s.dynamic.push_back(bits(state.positionX)); s.dynamic.push_back(bits(state.positionY));
            s.dynamic.push_back(bits(state.offsetX)); s.dynamic.push_back(bits(state.offsetY));
            s.dynamic.push_back(bits(state.lastPosition.x)); s.dynamic.push_back(bits(state.lastPosition.y));
            s.dynamic.push_back(bits(state.rotation)); s.dynamic.push_back(bits(state.scaleX)); s.dynamic.push_back(bits(state.scaleY));
            std::uint64_t flags = 0;
            flags |= static_cast<std::uint64_t>(state.groupDisabled) << 0U;
            flags |= static_cast<std::uint64_t>(state.disabled) << 1U;
            flags |= static_cast<std::uint64_t>(state.disabled2) << 2U;
            flags |= static_cast<std::uint64_t>(state.activatedP1) << 3U;
            flags |= static_cast<std::uint64_t>(state.activatedP2) << 4U;
            s.dynamic.push_back(flags);
            s.dynamic.push_back(static_cast<std::uint64_t>(static_cast<std::uint32_t>(state.enabledGroupsCounter)));
            if (auto* effect = typeinfo_cast<EffectGameObject*>(object)) {
                s.effect.push_back(static_cast<std::uint64_t>(object->m_uniqueID));
                s.effect.push_back(static_cast<std::uint64_t>(effect->m_activatedByPlayer1));
                s.effect.push_back(static_cast<std::uint64_t>(effect->m_activatedByPlayer2));
                s.effect.push_back(static_cast<std::uint64_t>(object->m_isDisabled));
                s.effect.push_back(static_cast<std::uint64_t>(object->m_isGroupDisabled));
            }
        }

        // Temporal dominance is intentionally much stricter than canonical
        // equality. It is available only for a provably quiescent static
        // platformer state with no trigger/group scheduler in the level.
        s.temporal = s.core;
        s.temporal.insert(s.temporal.end(), s.rng.begin(), s.rng.end());
        s.temporal.insert(s.temporal.end(), s.dynamic.begin(), s.dynamic.end());
        return s;
    }

    std::uint64_t controlDecisionEpoch() const {
        if (!layer) return 0;

        std::vector<std::uint64_t> words;
        words.reserve(32U + trackedObjects.size() * 3U);

        words.push_back(static_cast<std::uint64_t>(layer->m_gameState.m_isDualMode));
        words.push_back(static_cast<std::uint64_t>(layer->m_isPlatformer));
        words.push_back(static_cast<std::uint64_t>(layer->m_gameState.m_currentChannel));
        words.push_back(static_cast<std::uint64_t>(layer->m_gameState.m_rotateChannel));
        words.push_back(static_cast<std::uint64_t>(layer->m_gameState.m_levelFlipping));
        words.push_back(bits(layer->m_gameState.m_timeWarp));
        words.push_back(bits(layer->m_gameState.m_portalY));

        auto appendControlPlayer = [&](PlayerObject* player) {
            if (!player) {
                words.push_back(0x4e4f504c41594552ULL);
                return;
            }

            std::uint64_t flags = 0;
            flags |= static_cast<std::uint64_t>(player->m_isDead) << 0U;
            flags |= static_cast<std::uint64_t>(player->m_isOnGround) << 1U;
            flags |= static_cast<std::uint64_t>(player->m_isUpsideDown) << 2U;
            flags |= static_cast<std::uint64_t>(player->m_isShip) << 3U;
            flags |= static_cast<std::uint64_t>(player->m_isBall) << 4U;
            flags |= static_cast<std::uint64_t>(player->m_isBird) << 5U;
            flags |= static_cast<std::uint64_t>(player->m_isDart) << 6U;
            flags |= static_cast<std::uint64_t>(player->m_isRobot) << 7U;
            flags |= static_cast<std::uint64_t>(player->m_isSpider) << 8U;
            flags |= static_cast<std::uint64_t>(player->m_isSwing) << 9U;
            flags |= static_cast<std::uint64_t>(player->m_jumpBuffered) << 10U;
            flags |= static_cast<std::uint64_t>(player->m_isDashing) << 11U;
            flags |= static_cast<std::uint64_t>(player->m_isGoingLeft) << 12U;
            flags |= static_cast<std::uint64_t>(player->m_isPlatformer) << 13U;
            words.push_back(flags);
            words.push_back(bits(player->m_gravityMod));
            words.push_back(bits(player->m_vehicleSize));
            words.push_back(bits(player->m_playerSpeed));
            words.push_back(static_cast<std::uint64_t>(player->m_touchedRings.size()));
            words.push_back(static_cast<std::uint64_t>(player->m_jumpPadRelated.size()));
        };

        appendControlPlayer(layer->m_player1);
        appendControlPlayer(layer->m_player2);

        // Only discrete trigger/group state participates here. Object position,
        // rotation and scale are deliberately excluded: moving geometry may
        // change every engine tick and would destroy macro stepping.
        for (auto* object : trackedObjects) {
            if (!object) continue;
            const auto state = captureObject(object);
            std::uint64_t flags = 0;
            flags |= static_cast<std::uint64_t>(state.groupDisabled) << 0U;
            flags |= static_cast<std::uint64_t>(state.disabled) << 1U;
            flags |= static_cast<std::uint64_t>(state.disabled2) << 2U;
            flags |= static_cast<std::uint64_t>(state.activatedP1) << 3U;
            flags |= static_cast<std::uint64_t>(state.activatedP2) << 4U;

            words.push_back(static_cast<std::uint64_t>(object->m_uniqueID));
            words.push_back(flags);
            words.push_back(static_cast<std::uint64_t>(
                static_cast<std::uint32_t>(state.enabledGroupsCounter)
            ));
        }

        const auto hash = hashWords(words);
        return hash.lo ^ (hash.hi + 0x9e3779b97f4a7c15ULL + (hash.lo << 6U) + (hash.lo >> 2U));
    }

    UniversalCanonicalState makeCanonical(CanonicalSections const& s) const {
        UniversalCanonicalState c{};
        c.words.reserve(s.core.size() + s.rng.size() + s.effect.size() + s.dynamic.size() + 4U);
        c.words.push_back(0x434f5245ULL); c.words.insert(c.words.end(), s.core.begin(), s.core.end());
        c.words.push_back(0x524e47ULL); c.words.insert(c.words.end(), s.rng.begin(), s.rng.end());
        c.words.push_back(0x454646454354ULL); c.words.insert(c.words.end(), s.effect.begin(), s.effect.end());
        c.words.push_back(0x44594e414d4943ULL); c.words.insert(c.words.end(), s.dynamic.begin(), s.dynamic.end());
        c.hash = hashWords(c.words);

        // This projection is deliberately not claimed complete: GD 2.2081 has
        // opaque/evolving internal trigger/effect state. Because this is false,
        // UniversalSearchCore will never prune a distinct history merely from
        // this projection.
        c.completeRepresentation = false;

        const bool quiescent = layer
            && layer->m_isPlatformer
            && !hasTimedOrTriggeredWorld
            && layer->m_queuedButtons.empty()
            && actionIdle(action)
            && playerQuiescent(layer->m_player1)
            && (!layer->m_gameState.m_isDualMode || playerQuiescent(layer->m_player2));
        if (quiescent) {
            c.temporalDominanceEligible = true;
            c.temporalWords = s.temporal;
            c.temporalHash = hashWords(c.temporalWords);
        }
        return c;
    }

    bool verifyRestore(Snapshot const& snap) {
        const auto restored = buildSections();
        ++validation.roundTripChecks;
        ++validation.effectStateChecks;
        ++validation.rngChecks;
        ++validation.dynamicWorldChecks;

        const bool coreOk = restored.core == snap.sections.core
            && std::abs(layer->m_gameState.m_levelTime - snap.levelTime) <= 1e-12
            && layer->m_gameState.m_currentProgress == snap.currentProgress;
        const bool rngOk = restored.rng == snap.sections.rng;
        const bool effectOk = restored.effect == snap.sections.effect;
        const bool dynamicOk = restored.dynamic == snap.sections.dynamic;
        validation.rngPass = validation.rngPass && rngOk;
        validation.effectStatePass = validation.effectStatePass && effectOk;
        validation.dynamicWorldPass = validation.dynamicWorldPass && dynamicOk;
        validation.roundTripPass = validation.roundTripPass && coreOk && rngOk && effectOk && dynamicOk;
        if (!validation.roundTripPass) {
            error = "CHECKPOINT ROUNDTRIP MISMATCH core=" + std::to_string(coreOk)
                + " rng=" + std::to_string(rngOk)
                + " effect=" + std::to_string(effectOk)
                + " dynamic=" + std::to_string(dynamicOk);
            return false;
        }
        return true;
    }
};

GeometryDashRuntimeOracle::GeometryDashRuntimeOracle(PlayLayer* layer, double stepDt)
  : m_impl(std::make_unique<Impl>(layer, stepDt)) {}

GeometryDashRuntimeOracle::~GeometryDashRuntimeOracle() = default;

void GeometryDashRuntimeOracle::setStepCallback(StepCallback callback) { m_impl->callback = std::move(callback); }
void GeometryDashRuntimeOracle::setStepDt(double value) { if (std::isfinite(value) && value > 0.0) m_impl->dt = value; }
double GeometryDashRuntimeOracle::stepDt() const { return m_impl->dt; }
std::string const& GeometryDashRuntimeOracle::lastError() const { return m_impl->error; }
RuntimeOracleValidation GeometryDashRuntimeOracle::validation() const { return m_impl->validation; }

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
    out.fingerprint = m_impl->makeCanonical(m_impl->buildSections()).hash;
    return out;
}

std::optional<UniversalToken> GeometryDashRuntimeOracle::capture() {
    auto* layer = m_impl->layer;
    if (!layer) return std::nullopt;

    const auto randomSeed = GameToolbox::getfast_srand();
    const auto replaySeed = layer->m_replayRandSeed;
    const auto sections = m_impl->buildSections();
    if (m_impl->lastCapturedSections) {
        if (m_impl->lastCapturedSections->effect != sections.effect) ++m_impl->validation.effectTransitionsObserved;
        if (m_impl->lastCapturedSections->rng != sections.rng) ++m_impl->validation.rngTransitionsObserved;
        if (m_impl->lastCapturedSections->dynamic != sections.dynamic) ++m_impl->validation.dynamicTransitionsObserved;
    }
    m_impl->lastCapturedSections = sections;
    auto* checkpoint = layer->createCheckpoint();
    // Snapshot creation must be observational. If GD touches RNG while making
    // a checkpoint, restore it immediately so search capture has no side effect.
    GameToolbox::fast_srand(randomSeed);
    layer->m_replayRandSeed = replaySeed;
    if (!checkpoint) {
        m_impl->error = "createCheckpoint returned null";
        return std::nullopt;
    }

    auto snap = std::make_shared<Impl::Snapshot>();
    snap->checkpoint = checkpoint;
    snap->randomSeed = randomSeed;
    snap->replaySeed = replaySeed;
    snap->attempts = layer->m_attempts;
    snap->practiceMode = layer->m_isPracticeMode;
    snap->extraDelta = layer->m_extraDelta;
    snap->timeWarp = layer->m_gameState.m_timeWarp;
    snap->levelTime = layer->m_gameState.m_levelTime;
    snap->currentProgress = layer->m_gameState.m_currentProgress;
    snap->action = m_impl->action;
    snap->sections = sections;
    snap->canonical = m_impl->makeCanonical(sections);
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
    GameToolbox::fast_srand(snap.randomSeed);
    layer->m_replayRandSeed = snap.replaySeed;
    return m_impl->verifyRestore(snap);
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

void GeometryDashRuntimeOracle::discard(UniversalToken token) { m_impl->snapshots.erase(token); }

std::optional<UniversalCanonicalState> GeometryDashRuntimeOracle::canonicalState(UniversalToken token) const {
    auto it = m_impl->snapshots.find(token);
    if (it == m_impl->snapshots.end() || !it->second) return std::nullopt;
    return it->second->canonical;
}

std::uint64_t GeometryDashRuntimeOracle::decisionEpoch() const {
    return m_impl ? m_impl->controlDecisionEpoch() : 0;
}

} // namespace autobot::presolve
