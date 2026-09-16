#include "autobot/world/LevelParser.hpp"

#include <Geode/Geode.hpp>

#include <algorithm>
#include <cstdlib>
#include <string>
#include <unordered_map>
#include <vector>

using namespace geode::prelude;

namespace autobot::world {
namespace {

using SaveProperties = std::unordered_map<int, std::string>;

SaveProperties parseSaveProperties(gd::string const& encoded) {
    SaveProperties result;
    std::string text(encoded.c_str());
    std::vector<std::string> tokens;
    std::size_t start = 0;
    while (start <= text.size()) {
        auto end = text.find(',', start);
        if (end == std::string::npos) end = text.size();
        tokens.emplace_back(text.substr(start, end - start));
        if (end == text.size()) break;
        start = end + 1;
    }
    for (std::size_t i = 0; i + 1 < tokens.size(); i += 2) {
        char* keyEnd = nullptr;
        const long key = std::strtol(tokens[i].c_str(), &keyEnd, 10);
        if (!keyEnd || *keyEnd != '\0') continue;
        result[static_cast<int>(key)] = tokens[i + 1];
    }
    return result;
}

double propertyDouble(SaveProperties const& values, int key, double fallback) {
    auto it = values.find(key);
    if (it == values.end()) return fallback;
    char* end = nullptr;
    const double value = std::strtod(it->second.c_str(), &end);
    return end && *end == '\0' ? value : fallback;
}

int propertyInt(SaveProperties const& values, int key, int fallback) {
    return static_cast<int>(propertyDouble(values, key, static_cast<double>(fallback)));
}

bool propertyBool(SaveProperties const& values, int key, bool fallback) {
    return propertyInt(values, key, fallback ? 1 : 0) != 0;
}

TriggerKind triggerKindForObjectID(int objectID) {
    switch (objectID) {
        case 901: return TriggerKind::Move;
        case 1346: return TriggerKind::Rotate;
        case 2067: return TriggerKind::Scale;
        case 1049: return TriggerKind::Toggle;
        case 1268: return TriggerKind::Spawn;
        case 1347: return TriggerKind::Follow;
        case 1814: return TriggerKind::FollowPlayerY;
        case 1616: return TriggerKind::Stop;
        case 1935: return TriggerKind::TimeWarp;
        case 2066: return TriggerKind::Gravity;
        default: return TriggerKind::None;
    }
}

TriggerDescriptor snapshotTrigger(EffectGameObject* effect, PlayLayer* playLayer) {
    TriggerDescriptor trigger{};
    if (!effect || !playLayer) return trigger;

    trigger.kind = triggerKindForObjectID(effect->m_objectID);
    trigger.gameplayRelevant = trigger.kind != TriggerKind::None;
    if (!trigger.gameplayRelevant) return trigger;

    const auto properties = parseSaveProperties(effect->getSaveString(playLayer));
    trigger.targetGroupID = effect->m_targetGroupID;
    trigger.centerGroupID = effect->m_centerGroupID;
    trigger.durationSeconds = std::max(0.0, static_cast<double>(effect->m_duration));
    trigger.spawnDelaySeconds = std::max(
        0.0,
        propertyDouble(properties, 63, static_cast<double>(effect->m_spawnTriggerDelay))
    );
    trigger.spawnTriggered = effect->m_isSpawnTriggered;
    trigger.touchTriggered = effect->m_isTouchTriggered;
    trigger.multiTriggered = effect->m_isMultiTriggered;
    trigger.moveX = effect->m_moveOffset.x;
    trigger.moveY = effect->m_moveOffset.y;
    trigger.rotationDegrees = static_cast<double>(effect->m_rotationDegrees)
        + static_cast<double>(effect->m_times360) * 360.0;
    trigger.followXMod = effect->m_followXMod;
    trigger.followYMod = effect->m_followYMod;
    trigger.followYSpeed = effect->m_followYSpeed;
    trigger.followYDelay = effect->m_followYDelay;
    trigger.followYOffset = effect->m_followYOffset;
    trigger.followYMaxSpeed = effect->m_followYMaxSpeed;
    trigger.toggleOn = effect->m_activateGroup;
    trigger.timeWarp = effect->m_timeWarpTimeMod > 0.0f
        ? static_cast<double>(effect->m_timeWarpTimeMod)
        : 1.0;
    trigger.gravityValue = effect->m_gravityValue != 0.0f
        ? static_cast<double>(effect->m_gravityValue)
        : 1.0;

    // 2.2 Scale trigger target multipliers live in save properties 150/151;
    // 153/154 are the divide-by-value switches. Reading the serialized
    // properties avoids guessing at undocumented runtime member aliases.
    trigger.scaleX = propertyDouble(properties, 150, 1.0);
    trigger.scaleY = propertyDouble(properties, 151, 1.0);
    trigger.divideScaleX = propertyBool(properties, 153, false);
    trigger.divideScaleY = propertyBool(properties, 154, false);

    // Stop trigger property 580: 0=stop, 1=pause, 2=resume.
    if (trigger.kind == TriggerKind::Stop) {
        switch (propertyInt(properties, 580, 0)) {
            case 1: trigger.commandMode = TriggerCommandMode::Pause; break;
            case 2: trigger.commandMode = TriggerCommandMode::Resume; break;
            default: trigger.commandMode = TriggerCommandMode::Stop; break;
        }
    }

    // Touch-only triggers need contact semantics rather than X crossing. The
    // causal engine marks them uncertain until that contact is actually part
    // of the simulated path instead of silently assuming activation.
    trigger.uncertain = trigger.touchTriggered;
    return trigger;
}

bool isSpeedPortalObjectID(int objectID) {
    switch (objectID) {
        case 200:
        case 201:
        case 202:
        case 203:
        case 1334:
            return true;
        default:
            return false;
    }
}

void incrementCount(StaticWorld& world, GameplayObjectType type) {
    switch (type) {
        case GameplayObjectType::Solid: ++world.solids; break;
        case GameplayObjectType::Hazard: ++world.hazards; break;
        case GameplayObjectType::Orb: ++world.orbs; break;
        case GameplayObjectType::Pad: ++world.pads; break;
        case GameplayObjectType::Portal: ++world.portals; break;
        case GameplayObjectType::Decoration: ++world.decorations; break;
        case GameplayObjectType::Unknown: ++world.unknown; break;
    }
}

} // namespace

GameplayObjectType LevelParser::classify(GameObjectType type, int objectID) {
    if (isSpeedPortalObjectID(objectID)) return GameplayObjectType::Portal;

    switch (type) {
        case GameObjectType::Solid:
        case GameObjectType::Slope:
        case GameObjectType::Breakable:
            return GameplayObjectType::Solid;
        case GameObjectType::Hazard:
        case GameObjectType::AnimatedHazard:
            return GameplayObjectType::Hazard;
        case GameObjectType::YellowJumpRing:
        case GameObjectType::PinkJumpRing:
        case GameObjectType::GravityRing:
        case GameObjectType::GreenRing:
        case GameObjectType::DropRing:
        case GameObjectType::RedJumpRing:
        case GameObjectType::CustomRing:
        case GameObjectType::DashRing:
        case GameObjectType::GravityDashRing:
        case GameObjectType::SpiderOrb:
        case GameObjectType::TeleportOrb:
            return GameplayObjectType::Orb;
        case GameObjectType::YellowJumpPad:
        case GameObjectType::PinkJumpPad:
        case GameObjectType::GravityPad:
        case GameObjectType::RedJumpPad:
        case GameObjectType::SpiderPad:
            return GameplayObjectType::Pad;
        case GameObjectType::InverseGravityPortal:
        case GameObjectType::NormalGravityPortal:
        case GameObjectType::ShipPortal:
        case GameObjectType::CubePortal:
        case GameObjectType::InverseMirrorPortal:
        case GameObjectType::NormalMirrorPortal:
        case GameObjectType::BallPortal:
        case GameObjectType::RegularSizePortal:
        case GameObjectType::MiniSizePortal:
        case GameObjectType::UfoPortal:
        case GameObjectType::DualPortal:
        case GameObjectType::SoloPortal:
        case GameObjectType::WavePortal:
        case GameObjectType::RobotPortal:
        case GameObjectType::TeleportPortal:
        case GameObjectType::SpiderPortal:
        case GameObjectType::SwingPortal:
        case GameObjectType::GravityTogglePortal:
            return GameplayObjectType::Portal;
        case GameObjectType::Decoration:
            return GameplayObjectType::Decoration;
        default:
            return GameplayObjectType::Unknown;
    }
}

V01Support LevelParser::classifyV01Support(GameObjectType type, int objectID) {
    if (isSpeedPortalObjectID(objectID)) return V01Support::Supported;

    switch (type) {
        case GameObjectType::Solid:
        case GameObjectType::Slope:
        case GameObjectType::Hazard:
        case GameObjectType::AnimatedHazard:
        case GameObjectType::YellowJumpPad:
        case GameObjectType::PinkJumpPad:
        case GameObjectType::GravityPad:
        case GameObjectType::RedJumpPad:
        case GameObjectType::YellowJumpRing:
        case GameObjectType::PinkJumpRing:
        case GameObjectType::GravityRing:
        case GameObjectType::GreenRing:
        case GameObjectType::RedJumpRing:
        case GameObjectType::InverseGravityPortal:
        case GameObjectType::NormalGravityPortal:
        case GameObjectType::CubePortal:
        case GameObjectType::RegularSizePortal:
        case GameObjectType::MiniSizePortal:
            return V01Support::Supported;
        case GameObjectType::Decoration:
            return V01Support::NonGameplay;
        default:
            return V01Support::NotSupported;
    }
}

bool LevelParser::snapshotObjectAt(
    PlayLayer* playLayer,
    std::size_t objectArrayIndex,
    WorldObject& out
) {
    if (!playLayer || !playLayer->m_objects) return false;
    if (objectArrayIndex >= playLayer->m_objects->count()) return false;

    auto* object = static_cast<GameObject*>(playLayer->m_objects->objectAtIndex(objectArrayIndex));
    if (!object) return false;

    const auto rect = object->getObjectRect();
    const auto realPosition = object->getRealPosition();
    const auto nodePosition = object->getPosition();
    const auto anchor = object->getAnchorPoint();
    const auto contentSize = object->getContentSize();

    WorldObject copy{};
    copy.objectID = object->m_objectID;
    copy.uniqueID = object->m_uniqueID;
    copy.playLayerObjectIndex = objectArrayIndex;
    copy.rawGameObjectType = static_cast<int>(object->m_objectType);
    copy.type = classify(object->m_objectType, object->m_objectID);
    copy.v01Support = classifyV01Support(object->m_objectType, object->m_objectID);
    copy.x = realPosition.x;
    copy.y = realPosition.y;
    copy.nodeX = nodePosition.x;
    copy.nodeY = nodePosition.y;
    copy.rotation = object->getRotation();
    copy.rotationX = object->getRotationX();
    copy.rotationY = object->getRotationY();
    copy.scaleX = object->getScaleX();
    copy.scaleY = object->getScaleY();
    copy.anchorX = anchor.x;
    copy.anchorY = anchor.y;
    copy.contentWidth = contentSize.width;
    copy.contentHeight = contentSize.height;
    copy.objectRect = WorldRect{rect.origin.x, rect.origin.y, rect.size.width, rect.size.height};
    copy.enabled = !object->m_isDisabled;
    copy.groupDisabled = object->m_isGroupDisabled;
    copy.noTouch = object->m_isNoTouch;
    copy.passable = object->m_isPassable;
    copy.flipX = object->isFlipX();
    copy.flipY = object->isFlipY();
    copy.slope = object->m_objectType == GameObjectType::Slope;
    copy.groupCount = object->m_groupCount;
    if (object->m_groups && object->m_groupCount > 0) {
        const auto count = std::min<int>(object->m_groupCount, 10);
        copy.groups.reserve(static_cast<std::size_t>(count));
        for (int i = 0; i < count; ++i) {
            const int group = static_cast<int>((*object->m_groups)[static_cast<std::size_t>(i)]);
            if (group > 0) copy.groups.push_back(group);
        }
    }
    if (object->m_isTrigger) {
        auto* effect = typeinfo_cast<EffectGameObject*>(object);
        if (effect) copy.trigger = snapshotTrigger(effect, playLayer);
    }

    out = copy;
    return true;
}

StaticWorld LevelParser::parse(PlayLayer* playLayer) {
    StaticWorld world{};

    if (!playLayer) {
        world.error = "PlayLayer is null";
        return world;
    }
    if (!playLayer->m_objects) {
        world.error = "PlayLayer::m_objects is null";
        return world;
    }

    const auto objectCount = static_cast<std::size_t>(playLayer->m_objects->count());
    world.objects.reserve(objectCount);

    for (std::size_t objectArrayIndex = 0; objectArrayIndex < objectCount; ++objectArrayIndex) {
        WorldObject copy{};
        if (!snapshotObjectAt(playLayer, objectArrayIndex, copy)) continue;

        incrementCount(world, copy.type);
        if (copy.v01Support == V01Support::NotSupported && copy.type != GameplayObjectType::Decoration) {
            ++world.unsupportedGameplay;
        }
        world.objects.push_back(copy);
    }

    world.parsed = true;
    return world;
}

} // namespace autobot::world
