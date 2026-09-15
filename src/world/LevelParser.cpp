#include "autobot/world/LevelParser.hpp"

#include <Geode/Geode.hpp>

using namespace geode::prelude;

namespace autobot::world {
namespace {

bool isSpeedPortalObjectID(int objectID) {
    // Vanilla speed portals: slow, normal, fast, faster, fastest.
    // Classification only; physics behavior is intentionally NOT implemented yet.
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
    if (isSpeedPortalObjectID(objectID)) {
        return GameplayObjectType::Portal;
    }

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
    if (isSpeedPortalObjectID(objectID)) {
        return V01Support::Supported;
    }

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
            // This includes non-Cube gamemode portals, dual, teleport, dash,
            // spider-specific objects, breakables, modifiers, and unknowns.
            return V01Support::NotSupported;
    }
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

    world.objects.reserve(playLayer->m_objects->count());

    for (auto* object : geode::cocos::CCArrayExt<GameObject, false>(playLayer->m_objects)) {
        if (!object) continue;

        const auto rect = object->getObjectRect();
        const auto position = object->getRealPosition();

        WorldObject copy{};
        copy.objectID = object->m_objectID;
        copy.rawGameObjectType = static_cast<int>(object->m_objectType);
        copy.type = classify(object->m_objectType, object->m_objectID);
        copy.v01Support = classifyV01Support(object->m_objectType, object->m_objectID);
        copy.x = position.x;
        copy.y = position.y;
        copy.rotation = object->getRotation();
        copy.scaleX = object->getScaleX();
        copy.scaleY = object->getScaleY();
        copy.objectRect = WorldRect{rect.origin.x, rect.origin.y, rect.size.width, rect.size.height};
        copy.enabled = !object->m_isDisabled;
        copy.slope = object->m_objectType == GameObjectType::Slope;

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
