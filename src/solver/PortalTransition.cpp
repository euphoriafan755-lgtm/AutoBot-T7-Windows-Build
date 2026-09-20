#include "autobot/solver/PortalTransition.hpp"

#if __has_include(<Geode/Geode.hpp>)
#include <Geode/Geode.hpp>

using namespace geode::prelude;

namespace autobot::solver {
namespace {

bool isSpeedPortalID(int objectID) {
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

core::GameMode effectMode(PortalEffect effect) {
    switch (effect) {
        case PortalEffect::ModeCube: return core::GameMode::Cube;
        case PortalEffect::ModeShip: return core::GameMode::Ship;
        case PortalEffect::ModeBall: return core::GameMode::Ball;
        case PortalEffect::ModeUfo: return core::GameMode::Ufo;
        case PortalEffect::ModeWave: return core::GameMode::Wave;
        case PortalEffect::ModeRobot: return core::GameMode::Robot;
        case PortalEffect::ModeSpider: return core::GameMode::Spider;
        case PortalEffect::ModeSwing: return core::GameMode::Swing;
        default: return core::GameMode::Unknown;
    }
}

} // namespace

PortalEffect PortalTransitionResolver::resolve(world::CollisionPrimitive const& primitive) {
    if (primitive.classification != world::GameplayObjectType::Portal) {
        return PortalEffect::None;
    }
    if (isSpeedPortalID(primitive.objectID)) return PortalEffect::SpeedChange;

    const auto type = static_cast<GameObjectType>(primitive.rawGameObjectType);
    switch (type) {
        case GameObjectType::InverseGravityPortal: return PortalEffect::GravityInverse;
        case GameObjectType::NormalGravityPortal: return PortalEffect::GravityNormal;
        case GameObjectType::GravityTogglePortal: return PortalEffect::GravityToggle;
        case GameObjectType::CubePortal: return PortalEffect::ModeCube;
        case GameObjectType::ShipPortal: return PortalEffect::ModeShip;
        case GameObjectType::BallPortal: return PortalEffect::ModeBall;
        case GameObjectType::UfoPortal: return PortalEffect::ModeUfo;
        case GameObjectType::WavePortal: return PortalEffect::ModeWave;
        case GameObjectType::RobotPortal: return PortalEffect::ModeRobot;
        case GameObjectType::SpiderPortal: return PortalEffect::ModeSpider;
        case GameObjectType::SwingPortal: return PortalEffect::ModeSwing;
        case GameObjectType::RegularSizePortal: return PortalEffect::SizeNormal;
        case GameObjectType::MiniSizePortal: return PortalEffect::SizeMini;
        case GameObjectType::DualPortal: return PortalEffect::Dual;
        case GameObjectType::SoloPortal: return PortalEffect::Solo;
        case GameObjectType::TeleportPortal: return PortalEffect::Teleport;
        case GameObjectType::InverseMirrorPortal:
        case GameObjectType::NormalMirrorPortal:
            return PortalEffect::Unknown;
        default:
            return PortalEffect::Unknown;
    }
}

PortalApplication PortalTransitionResolver::apply(
    world::CollisionPrimitive const& primitive,
    SimState& state
) {
    PortalApplication result{};
    result.effect = resolve(primitive);

    switch (result.effect) {
        case PortalEffect::GravityInverse:
            state.upsideDown = true;
            result.stateChanged = true;
            break;
        case PortalEffect::GravityNormal:
            state.upsideDown = false;
            result.stateChanged = true;
            break;
        case PortalEffect::GravityToggle:
            state.upsideDown = !state.upsideDown;
            result.stateChanged = true;
            break;
        case PortalEffect::SizeNormal:
            state.mini = false;
            result.stateChanged = true;
            break;
        case PortalEffect::SizeMini:
            state.mini = true;
            result.stateChanged = true;
            break;
        case PortalEffect::ModeCube:
        case PortalEffect::ModeShip:
        case PortalEffect::ModeBall:
        case PortalEffect::ModeUfo:
        case PortalEffect::ModeWave:
        case PortalEffect::ModeRobot:
        case PortalEffect::ModeSpider:
        case PortalEffect::ModeSwing: {
            const auto nextMode = effectMode(result.effect);
            result.modeChanged = nextMode != state.mode;
            state.mode = nextMode;
            state.grounded = false;
            result.stateChanged = true;
            break;
        }
        case PortalEffect::SpeedChange:
            // The portal is recognized, but exact speed transition is left to
            // runtime state feedback until validated for GD 2.2081.
            result.modelUncertain = true;
            break;
        case PortalEffect::Teleport:
        case PortalEffect::Dual:
        case PortalEffect::Solo:
        case PortalEffect::Unknown:
            result.modelUncertain = true;
            break;
        case PortalEffect::None:
            break;
    }
    return result;
}

} // namespace autobot::solver

#endif // __has_include(<Geode/Geode.hpp>)
