#pragma once

#include "autobot/core/GameSnapshot.hpp"
#include "autobot/solver/SolverTypes.hpp"
#include "autobot/world/CollisionTypes.hpp"

namespace autobot::solver {

enum class PortalEffect {
    None,
    GravityInverse,
    GravityNormal,
    GravityToggle,
    ModeCube,
    ModeShip,
    ModeBall,
    ModeUfo,
    ModeWave,
    ModeRobot,
    ModeSpider,
    ModeSwing,
    SizeNormal,
    SizeMini,
    Dual,
    Solo,
    Teleport,
    SpeedChange,
    Unknown,
};

struct PortalApplication {
    PortalEffect effect = PortalEffect::None;
    bool stateChanged = false;
    bool modeChanged = false;
    bool modelUncertain = false;
};

class PortalTransitionResolver final {
public:
    [[nodiscard]] static PortalEffect resolve(world::CollisionPrimitive const& primitive);
    [[nodiscard]] static PortalApplication apply(
        world::CollisionPrimitive const& primitive,
        SimState& state
    );
};

} // namespace autobot::solver
