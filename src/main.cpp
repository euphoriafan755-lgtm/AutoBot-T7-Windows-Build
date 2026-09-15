#include <Geode/Geode.hpp>

using namespace geode::prelude;

$on_mod(Loaded) {
    log::info("AutoBot T7 V0.1 Zero-Shot Core loaded");
    log::info("Scope: GameStateReader + Diagnostic HUD + initial LevelParser");
    log::info("Planner, physics simulator, collision world, inputs and trajectory overlay are NOT implemented in this delivery");
}
