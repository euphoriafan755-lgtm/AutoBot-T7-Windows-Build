#include <Geode/Geode.hpp>

using namespace geode::prelude;

$on_mod(Loaded) {
    log::info("AutoBot T7 V0.1 Zero-Shot Core loaded");
    log::info("Scope: direct game-state reading + initial LevelParser. No physics/planner/input execution yet.");
}
