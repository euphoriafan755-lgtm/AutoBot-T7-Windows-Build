#include <Geode/Geode.hpp>

#include "autobot/core/BuildInfo.hpp"

using namespace geode::prelude;

$on_mod(Loaded) {
    log::info(
        "AutoBot T7 v{} Build {} ({}) loaded",
        autobot::core::build::version(),
        autobot::core::build::shortCommit(),
        autobot::core::build::buildDate()
    );
}
