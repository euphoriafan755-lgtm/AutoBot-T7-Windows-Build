#pragma once

#include <string_view>

#ifndef AUTOBOT_VERSION
#define AUTOBOT_VERSION "DEV"
#endif
#ifndef AUTOBOT_BUILD_COMMIT
#define AUTOBOT_BUILD_COMMIT "DEV"
#endif
#ifndef AUTOBOT_BUILD_DATE
#define AUTOBOT_BUILD_DATE "DEV"
#endif

namespace autobot::core::build {

inline constexpr std::string_view version() { return AUTOBOT_VERSION; }
inline constexpr std::string_view commit() { return AUTOBOT_BUILD_COMMIT; }
inline constexpr std::string_view buildDate() { return AUTOBOT_BUILD_DATE; }
inline constexpr std::string_view shortCommit() {
    constexpr std::string_view value = AUTOBOT_BUILD_COMMIT;
    return value.size() > 7 ? value.substr(0, 7) : value;
}

} // namespace autobot::core::build
