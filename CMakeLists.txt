cmake_minimum_required(VERSION 3.21)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)

project(AutoBotT7 VERSION 0.2.0 LANGUAGES CXX)

# Build #4 solver redesign:
# The previous realtime MPC / GlobalPlanner / UniversalSearch implementation
# remains in this repository only as historical source. It is intentionally NOT
# part of the AutoBotT7 target. A single reachability + real-GD-verification
# solver owns gameplay input.
include(FetchContent)
if(POLICY CMP0169)
    cmake_policy(SET CMP0169 OLD)
endif()

set(AUTOBOT_GDSOLVER_COMMIT "4fb6a2365d3623b211913a5a21de308e49c5fce2" CACHE STRING
    "Pinned gdsolver revision adapted by AutoBot T7")

FetchContent_Declare(
    autobot_gdsolver_upstream
    GIT_REPOSITORY https://github.com/gdsolver/gdsolver.git
    GIT_TAG ${AUTOBOT_GDSOLVER_COMMIT}
    GIT_SHALLOW FALSE
    GIT_PROGRESS TRUE
)

FetchContent_GetProperties(autobot_gdsolver_upstream)
if(NOT autobot_gdsolver_upstream_POPULATED)
    FetchContent_Populate(autobot_gdsolver_upstream)
endif()

set(AUTOBOT_GDSOLVER_ROOT "${autobot_gdsolver_upstream_SOURCE_DIR}")

# AutoBot is hands-off by default: entering a level begins a cold Solve session.
# The upstream panel still lets the user switch back to Normal/Replay manually.
set(_fxcensus "${AUTOBOT_GDSOLVER_ROOT}/src/mod/fxcensus.hpp")
file(READ "${_fxcensus}" _fxcensus_text)
string(REPLACE
    "inline int g_uiMode = UI_MODE_NORMAL;"
    "inline int g_uiMode = UI_MODE_SOLVE;"
    _fxcensus_text
    "${_fxcensus_text}"
)
if(NOT _fxcensus_text MATCHES "inline int g_uiMode = UI_MODE_SOLVE;")
    message(FATAL_ERROR "Pinned gdsolver UI mode patch did not apply")
endif()
file(WRITE "${_fxcensus}" "${_fxcensus_text}")

# Identity-only patch. Solver behaviour remains the pinned upstream behaviour.
set(_panel "${AUTOBOT_GDSOLVER_ROOT}/src/mod/ui_panel.hpp")
file(READ "${_panel}" _panel_text)
string(REPLACE "\"GDSOLVER\"" "\"AUTOBOT T7\"" _panel_text "${_panel_text}")
file(WRITE "${_panel}" "${_panel_text}")

file(GLOB_RECURSE AUTOBOT_SOLVER_SOURCES CONFIGURE_DEPENDS
    "${AUTOBOT_GDSOLVER_ROOT}/src/*.cpp"
)

if(NOT AUTOBOT_SOLVER_SOURCES)
    message(FATAL_ERROR "Pinned gdsolver sources were not populated")
endif()

add_library(${PROJECT_NAME} SHARED ${AUTOBOT_SOLVER_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    "${AUTOBOT_GDSOLVER_ROOT}/src"
    "${AUTOBOT_GDSOLVER_ROOT}/dp/src"
)

# Keep the embedded DP translation unit optimized the same way as upstream.
if(MSVC)
    set_source_files_properties(
        "${AUTOBOT_GDSOLVER_ROOT}/src/mod/dp_bridge.cpp"
        PROPERTIES COMPILE_OPTIONS "$<$<CONFIG:RelWithDebInfo>:/Ob2>"
    )
endif()

target_compile_definitions(${PROJECT_NAME} PRIVATE
    AUTOBOT_VERSION="${PROJECT_VERSION}"
    AUTOBOT_GDSOLVER_COMMIT="${AUTOBOT_GDSOLVER_COMMIT}"
)

if (DEFINED GEODE_SDK)
    set(AUTOBOT_GEODE_SDK "${GEODE_SDK}")
elseif (DEFINED ENV{GEODE_SDK})
    set(AUTOBOT_GEODE_SDK "$ENV{GEODE_SDK}")
else()
    message(FATAL_ERROR "Geode SDK not found. Set GEODE_SDK or GEODE_SDK environment variable.")
endif()

message(STATUS "AutoBot T7 solver source: gdsolver@${AUTOBOT_GDSOLVER_COMMIT}")
message(STATUS "Legacy AutoBot solver source: PRESENT BUT NOT BUILT")
message(STATUS "Using Geode SDK: ${AUTOBOT_GEODE_SDK}")

add_subdirectory(${AUTOBOT_GEODE_SDK} ${CMAKE_CURRENT_BINARY_DIR}/geode)
setup_geode_mod(${PROJECT_NAME})


option(AUTOBOT_BUILD_SOLVER_TESTS "Build Build #4 functional solver gates" OFF)
if(AUTOBOT_BUILD_SOLVER_TESTS)
    add_executable(autobot-build4-functional-tests
        tests/build4/ReachabilityFunctionalTest.cpp
    )
    add_executable(autobot-solve-perf-profile
        tests/perf/SolvePerfProfile.cpp
    )
    target_include_directories(autobot-build4-functional-tests PRIVATE
        "${AUTOBOT_GDSOLVER_ROOT}/dp/src"
    )
    target_include_directories(autobot-solve-perf-profile PRIVATE
        "${AUTOBOT_GDSOLVER_ROOT}/dp/src"
    )
    if(MSVC)
        target_compile_options(autobot-build4-functional-tests PRIVATE /W4)
        target_compile_options(autobot-solve-perf-profile PRIVATE /O2 /W4)
    else()
        target_compile_options(autobot-build4-functional-tests PRIVATE -Wall -Wextra)
        target_compile_options(autobot-solve-perf-profile PRIVATE -O3 -Wall -Wextra)
    endif()
endif()
