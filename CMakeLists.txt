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

# Performance-only patch #1. The upstream parallel-dedupe implementation is
# already order-preserving and bit-equivalent to its serial path; at the
# runtime's base cap (2000) the old >=4096 gate made that implementation
# unreachable. Lower only the dispatch threshold; no state transition, key,
# pruning rule, survivor ordering, or physics constant changes.
set(_autobot_cli "${AUTOBOT_GDSOLVER_ROOT}/dp/src/dp/cli.hpp")
file(READ "${_autobot_cli}" _autobot_cli_text)
set(_autobot_dedupe_old "&& kids.size() >= 4096;")
set(_autobot_dedupe_new "&& kids.size() >= 1024;")
string(FIND "${_autobot_cli_text}" "${_autobot_dedupe_old}" _autobot_dedupe_pos)
if(_autobot_dedupe_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver parallel-dedupe threshold anchor not found")
endif()
string(REPLACE
    "${_autobot_dedupe_old}"
    "${_autobot_dedupe_new}"
    _autobot_cli_text
    "${_autobot_cli_text}"
)
# Performance-only patch #2. A fixup replay compares model vs GD only
# through deathTick, but upstream replay kept simulating to the end of the level.
# Let replay honor the already-parsed --horizon only when explicitly supplied;
# all existing unbounded replay/search semantics remain unchanged.
set(_autobot_replay_loop_old "        for (long long t = t0 + 1; t <= tEnd; ++t) {")
set(_autobot_replay_loop_new [=[
        const long long replayEnd =
            horizon > 0 ? std::min(tEnd, t0 + horizon) : tEnd;
        for (long long t = t0 + 1; t <= replayEnd; ++t) {
]=])
string(FIND "${_autobot_cli_text}" "${_autobot_replay_loop_old}" _autobot_replay_loop_pos)
if(_autobot_replay_loop_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver replay loop anchor not found")
endif()
string(REPLACE "${_autobot_replay_loop_old}" "${_autobot_replay_loop_new}"
    _autobot_cli_text "${_autobot_cli_text}")

# Runtime profiling only. These counters are written to SearchOutcome after a
# solve; nothing in the search reads them back.
set(_autobot_search_stats_old [=[
    size_t maxAlive = 0;
    long long capHits = 0, capDropped = 0;
]=])
set(_autobot_search_stats_new [=[
    size_t maxAlive = 0;
    long long capHits = 0, capDropped = 0;
    long long perfStatesExpanded = 0, perfDedupHits = 0;
    unsigned long long perfPeakMemoryBytes = 0;
    double perfRejoinSeconds = 0.0;
]=])
string(FIND "${_autobot_cli_text}" "${_autobot_search_stats_old}" _autobot_search_stats_pos)
if(_autobot_search_stats_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver search stats anchor not found")
endif()
string(REPLACE "${_autobot_search_stats_old}" "${_autobot_search_stats_new}"
    _autobot_cli_text "${_autobot_cli_text}")

set(_autobot_layer_stats_old "        maxAlive = std::max(maxAlive, nxt.size());")
set(_autobot_layer_stats_new [=[
        perfStatesExpanded += nBorn;
        perfDedupHits += std::max<long long>(
            0, nBorn - nDied - static_cast<long long>(nxt.size()));
        const unsigned long long perfMemoryNow =
            static_cast<unsigned long long>(arena.capacity()) * sizeof(Node)
            + static_cast<unsigned long long>(cur.capacity()) * sizeof(State)
            + static_cast<unsigned long long>(nxt.capacity()) * sizeof(State)
            + static_cast<unsigned long long>(kids.capacity()) * sizeof(Child)
            + static_cast<unsigned long long>(gidx.capacity()) * sizeof(size_t)
            + static_cast<unsigned long long>(kidKeys.capacity()) * sizeof(uint64_t)
            + static_cast<unsigned long long>(kidFlag.capacity()) * sizeof(uint8_t)
            + static_cast<unsigned long long>(seen.bucket_count())
                * (sizeof(uint64_t) + sizeof(Slots));
        perfPeakMemoryBytes = std::max(perfPeakMemoryBytes, perfMemoryNow);
        maxAlive = std::max(maxAlive, nxt.size());
]=])
string(FIND "${_autobot_cli_text}" "${_autobot_layer_stats_old}" _autobot_layer_stats_pos)
if(_autobot_layer_stats_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver layer stats anchor not found")
endif()
string(REPLACE "${_autobot_layer_stats_old}" "${_autobot_layer_stats_new}"
    _autobot_cli_text "${_autobot_cli_text}")

set(_autobot_rejoin_call_old [=[
            const int ji = rejoinLayer(nxt, tt, [&](const State& s) {
                if (g_rjOldNode == 0xffffffffu) return false;
                uint32_t n = s.parent;
                for (long long k = tt; k > g_rjAfter; --k) n = arena[n].parent();
                return n == g_rjOldNode;
            }, [&](const State& s, const RefRow& r) {
                return !g_rjFull || (r.haveKey && keyOf(s, tt) == r.key);
            });
]=])
set(_autobot_rejoin_call_new [=[
            const auto perfRj0 = std::chrono::steady_clock::now();
            const int ji = rejoinLayer(nxt, tt, [&](const State& s) {
                if (g_rjOldNode == 0xffffffffu) return false;
                uint32_t n = s.parent;
                for (long long k = tt; k > g_rjAfter; --k) n = arena[n].parent();
                return n == g_rjOldNode;
            }, [&](const State& s, const RefRow& r) {
                return !g_rjFull || (r.haveKey && keyOf(s, tt) == r.key);
            });
            perfRejoinSeconds += std::chrono::duration<double>(
                std::chrono::steady_clock::now() - perfRj0).count();
]=])
string(FIND "${_autobot_cli_text}" "${_autobot_rejoin_call_old}" _autobot_rejoin_call_pos)
if(_autobot_rejoin_call_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver rejoin timing anchor not found")
endif()
string(REPLACE "${_autobot_rejoin_call_old}" "${_autobot_rejoin_call_new}"
    _autobot_cli_text "${_autobot_cli_text}")

set(_autobot_outcome_stats_old "    g_outcome.capHits = capHits;")
set(_autobot_outcome_stats_new [=[
    g_outcome.capHits = capHits;
    g_outcome.statesExpanded = perfStatesExpanded;
    g_outcome.dedupHits = perfDedupHits;
    g_outcome.statesDroppedByCap = capDropped;
    g_outcome.peakAliveStates = static_cast<long long>(maxAlive);
    g_outcome.peakMemoryBytes = perfPeakMemoryBytes;
    g_outcome.searchSimulatedPhysicsTicks = perfStatesExpanded;
    g_outcome.rejoinSeconds = perfRejoinSeconds;
]=])
string(FIND "${_autobot_cli_text}" "${_autobot_outcome_stats_old}" _autobot_outcome_stats_pos)
if(_autobot_outcome_stats_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver outcome stats anchor not found")
endif()
string(REPLACE "${_autobot_outcome_stats_old}" "${_autobot_outcome_stats_new}"
    _autobot_cli_text "${_autobot_cli_text}")

set(_autobot_witness_start_old [=[
    {
        std::ofstream tr(outPath + ".trace.csv");
]=])
set(_autobot_witness_start_new [=[
    {
        const auto perfCandidateReplay0 = std::chrono::steady_clock::now();
        std::ofstream tr(outPath + ".trace.csv");
]=])
string(FIND "${_autobot_cli_text}" "${_autobot_witness_start_old}" _autobot_witness_start_pos)
if(_autobot_witness_start_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver witness timing start anchor not found")
endif()
string(REPLACE "${_autobot_witness_start_old}" "${_autobot_witness_start_new}"
    _autobot_cli_text "${_autobot_cli_text}")

set(_autobot_witness_end_old [=[
        g_snapOut = nullptr;
    }

    if (!g_bandPath.empty()) {
]=])
set(_autobot_witness_end_new [=[
        g_snapOut = nullptr;
        g_outcome.candidateReplayTicks = static_cast<long long>(lvl.size());
        g_outcome.candidateReplaySeconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - perfCandidateReplay0).count();
    }

    if (!g_bandPath.empty()) {
]=])
string(FIND "${_autobot_cli_text}" "${_autobot_witness_end_old}" _autobot_witness_end_pos)
if(_autobot_witness_end_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver witness timing end anchor not found")
endif()
string(REPLACE "${_autobot_witness_end_old}" "${_autobot_witness_end_new}"
    _autobot_cli_text "${_autobot_cli_text}")

set(_autobot_replay_outcome_old "        g_outcome.replayDiedT = diedT;")
set(_autobot_replay_outcome_new [=[
        g_outcome.replayDiedT = diedT;
        g_outcome.replayTicks = std::max<long long>(0, lastT - t0);
        g_outcome.replayPotentialTicks = std::max<long long>(0, tEnd - t0);
]=])
string(FIND "${_autobot_cli_text}" "${_autobot_replay_outcome_old}" _autobot_replay_outcome_pos)
if(_autobot_replay_outcome_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver replay outcome anchor not found")
endif()
string(REPLACE "${_autobot_replay_outcome_old}" "${_autobot_replay_outcome_new}"
    _autobot_cli_text "${_autobot_cli_text}")

file(WRITE "${_autobot_cli}" "${_autobot_cli_text}")

set(_autobot_progress "${AUTOBOT_GDSOLVER_ROOT}/dp/src/dp/progress.hpp")
file(READ "${_autobot_progress}" _autobot_progress_text)
set(_autobot_progress_fields_old "    long long capHits = -1;  // -1 = no capstat line, i.e. the layer loop never ran")
set(_autobot_progress_fields_new [=[
    long long capHits = -1;  // -1 = no capstat line, i.e. the layer loop never ran
    // Profiling-only counters. The search never reads these back.
    long long statesExpanded = 0;
    long long dedupHits = 0;
    long long statesDroppedByCap = 0;
    long long peakAliveStates = 0;
    unsigned long long peakMemoryBytes = 0;
    long long searchSimulatedPhysicsTicks = 0;
    long long candidateReplayTicks = 0;
    double candidateReplaySeconds = 0.0;
    double rejoinSeconds = 0.0;
    // --replay accounting used by the bounded repair profiler.
    long long replayTicks = 0;
    long long replayPotentialTicks = 0;
]=])
string(FIND "${_autobot_progress_text}" "${_autobot_progress_fields_old}" _autobot_progress_fields_pos)
if(_autobot_progress_fields_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver progress fields anchor not found")
endif()
string(REPLACE "${_autobot_progress_fields_old}" "${_autobot_progress_fields_new}"
    _autobot_progress_text "${_autobot_progress_text}")

set(_autobot_progress_reset_old [=[
        deepT = -1; deepX = -1.0; capHits = -1; replayDiedT = -1; cancelT = -1; rejoinT = -1;
]=])
set(_autobot_progress_reset_new [=[
        deepT = -1; deepX = -1.0; capHits = -1; replayDiedT = -1; cancelT = -1; rejoinT = -1;
        statesExpanded = 0; dedupHits = 0; statesDroppedByCap = 0;
        peakAliveStates = 0; peakMemoryBytes = 0; searchSimulatedPhysicsTicks = 0;
        candidateReplayTicks = 0; candidateReplaySeconds = 0.0; rejoinSeconds = 0.0;
        replayTicks = 0; replayPotentialTicks = 0;
]=])
string(FIND "${_autobot_progress_text}" "${_autobot_progress_reset_old}" _autobot_progress_reset_pos)
if(_autobot_progress_reset_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver progress reset anchor not found")
endif()
string(REPLACE "${_autobot_progress_reset_old}" "${_autobot_progress_reset_new}"
    _autobot_progress_text "${_autobot_progress_text}")
file(WRITE "${_autobot_progress}" "${_autobot_progress_text}")

set(_autobot_bridge_h "${AUTOBOT_GDSOLVER_ROOT}/src/mod/dp_bridge.hpp")
file(READ "${_autobot_bridge_h}" _autobot_bridge_h_text)
set(_autobot_bridge_fields_old "    long long capHits = -1;")
set(_autobot_bridge_fields_new [=[
    long long capHits = -1;
    long long statesExpanded = 0, dedupHits = 0, statesDroppedByCap = 0;
    long long peakAliveStates = 0, searchSimulatedPhysicsTicks = 0;
    unsigned long long peakMemoryBytes = 0;
    long long candidateReplayTicks = 0, replayTicks = 0, replayPotentialTicks = 0;
    double candidateReplaySeconds = 0.0, rejoinSeconds = 0.0;
]=])
string(FIND "${_autobot_bridge_h_text}" "${_autobot_bridge_fields_old}" _autobot_bridge_fields_pos)
if(_autobot_bridge_fields_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver bridge fields anchor not found")
endif()
string(REPLACE "${_autobot_bridge_fields_old}" "${_autobot_bridge_fields_new}"
    _autobot_bridge_h_text "${_autobot_bridge_h_text}")
file(WRITE "${_autobot_bridge_h}" "${_autobot_bridge_h_text}")

set(_autobot_bridge_cpp "${AUTOBOT_GDSOLVER_ROOT}/src/mod/dp_bridge.cpp")
file(READ "${_autobot_bridge_cpp}" _autobot_bridge_cpp_text)
set(_autobot_bridge_copy_old [=[
    o.capHits = dp::g_outcome.capHits;
    o.cancelT = dp::g_outcome.cancelT;
]=])
set(_autobot_bridge_copy_new [=[
    o.capHits = dp::g_outcome.capHits;
    o.statesExpanded = dp::g_outcome.statesExpanded;
    o.dedupHits = dp::g_outcome.dedupHits;
    o.statesDroppedByCap = dp::g_outcome.statesDroppedByCap;
    o.peakAliveStates = dp::g_outcome.peakAliveStates;
    o.peakMemoryBytes = dp::g_outcome.peakMemoryBytes;
    o.searchSimulatedPhysicsTicks = dp::g_outcome.searchSimulatedPhysicsTicks;
    o.candidateReplayTicks = dp::g_outcome.candidateReplayTicks;
    o.candidateReplaySeconds = dp::g_outcome.candidateReplaySeconds;
    o.rejoinSeconds = dp::g_outcome.rejoinSeconds;
    o.replayTicks = dp::g_outcome.replayTicks;
    o.replayPotentialTicks = dp::g_outcome.replayPotentialTicks;
    o.cancelT = dp::g_outcome.cancelT;
]=])
string(FIND "${_autobot_bridge_cpp_text}" "${_autobot_bridge_copy_old}" _autobot_bridge_copy_pos)
if(_autobot_bridge_copy_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver bridge copy anchor not found")
endif()
string(REPLACE "${_autobot_bridge_copy_old}" "${_autobot_bridge_copy_new}"
    _autobot_bridge_cpp_text "${_autobot_bridge_cpp_text}")
file(WRITE "${_autobot_bridge_cpp}" "${_autobot_bridge_cpp_text}")

set(_autobot_repair "${autobot_gdsolver_upstream_SOURCE_DIR}/src/mod/repair.hpp")
file(READ "${_autobot_repair}" _autobot_repair_text)
set(_autobot_fixup_args_old [=[
    std::vector<std::string> a{"--replay", g_planPath, "--start", startArgStr,
                               "--out", base,
                               "--cap", std::to_string(kCap),
]=])
set(_autobot_fixup_args_new [=[
    // The recorder never reads model rows after GD's death. Bound this replay
    // to exactly the compared interval instead of simulating the unused tail.
    const long long compareTicks = std::max<long long>(1, deathTick - t0);
    std::vector<std::string> a{"--replay", g_planPath, "--start", startArgStr,
                               "--horizon", std::to_string(compareTicks),
                               "--out", base,
                               "--cap", std::to_string(kCap),
]=])
string(FIND "${_autobot_repair_text}" "${_autobot_fixup_args_old}" _autobot_fixup_args_pos)
if(_autobot_fixup_args_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver fixup argv anchor not found")
endif()
string(REPLACE "${_autobot_fixup_args_old}" "${_autobot_fixup_args_new}"
    _autobot_repair_text "${_autobot_repair_text}")
file(WRITE "${_autobot_repair}" "${_autobot_repair_text}")

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
