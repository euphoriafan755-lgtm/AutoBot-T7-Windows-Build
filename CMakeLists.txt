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

# End-to-end runtime profiling. All fields are observational only: they are
# written to result.txt and never consulted by solver/repair decisions.
set(_autobot_perf_insert_old "constexpr const char* kThreads = \"8\";")
set(_autobot_perf_insert_new [=[
constexpr const char* kThreads = "8";

struct RuntimePerf {
    using Clock = std::chrono::steady_clock;
    Clock::time_point totalStart{}, verifyStart{}, finalPrepStart{};
    bool totalActive = false, verifyActive = false, finalPrepActive = false;
    bool reported = false;

    double modelBuildSeconds = 0.0;
    double initialSearchSeconds = 0.0;
    double candidateReplaySeconds = 0.0;
    double realGdVerificationSeconds = 0.0;
    double deathDivergenceSeconds = 0.0;
    double recordFixupsSeconds = 0.0;
    double fixupResimulationSeconds = 0.0;
    double fixupPassSeconds = 0.0;       // matching/scan only, excludes resim
    double reanchorSeconds = 0.0;
    double suffixSearchSeconds = 0.0;
    double rejoinSeconds = 0.0;
    double finalReplayPrepSeconds = 0.0;

    long long statesExpanded = 0;
    long long dedupHits = 0;
    long long capHits = 0;
    long long statesDroppedByCap = 0;
    long long peakAliveStates = 0;
    unsigned long long peakMemoryBytes = 0;

    long long verificationAttempts = 0;
    long long repairCount = 0;
    long long reanchors = 0;
    long long rejoins = 0;
    long long totalSimulatedPhysicsTicks = 0;
    long long wastedTicksAvoided = 0;
};

inline RuntimePerf g_perf;

struct PerfScope {
    double* dst;
    RuntimePerf::Clock::time_point start;
    explicit PerfScope(double& d) : dst(&d), start(RuntimePerf::Clock::now()) {}
    ~PerfScope() {
        *dst += std::chrono::duration<double>(RuntimePerf::Clock::now() - start).count();
    }
};

struct PerfExclusiveScope {
    double* dst;
    double* nested;
    double nestedStart;
    RuntimePerf::Clock::time_point start;
    PerfExclusiveScope(double& d, double& n)
        : dst(&d), nested(&n), nestedStart(n), start(RuntimePerf::Clock::now()) {}
    ~PerfExclusiveScope() {
        const double total = std::chrono::duration<double>(
            RuntimePerf::Clock::now() - start).count();
        const double nestedDelta = *nested - nestedStart;
        *dst += std::max(0.0, total - nestedDelta);
    }
};

inline void perfReset() {
    g_perf = RuntimePerf{};
    g_perf.totalStart = RuntimePerf::Clock::now();
    g_perf.totalActive = true;
}

inline void perfAbsorbSearch(const dpbridge::SolveOutcome& o) {
    g_perf.statesExpanded += o.statesExpanded;
    g_perf.dedupHits += o.dedupHits;
    if (o.capHits > 0) g_perf.capHits += o.capHits;
    g_perf.statesDroppedByCap += o.statesDroppedByCap;
    g_perf.peakAliveStates = std::max(g_perf.peakAliveStates, o.peakAliveStates);
    g_perf.peakMemoryBytes = std::max(g_perf.peakMemoryBytes, o.peakMemoryBytes);
    g_perf.candidateReplaySeconds += o.candidateReplaySeconds;
    g_perf.rejoinSeconds += o.rejoinSeconds;
    g_perf.totalSimulatedPhysicsTicks += o.searchSimulatedPhysicsTicks
                                      + o.candidateReplayTicks;
    if (o.rejoinT >= 0) ++g_perf.rejoins;
}

inline void perfBeginVerification() {
    g_perf.verifyStart = RuntimePerf::Clock::now();
    g_perf.verifyActive = true;
    ++g_perf.verificationAttempts;
}

inline void perfEndVerification(long long physicsTicks) {
    if (!g_perf.verifyActive) return;
    g_perf.realGdVerificationSeconds += std::chrono::duration<double>(
        RuntimePerf::Clock::now() - g_perf.verifyStart).count();
    g_perf.totalSimulatedPhysicsTicks += std::max<long long>(0, physicsTicks);
    g_perf.verifyActive = false;
}

inline void perfFinalPrepBegin() {
    g_perf.finalPrepStart = RuntimePerf::Clock::now();
    g_perf.finalPrepActive = true;
}

inline void perfFinalPrepEnd() {
    if (!g_perf.finalPrepActive) return;
    g_perf.finalReplayPrepSeconds += std::chrono::duration<double>(
        RuntimePerf::Clock::now() - g_perf.finalPrepStart).count();
    g_perf.finalPrepActive = false;
}

inline void perfReport(const char* result) {
    if (g_perf.reported || !g_perf.totalActive) return;
    if (g_perf.finalPrepActive) perfFinalPrepEnd();
    const double total = std::chrono::duration<double>(
        RuntimePerf::Clock::now() - g_perf.totalStart).count();
    const double exclusiveSearch = std::max(
        1e-9,
        g_perf.initialSearchSeconds + g_perf.suffixSearchSeconds
            - g_perf.candidateReplaySeconds);
    const double statesPerSecond =
        static_cast<double>(g_perf.statesExpanded) / exclusiveSearch;

    char a[1024];
    std::snprintf(
        a, sizeof(a),
        "SOLVE_PROFILE result=%s MODEL_BUILD=%.6f INITIAL_SEARCH=%.6f "
        "CANDIDATE_REPLAY=%.6f REAL_GD_VERIFICATION=%.6f "
        "DEATH_DIVERGENCE_DETECTION=%.6f RECORD_FIXUPS=%.6f "
        "FIXUP_RESIMULATION=%.6f FIXUP_PASS=%.6f REANCHOR=%.6f "
        "SUFFIX_SEARCH=%.6f REJOIN=%.6f FINAL_REPLAY_PREP=%.6f TOTAL_SOLVE=%.6f",
        result,
        g_perf.modelBuildSeconds,
        g_perf.initialSearchSeconds,
        g_perf.candidateReplaySeconds,
        g_perf.realGdVerificationSeconds,
        g_perf.deathDivergenceSeconds,
        g_perf.recordFixupsSeconds,
        g_perf.fixupResimulationSeconds,
        g_perf.fixupPassSeconds,
        g_perf.reanchorSeconds,
        g_perf.suffixSearchSeconds,
        g_perf.rejoinSeconds,
        g_perf.finalReplayPrepSeconds,
        total);
    writeResult(a);

    char b[1024];
    std::snprintf(
        b, sizeof(b),
        "SOLVE_COUNTERS statesExpanded=%lld statesPerSecond=%.3f dedupHits=%lld "
        "capHits=%lld statesDroppedByCap=%lld peakAliveStates=%lld "
        "peakMemoryBytes=%llu peakMemoryScope=solver_estimate "
        "verificationAttempts=%lld repairCount=%lld reanchors=%lld rejoins=%lld "
        "totalSimulatedPhysicsTicks=%lld wastedTicksAvoided=%lld",
        g_perf.statesExpanded,
        statesPerSecond,
        g_perf.dedupHits,
        g_perf.capHits,
        g_perf.statesDroppedByCap,
        g_perf.peakAliveStates,
        g_perf.peakMemoryBytes,
        g_perf.verificationAttempts,
        g_perf.repairCount,
        g_perf.reanchors,
        g_perf.rejoins,
        g_perf.totalSimulatedPhysicsTicks,
        g_perf.wastedTicksAvoided);
    writeResult(b);
    g_perf.reported = true;
}
]=])
string(FIND "${_autobot_repair_text}" "${_autobot_perf_insert_old}" _autobot_perf_insert_pos)
if(_autobot_perf_insert_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver runtime perf insertion anchor not found")
endif()
string(REPLACE "${_autobot_perf_insert_old}" "${_autobot_perf_insert_new}"
    _autobot_repair_text "${_autobot_repair_text}")

set(_autobot_fixup_entry_old [=[
inline int fixupPass(long long t0, const std::string& startArgStr, const std::string& band,
                     long long deathTick) {
]=])
set(_autobot_fixup_entry_new [=[
inline int fixupPass(long long t0, const std::string& startArgStr, const std::string& band,
                     long long deathTick) {
    PerfExclusiveScope perfFixupPass(g_perf.fixupPassSeconds,
                                     g_perf.fixupResimulationSeconds);
]=])
string(FIND "${_autobot_repair_text}" "${_autobot_fixup_entry_old}" _autobot_fixup_entry_pos)
if(_autobot_fixup_entry_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver fixup timing entry anchor not found")
endif()
string(REPLACE "${_autobot_fixup_entry_old}" "${_autobot_fixup_entry_new}"
    _autobot_repair_text "${_autobot_repair_text}")

set(_autobot_fixup_solve_old [=[
    logSolverArgs(a);
    dpbridge::solveInProcess(g_csv, a);
    if (rotQ) logRotSeedRead(t0, "resim");
    const long long modelDied = dpbridge::outcome().replayDiedT;
]=])
set(_autobot_fixup_solve_new [=[
    logSolverArgs(a);
    const auto perfResim0 = RuntimePerf::Clock::now();
    dpbridge::solveInProcess(g_csv, a);
    g_perf.fixupResimulationSeconds += std::chrono::duration<double>(
        RuntimePerf::Clock::now() - perfResim0).count();
    const dpbridge::SolveOutcome perfReplayOutcome = dpbridge::outcome();
    g_perf.totalSimulatedPhysicsTicks += perfReplayOutcome.replayTicks;
    g_perf.wastedTicksAvoided += std::max<long long>(
        0, perfReplayOutcome.replayPotentialTicks - perfReplayOutcome.replayTicks);
    if (rotQ) logRotSeedRead(t0, "resim");
    const long long modelDied = perfReplayOutcome.replayDiedT;
]=])
string(FIND "${_autobot_repair_text}" "${_autobot_fixup_solve_old}" _autobot_fixup_solve_pos)
if(_autobot_fixup_solve_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver fixup resim timing anchor not found")
endif()
string(REPLACE "${_autobot_fixup_solve_old}" "${_autobot_fixup_solve_new}"
    _autobot_repair_text "${_autobot_repair_text}")

set(_autobot_record_fixups_old [=[
inline void recordFixups(long long deathTick) {
    if (!g_cfg.dpFixups || deathTick < 2 || g_fixupCount >= kFixupCap) return;
]=])
set(_autobot_record_fixups_new [=[
inline void recordFixups(long long deathTick) {
    if (!g_cfg.dpFixups || deathTick < 2 || g_fixupCount >= kFixupCap) return;
    PerfScope perfRecordFixups(g_perf.recordFixupsSeconds);
    ++g_perf.repairCount;
]=])
string(FIND "${_autobot_repair_text}" "${_autobot_record_fixups_old}" _autobot_record_fixups_pos)
if(_autobot_record_fixups_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver recordFixups timing anchor not found")
endif()
string(REPLACE "${_autobot_record_fixups_old}" "${_autobot_record_fixups_new}"
    _autobot_repair_text "${_autobot_repair_text}")

set(_autobot_reanchor_old "        const int held = heldBefore(g_plan, t0);")
set(_autobot_reanchor_new [=[
        const auto perfReanchor0 = RuntimePerf::Clock::now();
        const int held = heldBefore(g_plan, t0);
]=])
string(FIND "${_autobot_repair_text}" "${_autobot_reanchor_old}" _autobot_reanchor_pos)
if(_autobot_reanchor_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver reanchor timing anchor not found")
endif()
string(REPLACE "${_autobot_reanchor_old}" "${_autobot_reanchor_new}"
    _autobot_repair_text "${_autobot_repair_text}")

set(_autobot_suffix_solve_old [=[
        logSolverArgs(a);
        const int rc = dpbridge::solveInProcess(g_csv, a);
        if (rotQ) logRotSeedRead(t0, "anchor");
]=])
set(_autobot_suffix_solve_new [=[
        logSolverArgs(a);
        g_perf.reanchorSeconds += std::chrono::duration<double>(
            RuntimePerf::Clock::now() - perfReanchor0).count();
        ++g_perf.reanchors;
        const auto perfSuffix0 = RuntimePerf::Clock::now();
        const int rc = dpbridge::solveInProcess(g_csv, a);
        g_perf.suffixSearchSeconds += std::chrono::duration<double>(
            RuntimePerf::Clock::now() - perfSuffix0).count();
        if (rotQ) logRotSeedRead(t0, "anchor");
]=])
string(FIND "${_autobot_repair_text}" "${_autobot_suffix_solve_old}" _autobot_suffix_solve_pos)
if(_autobot_suffix_solve_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver suffix search timing anchor not found")
endif()
string(REPLACE "${_autobot_suffix_solve_old}" "${_autobot_suffix_solve_new}"
    _autobot_repair_text "${_autobot_repair_text}")

set(_autobot_suffix_outcome_old [=[
        const dpbridge::SolveOutcome o = dpbridge::outcome();
]=])
set(_autobot_suffix_outcome_new [=[
        const dpbridge::SolveOutcome o = dpbridge::outcome();
        perfAbsorbSearch(o);
]=])
string(FIND "${_autobot_repair_text}" "${_autobot_suffix_outcome_old}" _autobot_suffix_outcome_pos)
if(_autobot_suffix_outcome_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver suffix outcome timing anchor not found")
endif()
string(REPLACE "${_autobot_suffix_outcome_old}" "${_autobot_suffix_outcome_new}"
    _autobot_repair_text "${_autobot_repair_text}")

set(_autobot_initial_solve_old [=[
                    g_rc = dpbridge::solveInProcess(g_csv, a0);
                    if (dpbridge::outcome().verdict != dpbridge::OutcomeCancelled) break;
]=])
set(_autobot_initial_solve_new [=[
                    const auto perfInitial0 = RuntimePerf::Clock::now();
                    g_rc = dpbridge::solveInProcess(g_csv, a0);
                    g_perf.initialSearchSeconds += std::chrono::duration<double>(
                        RuntimePerf::Clock::now() - perfInitial0).count();
                    const dpbridge::SolveOutcome perfInitialOutcome = dpbridge::outcome();
                    perfAbsorbSearch(perfInitialOutcome);
                    if (perfInitialOutcome.verdict != dpbridge::OutcomeCancelled) break;
]=])
string(FIND "${_autobot_repair_text}" "${_autobot_initial_solve_old}" _autobot_initial_solve_pos)
if(_autobot_initial_solve_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver initial search timing anchor not found")
endif()
string(REPLACE "${_autobot_initial_solve_old}" "${_autobot_initial_solve_new}"
    _autobot_repair_text "${_autobot_repair_text}")

set(_autobot_model_start_old [=[
    g_pendingLayer = nullptr;
    std::ostringstream oss;
]=])
set(_autobot_model_start_new [=[
    g_pendingLayer = nullptr;
    perfReset();
    const auto perfModelBuild0 = RuntimePerf::Clock::now();
    std::ostringstream oss;
]=])
string(FIND "${_autobot_repair_text}" "${_autobot_model_start_old}" _autobot_model_start_pos)
if(_autobot_model_start_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver model-build timing start anchor not found")
endif()
string(REPLACE "${_autobot_model_start_old}" "${_autobot_model_start_new}"
    _autobot_repair_text "${_autobot_repair_text}")

set(_autobot_model_end_old [=[
    rotSeedLevelReset();       // ...and cfg dprotseed's queue order and witnesses
]=])
set(_autobot_model_end_new [=[
    rotSeedLevelReset();       // ...and cfg dprotseed's queue order and witnesses
    g_perf.modelBuildSeconds += std::chrono::duration<double>(
        RuntimePerf::Clock::now() - perfModelBuild0).count();
]=])
string(FIND "${_autobot_repair_text}" "${_autobot_model_end_old}" _autobot_model_end_pos)
if(_autobot_model_end_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver model-build timing end anchor not found")
endif()
string(REPLACE "${_autobot_model_end_old}" "${_autobot_model_end_new}"
    _autobot_repair_text "${_autobot_repair_text}")

set(_autobot_death_start_old [=[
    // A VOID attempt: a death in the first moments of a run that recorded (nearly)
]=])
set(_autobot_death_start_new [=[
    perfEndVerification(dt);
    PerfScope perfDeathDivergence(g_perf.deathDivergenceSeconds);
    // A VOID attempt: a death in the first moments of a run that recorded (nearly)
]=])
string(FIND "${_autobot_repair_text}" "${_autobot_death_start_old}" _autobot_death_start_pos)
if(_autobot_death_start_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver death timing anchor not found")
endif()
string(REPLACE "${_autobot_death_start_old}" "${_autobot_death_start_new}"
    _autobot_repair_text "${_autobot_repair_text}")

set(_autobot_verify_begin_old [=[
    g_cfg.inputs = g_plan;
    g_paused = false;
    // The screen stays off here. These replays are the loop TESTING a candidate, not showing a
]=])
set(_autobot_verify_begin_new [=[
    g_cfg.inputs = g_plan;
    g_paused = false;
    perfBeginVerification();
    // The screen stays off here. These replays are the loop TESTING a candidate, not showing a
]=])
string(FIND "${_autobot_repair_text}" "${_autobot_verify_begin_old}" _autobot_verify_begin_pos)
if(_autobot_verify_begin_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver verification timing anchor not found")
endif()
string(REPLACE "${_autobot_verify_begin_old}" "${_autobot_verify_begin_new}"
    _autobot_repair_text "${_autobot_repair_text}")

set(_autobot_giveup_old [=[
    g_paused = true;
    endSession(sessionWhy);
]=])
set(_autobot_giveup_new [=[
    g_paused = true;
    perfReport("INCOMPLETE");
    endSession(sessionWhy);
]=])
string(FIND "${_autobot_repair_text}" "${_autobot_giveup_old}" _autobot_giveup_pos)
if(_autobot_giveup_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver giveUp profile anchor not found")
endif()
string(REPLACE "${_autobot_giveup_old}" "${_autobot_giveup_new}"
    _autobot_repair_text "${_autobot_repair_text}")

file(WRITE "${_autobot_repair}" "${_autobot_repair_text}")

set(_autobot_play_hooks "${AUTOBOT_GDSOLVER_ROOT}/src/mod/hooks_playlayer.cpp")
file(READ "${_autobot_play_hooks}" _autobot_play_hooks_text)
set(_autobot_final_prep_start_old [=[
            if (g_cfg.dpSolve && !g_dpShowSolution && !g_cfg.inputs.empty()) {
]=])
set(_autobot_final_prep_start_new [=[
            if (g_cfg.dpSolve && !g_dpShowSolution)
                dpsolve::perfEndVerification(g_tick);
            if (g_cfg.dpSolve && !g_dpShowSolution)
                dpsolve::perfFinalPrepBegin();
            if (g_cfg.dpSolve && !g_dpShowSolution && !g_cfg.inputs.empty()) {
]=])
string(FIND "${_autobot_play_hooks_text}" "${_autobot_final_prep_start_old}" _autobot_final_prep_start_pos)
if(_autobot_final_prep_start_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver final prep timing start anchor not found")
endif()
string(REPLACE "${_autobot_final_prep_start_old}" "${_autobot_final_prep_start_new}"
    _autobot_play_hooks_text "${_autobot_play_hooks_text}")

set(_autobot_final_prep_end_old [=[
            if (g_cfg.dpSolve && dpsolve::onCleared()) return;
]=])
set(_autobot_final_prep_end_new [=[
            if (g_cfg.dpSolve && !g_dpShowSolution) {
                dpsolve::perfFinalPrepEnd();
                dpsolve::perfReport("SOLVED");
            }
            if (g_cfg.dpSolve && dpsolve::onCleared()) return;
]=])
string(FIND "${_autobot_play_hooks_text}" "${_autobot_final_prep_end_old}" _autobot_final_prep_end_pos)
if(_autobot_final_prep_end_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver final prep timing end anchor not found")
endif()
string(REPLACE "${_autobot_final_prep_end_old}" "${_autobot_final_prep_end_new}"
    _autobot_play_hooks_text "${_autobot_play_hooks_text}")

set(_autobot_exit_profile_old [=[
                log::info("session ended by level exit");
                endSession("level_exit");
]=])
set(_autobot_exit_profile_new [=[
                log::info("session ended by level exit");
                if (g_cfg.dpSolve && !g_dpShowSolution)
                    dpsolve::perfReport("INCOMPLETE");
                endSession("level_exit");
]=])
string(FIND "${_autobot_play_hooks_text}" "${_autobot_exit_profile_old}" _autobot_exit_profile_pos)
if(_autobot_exit_profile_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver level-exit profile anchor not found")
endif()
string(REPLACE "${_autobot_exit_profile_old}" "${_autobot_exit_profile_new}"
    _autobot_play_hooks_text "${_autobot_play_hooks_text}")

set(_autobot_maxattempt_profile_old [=[
        if (g_started && g_finishedAttempts >= g_cfg.maxAttempts) {
            endSession("max_attempts");
        }
]=])
set(_autobot_maxattempt_profile_new [=[
        if (g_started && g_finishedAttempts >= g_cfg.maxAttempts) {
            if (g_cfg.dpSolve && !g_dpShowSolution)
                dpsolve::perfReport("INCOMPLETE");
            endSession("max_attempts");
        }
]=])
string(FIND "${_autobot_play_hooks_text}" "${_autobot_maxattempt_profile_old}" _autobot_maxattempt_profile_pos)
if(_autobot_maxattempt_profile_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver max-attempts profile anchor not found")
endif()
string(REPLACE "${_autobot_maxattempt_profile_old}" "${_autobot_maxattempt_profile_new}"
    _autobot_play_hooks_text "${_autobot_play_hooks_text}")


# Public/custom level lifecycle stability:
# The pinned source cached the GJGameLevel* passed into PlayLayer::init as a
# process-global raw pointer before the original init and never cleared it in
# resetSessionState(). Main levels are manager-owned long-lived objects, which
# masked this. Saved/online/editor levels do not grant that lifetime guarantee.
# Retain the level while progress suppression may dereference it, clear it on
# session reset, and after the original PlayLayer init rebind to the layer's
# actual m_level object.
set(_autobot_config "\${AUTOBOT_GDSOLVER_ROOT}/src/mod/config.hpp")
file(READ "\${_autobot_config}" _autobot_config_text)
set(_autobot_progress_owner_old [=[
inline LevelProgress g_progressAtStart;
inline GJGameLevel* g_progressLevel = nullptr;
]=])
set(_autobot_progress_owner_new [=[
inline LevelProgress g_progressAtStart;
inline GJGameLevel* g_progressLevel = nullptr;

// g_progressLevel is dereferenced after resets and at session teardown. Own one
// explicit retain while it is published so saved/online/editor level lifetime
// cannot depend on whichever menu/cache object launched PlayLayer.
inline void bindProgressLevel(GJGameLevel* level) {
    if (level == g_progressLevel) return;
    GJGameLevel* old = g_progressLevel;
    if (level) level->retain();
    g_progressLevel = level;
    log::info("PROGRESS_LEVEL_BIND old={} new={}",
              static_cast<void*>(old), static_cast<void*>(level));
    if (old) old->release();
}

inline void clearProgressLevel() {
    GJGameLevel* old = g_progressLevel;
    g_progressLevel = nullptr;
    g_progressAtStart = LevelProgress{};
    log::info("PROGRESS_LEVEL_CLEAR old={}", static_cast<void*>(old));
    if (old) old->release();
}

inline bool isPublicCustomLevel(GJGameLevel* level) {
    if (!level) return false;
    const int id = level->m_levelID.value();
    return level->m_levelType != GJLevelType::Main || id < 1 || id > 22;
}
]=])
string(FIND "\${_autobot_config_text}" "\${_autobot_progress_owner_old}" _autobot_progress_owner_pos)
if(_autobot_progress_owner_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver progress-level ownership anchor not found")
endif()
string(REPLACE "\${_autobot_progress_owner_old}" "\${_autobot_progress_owner_new}"
    _autobot_config_text "\${_autobot_config_text}")
file(WRITE "\${_autobot_config}" "\${_autobot_config_text}")

set(_autobot_plan_io "\${AUTOBOT_GDSOLVER_ROOT}/src/mod/plan_io.hpp")
file(READ "\${_autobot_plan_io}" _autobot_plan_io_text)
set(_autobot_progress_clear_old [=[
    g_blockedAch = 0; g_blockedStat = 0; g_blockedCoin = 0; g_blockedProgress = 0;
    g_progressRestores = 0;
]=])
set(_autobot_progress_clear_new [=[
    g_blockedAch = 0; g_blockedStat = 0; g_blockedCoin = 0; g_blockedProgress = 0;
    // Release the retained level before a new scene/session can reuse this
    // process. This was missing while other level-owned raw pointers already
    // had explicit reset paths.
    clearProgressLevel();
    g_progressRestores = 0;
]=])
string(FIND "\${_autobot_plan_io_text}" "\${_autobot_progress_clear_old}" _autobot_progress_clear_pos)
if(_autobot_progress_clear_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver progress-level cleanup anchor not found")
endif()
string(REPLACE "\${_autobot_progress_clear_old}" "\${_autobot_progress_clear_new}"
    _autobot_plan_io_text "\${_autobot_plan_io_text}")
file(WRITE "\${_autobot_plan_io}" "\${_autobot_plan_io_text}")

set(_autobot_session "\${AUTOBOT_GDSOLVER_ROOT}/src/mod/session.hpp")
file(READ "\${_autobot_session}" _autobot_session_text)
set(_autobot_session_config_begin_old [=[
inline bool uiConfigureSession(int levelId) {
    if (g_uiMode == UI_MODE_SOLVE) {
]=])
set(_autobot_session_config_begin_new [=[
inline bool uiConfigureSession(int levelId) {
    log::info("SESSION_CONFIG_BEGIN levelID={} uiMode={}", levelId, g_uiMode);
    if (g_uiMode == UI_MODE_SOLVE) {
]=])
string(FIND "\${_autobot_session_text}" "\${_autobot_session_config_begin_old}" _autobot_session_config_begin_pos)
if(_autobot_session_config_begin_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver session-config begin anchor not found")
endif()
string(REPLACE "\${_autobot_session_config_begin_old}" "\${_autobot_session_config_begin_new}"
    _autobot_session_text "\${_autobot_session_text}")

set(_autobot_session_config_end_old [=[
    solver::g_solveStart = std::chrono::steady_clock::now();
    return true;
}
]=])
set(_autobot_session_config_end_new [=[
    solver::g_solveStart = std::chrono::steady_clock::now();
    writeResult("SESSION_CONFIG_END levelID=" + std::to_string(levelId)
                + " started=1");
    log::info("SESSION_CONFIG_END levelID={} started=1", levelId);
    return true;
}
]=])
string(FIND "\${_autobot_session_text}" "\${_autobot_session_config_end_old}" _autobot_session_config_end_pos)
if(_autobot_session_config_end_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver session-config end anchor not found")
endif()
string(REPLACE "\${_autobot_session_config_end_old}" "\${_autobot_session_config_end_new}"
    _autobot_session_text "\${_autobot_session_text}")

set(_autobot_session_end_old [=[
inline void endSession(const std::string& why) {
    if (g_sessionOver) return;
]=])
set(_autobot_session_end_new [=[
inline void endSession(const std::string& why) {
    if (g_sessionOver) return;
    log::info("SESSION_END reason={} progressLevel={}",
              why, static_cast<void*>(g_progressLevel));
    writeResult("SESSION_END reason=" + why);
]=])
string(FIND "\${_autobot_session_text}" "\${_autobot_session_end_old}" _autobot_session_end_pos)
if(_autobot_session_end_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver session-end checkpoint anchor not found")
endif()
string(REPLACE "\${_autobot_session_end_old}" "\${_autobot_session_end_new}"
    _autobot_session_text "\${_autobot_session_text}")
file(WRITE "\${_autobot_session}" "\${_autobot_session_text}")

set(_autobot_play_init_old [=[
    bool init(GJGameLevel* level, bool useReplay, bool dontCreateObjects) {
        // Depending on the panel mode, auto-configure a session whichever level is entered.
        // Level selection can use the game's own UI as is (custom levels included).
        // A session in progress (g_started && !g_sessionOver) (including autorun.cfg
        // launches) is not interfered with. Leftovers of a finished session count as absent
        // (uiConfigureSession resets everything)
        bool want = (!g_started || g_sessionOver) && g_uiMode > 0 && level;
        if (want) uiConfigureSession(level->m_levelID.value());
        // Sample the level's own record before it has run a tick, so the session-end line can
        // state whether anything was written into it (see progressDiff).
        g_progressLevel = level;
        g_progressAtStart = sampleProgress(level);
        return PlayLayer::init(level, useReplay, dontCreateObjects);
    }
]=])
set(_autobot_play_init_new [=[
    bool init(GJGameLevel* level, bool useReplay, bool dontCreateObjects) {
        const bool custom = isPublicCustomLevel(level);
        if (custom && level) {
            log::info(
                "CUSTOM_LEVEL_INIT_ENTER ptr={} levelID={} levelType={} "
                "levelVersion={} gameVersion={} objectCount={} levelStringSize={}",
                static_cast<void*>(level), level->m_levelID.value(),
                static_cast<int>(level->m_levelType), level->m_levelVersion,
                level->m_gameVersion, level->m_objectCount,
                level->m_levelString.size());
        }
        log::info("PLAYLAYER_INIT_BEFORE_ORIGINAL level={} custom={}",
                  static_cast<void*>(level), custom ? 1 : 0);

        // Depending on the panel mode, auto-configure a session whichever level is entered.
        // uiConfigureSession resets old session state, including the previous retained level,
        // so ownership of this level is established only after that reset.
        bool want = (!g_started || g_sessionOver) && g_uiMode > 0 && level;
        if (want) uiConfigureSession(level->m_levelID.value());

        bindProgressLevel(level);
        g_progressAtStart = sampleProgress(level);
        if (custom && level && g_started) {
            writeResult(fmt::format(
                "CUSTOM_LEVEL_INIT_ENTER ptr={} levelID={} levelType={} "
                "levelVersion={} gameVersion={} objectCount={} levelStringSize={}",
                static_cast<void*>(level), level->m_levelID.value(),
                static_cast<int>(level->m_levelType), level->m_levelVersion,
                level->m_gameVersion, level->m_objectCount,
                level->m_levelString.size()));
        }
        if (g_started) writeResult("PLAYLAYER_INIT_BEFORE_ORIGINAL");

        const bool ok = PlayLayer::init(level, useReplay, dontCreateObjects);
        log::info("PLAYLAYER_INIT_AFTER_ORIGINAL ok={} inputLevel={} layerLevel={}",
                  ok ? 1 : 0, static_cast<void*>(level),
                  static_cast<void*>(this->m_level));
        if (g_started) writeResult(std::string("PLAYLAYER_INIT_AFTER_ORIGINAL ok=")
                                   + (ok ? "1" : "0"));

        if (!ok) {
            clearProgressLevel();
            return false;
        }

        // Bind to the actual level retained/used by PlayLayer after GD has finished init.
        // Some custom launch flows do not have the same ownership as getMainLevel().
        GJGameLevel* liveLevel = this->m_level ? this->m_level : level;
        bindProgressLevel(liveLevel);
        if (custom)
            log::info("CUSTOM_LEVEL_INIT_BOUND input={} live={}",
                      static_cast<void*>(level), static_cast<void*>(liveLevel));
        return true;
    }
]=])
string(FIND "\${_autobot_play_hooks_text}" "\${_autobot_play_init_old}" _autobot_play_init_pos)
if(_autobot_play_init_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver PlayLayer::init lifecycle anchor not found")
endif()
string(REPLACE "\${_autobot_play_init_old}" "\${_autobot_play_init_new}"
    _autobot_play_hooks_text "\${_autobot_play_hooks_text}")

set(_autobot_quit_checkpoint_old [=[
    void onQuit() {
        // Exiting from the pause menu = session aborted. endSession alone is not enough:
]=])
set(_autobot_quit_checkpoint_new [=[
    void onQuit() {
        log::info("LEVEL_QUIT this={} level={} started={} over={}",
                  static_cast<void*>(this), static_cast<void*>(this->m_level),
                  g_started ? 1 : 0, g_sessionOver ? 1 : 0);
        if (g_started) writeResult("LEVEL_QUIT");
        // Exiting from the pause menu = session aborted. endSession alone is not enough:
]=])
string(FIND "\${_autobot_play_hooks_text}" "\${_autobot_quit_checkpoint_old}" _autobot_quit_checkpoint_pos)
if(_autobot_quit_checkpoint_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver level-quit checkpoint anchor not found")
endif()
string(REPLACE "\${_autobot_quit_checkpoint_old}" "\${_autobot_quit_checkpoint_new}"
    _autobot_play_hooks_text "\${_autobot_play_hooks_text}")

set(_autobot_reset_checkpoint_old [=[
    void resetLevel() {
        // A reset of an old layer (embers awaiting the scene swap) must not touch the books
]=])
set(_autobot_reset_checkpoint_new [=[
    void resetLevel() {
        log::info("LEVEL_RESET this={} live={} tick={}",
                  static_cast<void*>(this), static_cast<void*>(PlayLayer::get()),
                  (long long)g_tick);
        if (g_started) writeResult("LEVEL_RESET tick=" + std::to_string((long long)g_tick));
        // A reset of an old layer (embers awaiting the scene swap) must not touch the books
]=])
string(FIND "\${_autobot_play_hooks_text}" "\${_autobot_reset_checkpoint_old}" _autobot_reset_checkpoint_pos)
if(_autobot_reset_checkpoint_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver level-reset checkpoint anchor not found")
endif()
string(REPLACE "\${_autobot_reset_checkpoint_old}" "\${_autobot_reset_checkpoint_new}"
    _autobot_play_hooks_text "\${_autobot_play_hooks_text}")

file(WRITE "\${_autobot_play_hooks}" "\${_autobot_play_hooks_text}")

# Solver-start checkpoints. No search behavior changes.
file(READ "\${_autobot_repair}" _autobot_repair_text)
set(_autobot_solver_start_begin_old [=[
inline void start(GJBaseGameLayer* l) {
    if (!l) return;
    if (refuseIfPlatformer(l)) return;
]=])
set(_autobot_solver_start_begin_new [=[
inline void start(GJBaseGameLayer* l) {
    if (!l) return;
    log::info("SOLVER_START_BEGIN layer={} playLayer={} levelID={}",
              static_cast<void*>(l), static_cast<void*>(PlayLayer::get()),
              g_cfg.levelId);
    writeResult("SOLVER_START_BEGIN levelID=" + std::to_string(g_cfg.levelId));
    if (refuseIfPlatformer(l)) return;
]=])
string(FIND "\${_autobot_repair_text}" "\${_autobot_solver_start_begin_old}" _autobot_solver_start_begin_pos)
if(_autobot_solver_start_begin_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver solver-start begin anchor not found")
endif()
string(REPLACE "\${_autobot_solver_start_begin_old}" "\${_autobot_solver_start_begin_new}"
    _autobot_repair_text "\${_autobot_repair_text}")

set(_autobot_solver_start_end_old [=[
    if (!startBootstrapRecord())
        beginFirstAttempt();
}
]=])
set(_autobot_solver_start_end_new [=[
    const bool bootstrapStarted = startBootstrapRecord();
    if (!bootstrapStarted)
        beginFirstAttempt();
    writeResult("SOLVER_START_END csvBytes=" + std::to_string(g_csv.size())
                + " bootstrap=" + std::to_string(bootstrapStarted ? 1 : 0));
    log::info("SOLVER_START_END csvBytes={} bootstrap={}",
              g_csv.size(), bootstrapStarted ? 1 : 0);
}
]=])
string(FIND "\${_autobot_repair_text}" "\${_autobot_solver_start_end_old}" _autobot_solver_start_end_pos)
if(_autobot_solver_start_end_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver solver-start end anchor not found")
endif()
string(REPLACE "\${_autobot_solver_start_end_old}" "\${_autobot_solver_start_end_new}"
    _autobot_repair_text "\${_autobot_repair_text}")
file(WRITE "\${_autobot_repair}" "\${_autobot_repair_text}")

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

# Visible AutoBot branding. Legal MIT attribution remains in
# THIRD_PARTY_GDSOLVER_LICENSE.txt; this changes UI strings only.
set(_hud "${AUTOBOT_GDSOLVER_ROOT}/src/mod/hud.hpp")
file(READ "${_hud}" _hud_text)
foreach(_legacy_badge "GDSOLVER BOT - SOLVING" "GDSOLVER BOT - REPLAY")
    string(FIND "${_hud_text}" "${_legacy_badge}" _autobot_badge_pos)
    if(_autobot_badge_pos EQUAL -1)
        message(FATAL_ERROR "Pinned gdsolver HUD branding anchor not found: ${_legacy_badge}")
    endif()
    string(REPLACE "${_legacy_badge}" "AutoBotT7 Solver" _hud_text "${_hud_text}")
endforeach()
file(WRITE "${_hud}" "${_hud_text}")

set(_panel "${AUTOBOT_GDSOLVER_ROOT}/src/mod/ui_panel.hpp")
file(READ "${_panel}" _panel_text)
string(FIND "${_panel_text}" "\"GDSOLVER\"" _autobot_panel_brand_pos)
if(_autobot_panel_brand_pos EQUAL -1)
    message(FATAL_ERROR "Pinned gdsolver panel branding anchor not found")
endif()
string(REPLACE "\"GDSOLVER\"" "\"AutoBotT7\"" _panel_text "${_panel_text}")
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
