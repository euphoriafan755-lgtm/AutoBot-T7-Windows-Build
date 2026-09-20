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
file(WRITE "${_autobot_play_hooks}" "${_autobot_play_hooks_text}")

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
