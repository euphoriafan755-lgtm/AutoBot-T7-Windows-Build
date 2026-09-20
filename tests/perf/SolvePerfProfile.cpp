#include "dp/cli.hpp"

#include <cassert>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string join(std::vector<std::string> const& fields) {
    std::ostringstream out;
    for (std::size_t i = 0; i < fields.size(); ++i) {
        if (i) out << ',';
        out << fields[i];
    }
    return out.str();
}

std::string stressLevel() {
    const std::vector<std::string> header{
        "id","type","cx","cy","w","h","groups","uid","radius","rot",
        "sy0","sy1","shz","sdir","sup","w0","h0","tpy","tpg","tpix",
        "tpiy","tw","zoom","zdur","zease","zrate","mvdir","gnddir",
        "optp1","optp2","flipx","flipy","nofx","notouch","tpex","tpey",
        "dis","editvel","vmodx","vmody","ovrvel","force","free","touch",
        "spawn","chan","axis","exstat"
    };

    // One unreachable hazard sets a long goal without constraining the route.
    // The player starts as a ship, so both input levels remain meaningful and
    // the frontier quickly reaches the same 2,000-state cap used by the mod.
    std::vector<std::string> marker(header.size(), "0");
    marker[0] = "8";
    marker[1] = "2";
    marker[2] = "8000";
    marker[3] = "1000";
    marker[4] = "30";
    marker[5] = "30";
    marker[6] = "";
    marker[7] = "1";
    marker[15] = "30";
    marker[16] = "30";

    return join(header) + "\n" + join(marker) + "\n";
}

int runDp(std::vector<std::string> args, std::string const& csv) {
    dp::g_levelCsv = csv;
    std::vector<char*> argv;
    argv.reserve(args.size());
    for (auto& arg : args) argv.push_back(arg.data());
    return dp::cliMain(static_cast<int>(argv.size()), argv.data());
}

} // namespace

int main() {
    const std::string csv = stressLevel();
    std::remove("perf-baseline-plan.txt");
    std::remove("perf-baseline-plan.txt.trace.csv");

    const auto start = std::chrono::steady_clock::now();
    const int rc = runDp(
        {
            "leveldp", "memory",
            "--start", "0,-1.29825,300,0,1,0,0",
            "--out", "perf-baseline-plan.txt",
            "--horizon", "2500",
            "--cap", "2000",
            "--shipyq", "0.25",
            "--shipvq", "1.0",
            "--threads", "8",
            "--phaseprof"
        },
        csv
    );
    const double sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start
    ).count();

    assert(rc == 0);
    assert(dp::g_outcome.verdict == dp::VerdictSolved
           || dp::g_outcome.verdict == dp::VerdictPartial);

    std::ifstream plan("perf-baseline-plan.txt", std::ios::binary | std::ios::ate);
    assert(plan.good());
    const auto bytes = static_cast<long long>(plan.tellg());

    std::cout << std::fixed << std::setprecision(3)
              << "SOLVE_PERF_PROFILE_CURRENT=PASS"
              << " wall_s=" << sec
              << " verdict=" << dp::g_outcome.verdict
              << " cap_hits=" << dp::g_outcome.capHits
              << " plan_bytes=" << bytes
              << " threads=8 cap=2000 horizon=2500\n";
    return 0;
}
