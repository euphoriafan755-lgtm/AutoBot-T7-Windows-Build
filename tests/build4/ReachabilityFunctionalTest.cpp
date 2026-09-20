#include "dp/cli.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
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

std::string generatedJumpLevel() {
    const std::vector<std::string> header{
        "id","type","cx","cy","w","h","groups","uid","radius","rot",
        "sy0","sy1","shz","sdir","sup","w0","h0","tpy","tpg","tpix",
        "tpiy","tw","zoom","zdur","zease","zrate","mvdir","gnddir",
        "optp1","optp2","flipx","flipy","nofx","notouch","tpex","tpey",
        "dis","editvel","vmodx","vmody","ovrvel","force","free","touch",
        "spawn","chan","axis","exstat"
    };

    std::vector<std::string> spike(header.size(), "0");
    spike[0] = "8";        // arbitrary object id; decision logic does not key on it
    spike[1] = "2";        // gameplay hazard
    spike[2] = "90";       // far enough away that a cold search must decide
    spike[3] = "105";      // floor-height spike
    spike[4] = "30";
    spike[5] = "30";
    spike[6] = "";         // no groups
    spike[7] = "1";        // unique id
    spike[8] = "0";        // rectangle, not saw radius
    spike[9] = "0";
    spike[15] = "30";
    spike[16] = "30";

    return join(header) + "\n" + join(spike) + "\n";
}

int runDp(std::vector<std::string> args, std::string const& csv) {
    dp::g_levelCsv = csv;
    std::vector<char*> argv;
    argv.reserve(args.size());
    for (auto& arg : args) argv.push_back(arg.data());
    return dp::cliMain(static_cast<int>(argv.size()), argv.data());
}

bool planContainsPress(std::string const& path) {
    std::ifstream in(path);
    std::string line;
    long long tick = 0;
    int level = 0;
    while (std::getline(in, line)) {
        if (std::sscanf(line.c_str(), "input=%lld,%d", &tick, &level) == 2
            && level == 1) {
            return true;
        }
    }
    return false;
}

} // namespace

int main() {
    const auto csv = generatedJumpLevel();

    std::remove("build4-generated-plan.txt");
    std::remove("build4-generated-plan.txt.trace.csv");

    const int coldRc = runDp(
        {"leveldp", "memory", "--out", "build4-generated-plan.txt", "--threads", "1"},
        csv
    );
    assert(coldRc == 0);
    assert(dp::g_outcome.verdict == dp::VerdictSolved);
    assert(planContainsPress("build4-generated-plan.txt"));

    std::cout
        << "REACHABILITY_BASIC_JUMP_TEST=PASS "
        << "no_press_path_blocked=YES press_solution=YES\n";
    std::cout
        << "COLD_SOLVE_GENERATED_LEVEL_TEST=PASS "
        << "source=geometry_only verdict=SOLVED\n";

    const std::vector<std::uint8_t> levels{0, 0, 1, 1, 1, 0, 0, 1, 1, 0};
    const std::vector<std::uint8_t> modes(levels.size(), 0);
    const auto edgesA = dp::planEdges(levels, modes, 100, 0, 0, false);
    const auto edgesB = dp::planEdges(levels, modes, 100, 0, 0, false);
    assert(edgesA.size() == edgesB.size());
    for (std::size_t i = 0; i < edgesA.size(); ++i) {
        assert(edgesA[i].press == edgesB[i].press);
        assert(edgesA[i].level == edgesB[i].level);
    }
    assert(edgesA.size() == 4);
    assert(edgesA[0].press == 102 && edgesA[0].level == 1);
    assert(edgesA[1].press == 105 && edgesA[1].level == 0);

    const int replayRc = runDp(
        {
            "leveldp", "memory",
            "--replay", "build4-generated-plan.txt",
            "--out", "build4-replay",
            "--threads", "1"
        },
        csv
    );
    assert(replayRc == 0);
    assert(dp::g_outcome.replayDiedT < 0);

    std::cout
        << "REPLAY_SAME_PLAN_TEST=PASS "
        << "physical_edges_identical=YES replay_survived=YES\n";

    {
        std::ofstream empty("build4-no-input.txt", std::ios::trunc);
    }
    const int failedReplayRc = runDp(
        {
            "leveldp", "memory",
            "--replay", "build4-no-input.txt",
            "--out", "build4-no-input-replay",
            "--threads", "1"
        },
        csv
    );
    assert(failedReplayRc == 0);
    const auto firstFailureTick = dp::g_outcome.replayDiedT;
    assert(firstFailureTick > 20);

    const long long anchorTick = firstFailureTick - 20;
    const double anchorX = static_cast<double>(anchorTick - 1) * dp::kDx;
    std::ostringstream anchor;
    anchor.precision(17);
    anchor << anchorTick << ',' << anchorX << ",105,0,0,1,0";

    const int suffixRc = runDp(
        {
            "leveldp", "memory",
            "--start", anchor.str(),
            "--out", "build4-reanchored-plan.txt",
            "--threads", "1"
        },
        csv
    );
    assert(suffixRc == 0);
    assert(dp::g_outcome.verdict == dp::VerdictSolved);
    assert(planContainsPress("build4-reanchored-plan.txt"));

    std::cout
        << "DIVERGENCE_REANCHOR_TEST=PASS "
        << "first_failure_tick=" << firstFailureTick
        << " anchor_tick=" << anchorTick
        << " suffix_solved=YES\n";

    return 0;
}
