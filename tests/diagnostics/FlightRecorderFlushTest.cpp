#include "diagnostics/flight_recorder_core.hpp"
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
int main(int argc, char** argv) {
    if (argc == 3 && std::string(argv[1]) == "--child") {
        const std::filesystem::path path(argv[2]);
        if (!autobot::diag::appendCrashSafeLine(path, "CHECKPOINT_BEFORE_ABRUPT_STOP")) return 3;
        std::_Exit(29);
    }
    const auto path = std::filesystem::current_path() / "flight-recorder-abrupt-stop-test.txt";
    std::error_code ec;
    std::filesystem::remove(path, ec);
    const std::string exe = std::filesystem::absolute(argv[0]).string();
    const std::string command = "\"" + exe + "\" --child \"" + path.string() + "\"";
    (void)std::system(command.c_str());
    std::ifstream in(path, std::ios::binary);
    const std::string contents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::filesystem::remove(path, ec);
    if (contents.find("CHECKPOINT_BEFORE_ABRUPT_STOP\n") == std::string::npos) {
        std::cerr << "checkpoint missing after abrupt child termination\n";
        return 4;
    }
    std::cout << "FLIGHT_RECORDER_FLUSH_TEST=PASS\n";
    return 0;
}
