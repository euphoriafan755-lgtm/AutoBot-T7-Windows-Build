#include "diagnostics/flight_recorder_core.hpp"
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#ifdef _WIN32
#include <process.h>
#endif
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
#ifdef _WIN32
    const std::string pathString = path.string();
    const char* childArgv[] = {exe.c_str(), "--child", pathString.c_str(), nullptr};
    (void)_spawnv(_P_WAIT, exe.c_str(), childArgv);
#else
    const std::string command = "\"" + exe + "\" --child \"" + path.string() + "\"";
    (void)std::system(command.c_str());
#endif
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
