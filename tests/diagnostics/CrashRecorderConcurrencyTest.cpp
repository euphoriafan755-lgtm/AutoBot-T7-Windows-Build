#include "diagnostics/flight_recorder_core.hpp"
#include <atomic>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

static std::vector<std::string> readLines(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    std::vector<std::string> out;
    std::string line;
    while (std::getline(in, line)) out.push_back(line);
    return out;
}

int main() {
    constexpr int kMainWrites = 200;
    constexpr int kWorkerThreads = 4;
    constexpr int kWorkerWrites = 200;
    const auto root = std::filesystem::current_path();
    const auto mainPath = root / "recorder-main-concurrency-test.txt";
    const auto workerPath = root / "recorder-worker-concurrency-test.txt";
    std::error_code ec;
    std::filesystem::remove(mainPath, ec);
    std::filesystem::remove(workerPath, ec);

    std::atomic<bool> go{false};
    std::atomic<int> failedAppends{0};
    std::atomic<int> exceptions{0};

    std::thread mainWriter([&] {
        try {
            while (!go.load(std::memory_order_acquire)) std::this_thread::yield();
            for (int i = 0; i < kMainWrites; ++i) {
                char line[64]{};
                std::snprintf(line, sizeof line, "MAIN seq=%d", i);
                if (!autobot::diag::appendCrashSafeLine(mainPath, line)) ++failedAppends;
            }
        } catch (...) { ++exceptions; }
    });

    std::vector<std::thread> workers;
    workers.reserve(kWorkerThreads);
    for (int w = 0; w < kWorkerThreads; ++w) {
        workers.emplace_back([&, w] {
            try {
                while (!go.load(std::memory_order_acquire)) std::this_thread::yield();
                for (int i = 0; i < kWorkerWrites; ++i) {
                    char line[96]{};
                    std::snprintf(line, sizeof line,
                        "WORKER id=%d seq=%d thread=WORKER", w, i);
                    if (!autobot::diag::appendWorkerCrashSafeLine(workerPath, line))
                        ++failedAppends;
                }
            } catch (...) { ++exceptions; }
        });
    }

    go.store(true, std::memory_order_release);
    mainWriter.join();
    for (auto& t : workers) t.join();

    const auto mainLines = readLines(mainPath);
    const auto workerLines = readLines(workerPath);
    std::unordered_set<std::string> expectedMain;
    for (int i = 0; i < kMainWrites; ++i)
        expectedMain.insert("MAIN seq=" + std::to_string(i));

    std::unordered_set<std::string> expectedWorker;
    for (int w = 0; w < kWorkerThreads; ++w)
        for (int i = 0; i < kWorkerWrites; ++i)
            expectedWorker.insert("WORKER id=" + std::to_string(w) +
                " seq=" + std::to_string(i) + " thread=WORKER");

    int mixed = 0, truncated = 0;
    for (const auto& line : mainLines) {
        if (line.find("WORKER") != std::string::npos) ++mixed;
        if (expectedMain.find(line) == expectedMain.end()) ++truncated;
    }
    for (const auto& line : workerLines) {
        if (line.find("MAIN") != std::string::npos) ++mixed;
        if (expectedWorker.find(line) == expectedWorker.end()) ++truncated;
    }
    const std::unordered_set<std::string> actualMain(mainLines.begin(), mainLines.end());
    const std::unordered_set<std::string> actualWorker(workerLines.begin(), workerLines.end());
    const bool countsOk =
        mainLines.size() == static_cast<std::size_t>(kMainWrites) &&
        workerLines.size() == static_cast<std::size_t>(kWorkerThreads * kWorkerWrites);
    const bool setsOk = actualMain == expectedMain && actualWorker == expectedWorker;

    std::filesystem::remove(mainPath, ec);
    std::filesystem::remove(workerPath, ec);

    if (failedAppends.load() || exceptions.load() || mixed || truncated || !countsOk || !setsOk) {
        std::cerr << "recorder concurrency failure failed=" << failedAppends.load()
                  << " exceptions=" << exceptions.load()
                  << " mixed=" << mixed << " truncated=" << truncated
                  << " mainLines=" << mainLines.size()
                  << " workerLines=" << workerLines.size() << "\n";
        return 2;
    }

    std::cout << "CRASH_RECORDER_MAIN_WORKER_CONCURRENCY_TEST=PASS "
                 "failed_appends=0 truncated=0 mixed=0 exceptions=0 deadlock=0\n";
    return 0;
}
