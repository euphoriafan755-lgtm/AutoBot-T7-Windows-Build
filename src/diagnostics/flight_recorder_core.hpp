#pragma once
#include <cstdio>
#include <filesystem>
#include <mutex>
#include <string_view>
#ifdef _WIN32
#include <Windows.h>
#include <io.h>
#else
#include <unistd.h>
#endif

namespace autobot::diag {

inline bool appendCrashSafeLine(const std::filesystem::path& path, std::string_view line) {
    std::error_code ec;
    if (!path.parent_path().empty())
        std::filesystem::create_directories(path.parent_path(), ec);
    FILE* file = nullptr;
#ifdef _WIN32
    if (fopen_s(&file, path.string().c_str(), "ab") != 0 || !file) return false;
#else
    file = std::fopen(path.string().c_str(), "ab");
    if (!file) return false;
#endif
    bool ok = true;
    if (!line.empty() && std::fwrite(line.data(), 1, line.size(), file) != line.size()) ok = false;
    static constexpr char newline = '\n';
    if (std::fwrite(&newline, 1, 1, file) != 1) ok = false;
    if (std::fflush(file) != 0) ok = false;
#ifdef _WIN32
    if (_commit(_fileno(file)) != 0) ok = false;
#else
    if (::fsync(fileno(file)) != 0) ok = false;
#endif
    if (std::fclose(file) != 0) ok = false;
    return ok;
}

// Worker-only durable append: private path, no shared FILE*, explicit
// serialization, no Geode/Cocos APIs, and no exception can escape.
inline bool appendWorkerCrashSafeLine(const std::filesystem::path& path,
                                      std::string_view line) noexcept {
    try {
        static std::mutex workerWriteMutex;
        std::lock_guard<std::mutex> lock(workerWriteMutex);
#ifdef _WIN32
        HANDLE file = ::CreateFileW(
            path.c_str(), FILE_APPEND_DATA,
            FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
            nullptr, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (file == INVALID_HANDLE_VALUE) return false;

        auto writeAll = [file](const char* data, std::size_t size) noexcept {
            while (size > 0) {
                const DWORD chunk = size > static_cast<std::size_t>(0x7fffffffu)
                    ? 0x7fffffffu : static_cast<DWORD>(size);
                DWORD written = 0;
                if (!::WriteFile(file, data, chunk, &written, nullptr) || written == 0)
                    return false;
                data += written;
                size -= written;
            }
            return true;
        };

        bool ok = writeAll(line.data(), line.size());
        static constexpr char newline = '\n';
        if (!writeAll(&newline, 1)) ok = false;
        if (!::FlushFileBuffers(file)) ok = false;
        if (!::CloseHandle(file)) ok = false;
        return ok;
#else
        FILE* file = std::fopen(path.c_str(), "ab");
        if (!file) return false;
        bool ok = true;
        if (!line.empty() && std::fwrite(line.data(), 1, line.size(), file) != line.size()) ok = false;
        static constexpr char newline = '\n';
        if (std::fwrite(&newline, 1, 1, file) != 1) ok = false;
        if (std::fflush(file) != 0) ok = false;
        if (::fsync(fileno(file)) != 0) ok = false;
        if (std::fclose(file) != 0) ok = false;
        return ok;
#endif
    } catch (...) {
        return false;
    }
}

} // namespace autobot::diag
