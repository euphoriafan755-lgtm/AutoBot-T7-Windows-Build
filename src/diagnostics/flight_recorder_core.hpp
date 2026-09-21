#pragma once
#include <cstdio>
#include <filesystem>
#include <string_view>
#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif
namespace autobot::diag {
inline bool appendCrashSafeLine(const std::filesystem::path& path, std::string_view line) {
    std::error_code ec;
    if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path(), ec);
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
}  // namespace autobot::diag
