#pragma once

#include <Geode/utils/async.hpp>
#include <Geode/utils/web.hpp>

#include <string>

namespace autobot::update {

class UpdateManager final {
public:
    static UpdateManager& get();
    void checkOnce();

    [[nodiscard]] bool checked() const { return m_checked; }
    [[nodiscard]] bool updateAvailable() const { return m_updateAvailable; }
    [[nodiscard]] std::string const& status() const { return m_status; }

private:
    void handleLatest(geode::utils::web::WebResponse response);
    void download(std::string version, std::string url, std::string expectedHash);
    void handleDownload(
        std::string version,
        std::string expectedHash,
        geode::utils::web::WebResponse response
    );
    bool stageVerifiedPackage(
        std::string const& version,
        std::string const& expectedHash,
        geode::ByteVector const& bytes,
        std::string& error
    );

    bool m_checked = false;
    bool m_updateAvailable = false;
    std::string m_status = "NOT CHECKED";
    geode::async::TaskHolder<geode::utils::web::WebResponse> m_latestTask;
    geode::async::TaskHolder<geode::utils::web::WebResponse> m_downloadTask;
};

} // namespace autobot::update
