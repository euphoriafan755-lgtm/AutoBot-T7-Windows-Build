#include "autobot/update/UpdateManager.hpp"

#include "autobot/core/BuildInfo.hpp"

#include <Geode/Geode.hpp>
#include <Geode/loader/ModMetadata.hpp>
#include <Geode/utils/file.hpp>
#include <Geode/utils/general.hpp>
#include <Geode/utils/hash.hpp>

#include <filesystem>
#include <string>
#include <system_error>

using namespace geode::prelude;

namespace autobot::update {
namespace {

constexpr auto kLatestURL =
    "https://api.geode-sdk.org/v1/mods/thiago7tv.autobot-t7/versions/latest?gd=2.2081&platforms=win";

std::string jsonString(matjson::Value const& object, char const* key) {
    if (!object.isObject() || !object.contains(key) || !object[key].isString()) return {};
    return object[key].asString().unwrapOrDefault();
}

bool hashMatches(ByteVector const& bytes, std::string const& expectedHash) {
    return !expectedHash.empty() && geode::sha256(bytes).toString() == expectedHash;
}

void reject(std::string const& why) {
    log::error("UPDATE REJECTED: {}", why);
}

} // namespace

UpdateManager& UpdateManager::get() {
    static UpdateManager instance;
    return instance;
}

void UpdateManager::checkOnce() {
    if (m_checked) return;
    m_checked = true;
    m_status = "CHECKING";
    log::info(
        "UPDATE CHECK installed={} commit={} endpoint=GEODE_INDEX",
        core::build::version(),
        core::build::shortCommit()
    );

    m_latestTask.spawn(
        "AutoBot T7 update metadata",
        web::WebRequest{}.get(kLatestURL),
        [this](web::WebResponse response) { handleLatest(std::move(response)); }
    );
}

void UpdateManager::handleLatest(web::WebResponse response) {
    if (!response.ok()) {
        m_status = response.code() == 404 ? "UPDATE CHANNEL NOT PUBLISHED" : "UPDATE CHECK FAILED";
        log::warn("{} code={}", m_status, response.code());
        return;
    }

    auto parsed = response.json();
    if (!parsed) {
        m_status = "UPDATE MANIFEST INVALID";
        reject("Geode Index response was not valid JSON");
        return;
    }
    auto root = parsed.unwrap();
    if (!root.isObject() || !root.contains("payload") || !root["payload"].isObject()) {
        m_status = "UPDATE MANIFEST INVALID";
        reject("Geode Index payload missing");
        return;
    }
    auto payload = root["payload"];
    const auto versionString = jsonString(payload, "version");
    const auto downloadURL = jsonString(payload, "download_link");
    const auto expectedHash = jsonString(payload, "hash");
    const auto modID = jsonString(payload, "mod_id");
    if (versionString.empty() || downloadURL.empty() || expectedHash.empty()
        || modID != Mod::get()->getID()) {
        m_status = "UPDATE MANIFEST INVALID";
        reject("version/download/hash/mod_id validation failed");
        return;
    }

    auto parsedVersion = VersionInfo::parse(versionString);
    if (!parsedVersion) {
        m_status = "UPDATE MANIFEST INVALID";
        reject("latest version is not valid semver");
        return;
    }
    const auto latest = parsedVersion.unwrap();
    if (!(Mod::get()->getVersion() < latest)) {
        m_status = "UP TO DATE";
        log::info("UPDATE CHECK UP_TO_DATE installed={} latest={}", Mod::get()->getVersion(), latest);
        return;
    }

    m_updateAvailable = true;
    m_status = "UPDATE AVAILABLE";
    log::info("UPDATE AVAILABLE installed={} latest={}", Mod::get()->getVersion(), latest);
    download(versionString, downloadURL, expectedHash);
}

void UpdateManager::download(std::string version, std::string url, std::string expectedHash) {
    m_status = "DOWNLOADING VERIFIED UPDATE";
    m_downloadTask.spawn(
        "AutoBot T7 update package",
        web::WebRequest{}.get(std::move(url)),
        [this, version = std::move(version), expectedHash = std::move(expectedHash)](web::WebResponse response) mutable {
            handleDownload(std::move(version), std::move(expectedHash), std::move(response));
        }
    );
}

void UpdateManager::handleDownload(
    std::string version,
    std::string expectedHash,
    web::WebResponse response
) {
    if (!response.ok()) {
        m_status = "UPDATE DOWNLOAD FAILED";
        reject(fmt::format("download HTTP {}", response.code()));
        return;
    }

    auto const& bytes = response.data();
    const auto actualHash = geode::sha256(bytes).toString();
    if (!hashMatches(bytes, expectedHash)) {
        m_status = "UPDATE REJECTED";
        reject(fmt::format("SHA256 mismatch expected={} actual={}", expectedHash, actualHash));
        return;
    }

    std::string error;
    if (!stageVerifiedPackage(version, expectedHash, bytes, error)) {
        m_status = "UPDATE REJECTED";
        reject(error);
        return;
    }

    m_status = "UPDATE STAGED - RESTARTING";
    log::info("UPDATE VERIFIED version={} sha256={} replacement=ATOMIC_WITH_ROLLBACK", version, actualHash);
    utils::game::restart(true, false);
}

bool UpdateManager::stageVerifiedPackage(
    std::string const& version,
    std::string const& expectedHash,
    ByteVector const& bytes,
    std::string& error
) {
    auto* mod = Mod::get();
    const auto current = mod->getPackagePath();
    if (current.empty()) {
        error = "installed package path is empty";
        return false;
    }

    const auto dir = current.parent_path();
    const auto pending = dir / "thiago7tv.autobot-t7.pending.geode";
    const auto backup = dir / "thiago7tv.autobot-t7.backup.geode";
    std::error_code ec;
    std::filesystem::remove(pending, ec);
    ec.clear();
    std::filesystem::remove(backup, ec);
    ec.clear();

    auto write = file::writeBinary(pending, bytes);
    if (!write) {
        error = "failed to write pending package: " + write.unwrapErr();
        return false;
    }

    auto pendingBytes = file::readBinary(pending);
    if (!pendingBytes || !hashMatches(pendingBytes.unwrap(), expectedHash)) {
        std::filesystem::remove(pending, ec);
        error = "pending package hash verification failed";
        return false;
    }

    auto metadata = ModMetadata::createFromGeodeFile(pending);
    if (metadata.hasErrors()) {
        std::filesystem::remove(pending, ec);
        error = "pending .geode metadata invalid";
        return false;
    }
    if (metadata.getID() != mod->getID()) {
        std::filesystem::remove(pending, ec);
        error = "pending .geode mod ID mismatch";
        return false;
    }
    auto wanted = VersionInfo::parse(version);
    if (!wanted || metadata.getVersion() != wanted.unwrap()) {
        std::filesystem::remove(pending, ec);
        error = "pending .geode version mismatch";
        return false;
    }
    auto targets = metadata.checkTargetVersions();
    if (!targets) {
        std::filesystem::remove(pending, ec);
        error = "pending .geode target mismatch: " + targets.unwrapErr();
        return false;
    }

    // Same-directory renames provide the atomic boundary. If installing the new
    // file fails, put the known-working package back immediately.
    std::filesystem::rename(current, backup, ec);
    if (ec) {
        std::filesystem::remove(pending, ec);
        error = "could not stage current package backup";
        return false;
    }

    ec.clear();
    std::filesystem::rename(pending, current, ec);
    if (ec) {
        std::error_code rollback;
        std::filesystem::rename(backup, current, rollback);
        error = rollback
            ? "install rename failed and rollback also failed"
            : "install rename failed; rollback restored current version";
        return false;
    }

    auto installedBytes = file::readBinary(current);
    if (!installedBytes || !hashMatches(installedBytes.unwrap(), expectedHash)) {
        std::filesystem::remove(current, ec);
        ec.clear();
        std::filesystem::rename(backup, current, ec);
        error = ec
            ? "post-install hash failed and rollback failed"
            : "post-install hash failed; rollback restored current version";
        return false;
    }

    std::filesystem::remove(backup, ec);
    return true;
}

} // namespace autobot::update
