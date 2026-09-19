#include "autobot/presolve/UniversalRuntimeSession.hpp"

#include <Geode/Geode.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

using namespace geode::prelude;

namespace autobot::presolve {

bool UniversalRuntimeSession::begin(PlayLayer* layer) {
    reset();
    if (!layer) {
        enterError("PLAYLAYER NULL");
        return false;
    }
    m_owner = layer;
    m_oracle = std::make_unique<GeometryDashRuntimeOracle>(layer, 1.0 / 240.0);
    auto root = m_oracle->capture();
    if (!root) {
        enterError("VISIBLE ROOT SNAPSHOT FAILED: " + m_oracle->lastError());
        return false;
    }
    m_visibleRoot = *root;
    if (!m_search.begin(*m_oracle)) {
        enterError("UNIVERSAL SEARCH BEGIN FAILED: " + m_oracle->lastError());
        return false;
    }
    m_stage = UniversalRuntimeStage::Searching;
    m_reason = "ENGINE RUNTIME ORACLE SEARCH";
    // SEARCHING must keep the external scheduler alive. The hook freezes visible gameplay
    // by returning before the base gameplay update; only oracle-internal steps may advance GD.
    layer->m_isPaused = false;
    return true;
}

void UniversalRuntimeSession::reset() {
    if (m_oracle) {
        m_search.reset(m_oracle.get());
        if (m_lastSafePlayback != kInvalidUniversalToken) m_oracle->discard(m_lastSafePlayback);
        if (m_visibleRoot != kInvalidUniversalToken) m_oracle->discard(m_visibleRoot);
    }
    m_owner = nullptr;
    m_oracle.reset();
    m_search = {};
    m_visibleRoot = kInvalidUniversalToken;
    m_lastSafePlayback = kInvalidUniversalToken;
    m_policy.clear();
    m_expectedStates.clear();
    m_playbackCursor = 0;
    m_refinement = 0;
    m_recoveryCount = 0;
    m_accumulator = 0.0;
    m_stage = UniversalRuntimeStage::Idle;
    m_reason = "IDLE";
}

void UniversalRuntimeSession::setStepCallback(StepCallback callback) {
    if (m_oracle) m_oracle->setStepCallback(std::move(callback));
}

void UniversalRuntimeSession::enterError(std::string reason) {
    m_stage = UniversalRuntimeStage::Error;
    m_reason = std::move(reason);
    if (m_owner) m_owner->m_isPaused = true;
}

bool UniversalRuntimeSession::restartAtFinerResolution() {
    if (!m_oracle || m_visibleRoot == kInvalidUniversalToken) return false;
    if (!m_oracle->restore(m_visibleRoot)) return false;
    m_search.reset(m_oracle.get());
    ++m_refinement;
    m_oracle->setStepDt(m_oracle->stepDt() * 0.5);
    if (!m_search.begin(*m_oracle)) return false;
    m_reason = "SEARCH REFINEMENT " + std::to_string(m_refinement)
        + " DT=" + std::to_string(m_oracle->stepDt());
    return true;
}

bool UniversalRuntimeSession::beginRecoveryFromToken(UniversalToken token, std::string reason) {
    if (!m_oracle || token == kInvalidUniversalToken) return false;
    if (!m_oracle->restore(token)) return false;

    m_search.reset(m_oracle.get());
    if (m_visibleRoot != kInvalidUniversalToken && m_visibleRoot != token) m_oracle->discard(m_visibleRoot);
    if (m_lastSafePlayback != kInvalidUniversalToken && m_lastSafePlayback != token) m_oracle->discard(m_lastSafePlayback);
    m_lastSafePlayback = kInvalidUniversalToken;
    m_visibleRoot = token;
    m_policy.clear();
    m_expectedStates.clear();
    m_playbackCursor = 0;
    m_accumulator = 0.0;
    ++m_recoveryCount;

    if (!m_search.begin(*m_oracle)) return false;
    m_stage = UniversalRuntimeStage::Searching;
    m_reason = "DESYNC RECOVERY REPLAN #" + std::to_string(m_recoveryCount) + ": " + reason;
    if (m_owner) m_owner->m_isPaused = false;
    return true;
}

void UniversalRuntimeSession::fail(std::string reason) {
    enterError(std::move(reason));
}

bool UniversalRuntimeSession::canonicalMatchesExpected(std::size_t index, UniversalToken token) const {
    if (!m_oracle || index >= m_expectedStates.size()) return false;
    auto current = m_oracle->canonicalState(token);
    if (!current) return false;
    // Runtime mismatch detection uses exact semantic words, never hash equality.
    return current->words == m_expectedStates[index].words;
}

void UniversalRuntimeSession::searchSlice(std::size_t expansionBudget) {
    if (!m_oracle || m_stage != UniversalRuntimeStage::Searching) return;
    auto stats = m_search.work(*m_oracle, std::max<std::size_t>(1, expansionBudget));
    if (!m_oracle->restore(m_visibleRoot)) {
        enterError("VISIBLE ROOT RESTORE FAILED: " + m_oracle->lastError());
        return;
    }
    // Keep the scheduler alive between slices; the outer hook owns the visible freeze.
    if (m_owner) m_owner->m_isPaused = false;

    if (m_search.ready()) {
        m_policy = m_search.policy();
        m_expectedStates = m_search.replayStates();
        if (m_expectedStates.size() != m_policy.size()) {
            enterError("VERIFIED REPLAY STATE COUNT MISMATCH");
            return;
        }
        m_stage = UniversalRuntimeStage::Ready;
        m_reason = m_recoveryCount == 0
            ? "TRUE ENGINE POLICY REPLAY VERIFIED"
            : "RECOVERY POLICY REPLAY VERIFIED";
        return;
    }
    if (stats.stage == UniversalSearchStage::Exhausted) {
        if (!restartAtFinerResolution()) {
            enterError("SEARCH REFINEMENT FAILED: " + m_oracle->lastError());
        }
        return;
    }
    if (stats.stage == UniversalSearchStage::Error) {
        enterError("UNIVERSAL SEARCH ERROR: " + m_oracle->lastError());
    }
}

void UniversalRuntimeSession::startPlayback() {
    if (!m_oracle || m_stage != UniversalRuntimeStage::Ready) return;
    if (!m_oracle->restore(m_visibleRoot)) {
        enterError("ROOT RESTORE BEFORE PLAYBACK FAILED: " + m_oracle->lastError());
        return;
    }
    m_search.reset(m_oracle.get());
    if (m_lastSafePlayback != kInvalidUniversalToken) {
        m_oracle->discard(m_lastSafePlayback);
        m_lastSafePlayback = kInvalidUniversalToken;
    }
    m_playbackCursor = 0;
    m_accumulator = 0.0;
    m_stage = UniversalRuntimeStage::Playing;
    m_reason = m_recoveryCount == 0
        ? "AUTOPLAY ENGINE-VERIFIED POLICY"
        : "AUTOPLAY RECOVERED ENGINE-VERIFIED POLICY";
    if (m_owner) m_owner->m_isPaused = false;
}

void UniversalRuntimeSession::playbackFrame(double realDt) {
    if (m_stage == UniversalRuntimeStage::Ready) startPlayback();
    if (!m_oracle || m_stage != UniversalRuntimeStage::Playing) return;
    if (!std::isfinite(realDt) || realDt <= 0.0) realDt = m_oracle->stepDt();
    m_accumulator += realDt;
    const double step = m_oracle->stepDt();
    while (m_accumulator + 1e-12 >= step && m_stage == UniversalRuntimeStage::Playing) {
        if (m_playbackCursor >= m_policy.size()) {
            const auto final = m_oracle->observe();
            if (final.complete) {
                m_stage = UniversalRuntimeStage::Complete;
                m_reason = "LEVEL COMPLETION REACHED";
                if (m_owner) m_owner->m_isPaused = false;
            } else {
                enterError("VERIFIED POLICY ENDED BEFORE RUNTIME COMPLETION");
            }
            break;
        }

        if (m_lastSafePlayback != kInvalidUniversalToken) m_oracle->discard(m_lastSafePlayback);
        auto safe = m_oracle->capture();
        if (!safe) {
            enterError("PLAYBACK SAFE SNAPSHOT FAILED: " + m_oracle->lastError());
            break;
        }
        m_lastSafePlayback = *safe;

        const auto expectedIndex = m_playbackCursor;
        const auto observation = m_oracle->step(m_policy[m_playbackCursor++]);
        m_accumulator -= step;

        if (!observation.valid) {
            enterError("RUNTIME OBSERVATION INVALID");
            break;
        }
        if (observation.complete) {
            m_stage = UniversalRuntimeStage::Complete;
            m_reason = "LEVEL COMPLETION REACHED";
            if (m_owner) m_owner->m_isPaused = false;
            break;
        }

        if (observation.dead) {
            // Recover from the immediately preceding alive real state, not 0%.
            const auto recoveryToken = m_lastSafePlayback;
            m_lastSafePlayback = kInvalidUniversalToken;
            if (!beginRecoveryFromToken(recoveryToken, "PLAYBACK DIED BEFORE EXPECTED STATE")) {
                enterError("DESYNC RECOVERY FAILED AFTER DEATH: " + m_oracle->lastError());
            }
            break;
        }

        auto current = m_oracle->capture();
        if (!current) {
            enterError("PLAYBACK CANONICAL SNAPSHOT FAILED: " + m_oracle->lastError());
            break;
        }
        if (!canonicalMatchesExpected(expectedIndex, *current)) {
            const auto recoveryToken = *current;
            if (!beginRecoveryFromToken(recoveryToken, "EXPECTED CANONICAL STATE MISMATCH")) {
                m_oracle->discard(recoveryToken);
                enterError("DESYNC RECOVERY FAILED: " + m_oracle->lastError());
            }
            break;
        }
        m_oracle->discard(*current);
    }
}

double UniversalRuntimeSession::stepDt() const { return m_oracle ? m_oracle->stepDt() : 0.0; }
RuntimeOracleValidation UniversalRuntimeSession::validation() const { return m_oracle ? m_oracle->validation() : RuntimeOracleValidation{}; }

} // namespace autobot::presolve
