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
        enterError("A) SEARCH DRIVER NO CORRE: PLAYLAYER NULL");
        return false;
    }

    m_owner = layer;

    // Start coarse enough for real use, then tighten only when search evidence
    // says the current resolution cannot close the route.
    m_oracle = std::make_unique<GeometryDashRuntimeOracle>(layer, 1.0 / 120.0);

    auto root = m_oracle->capture();
    if (!root) {
        enterError("C) RESTORE INCORRECTO: VISIBLE ROOT SNAPSHOT FAILED: " + m_oracle->lastError());
        return false;
    }
    m_visibleRoot = *root;

    if (!m_search.begin(*m_oracle)) {
        enterError("A) SEARCH DRIVER NO CORRE: UNIVERSAL SEARCH BEGIN FAILED: " + m_oracle->lastError());
        return false;
    }

    m_stage = UniversalRuntimeStage::Searching;
    m_reason = "ENGINE RUNTIME ORACLE SEARCH";
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
    m_stallRecoveryCount = 0;
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

    constexpr double kMinStep = 1.0 / 960.0;
    constexpr std::size_t kMaxRefinements = 4;
    if (m_refinement >= kMaxRefinements || m_oracle->stepDt() <= kMinStep + 1e-12) {
        return false;
    }

    if (!m_oracle->restore(m_visibleRoot)) return false;
    m_search.reset(m_oracle.get());

    ++m_refinement;
    const auto nextDt = std::max(kMinStep, m_oracle->stepDt() * 0.5);
    m_oracle->setStepDt(nextDt);

    if (!m_search.begin(*m_oracle)) return false;

    m_reason = "COARSE-TO-FINE SEARCH REFINEMENT " + std::to_string(m_refinement)
        + " DT=" + std::to_string(m_oracle->stepDt());
    return true;
}

bool UniversalRuntimeSession::recoverFromStall(std::string reason) {
    if (!m_oracle || m_stage != UniversalRuntimeStage::Searching) return false;
    ++m_stallRecoveryCount;

    // First change only search ordering/granularity. UniversalSearchCore keeps
    // every live state, so this does not sacrifice a valid solution.
    if (m_search.refineStrategy()) {
        m_reason = "E) SEARCH ES DEMASIADO LENTO: STALL -> STRATEGY TIER "
            + std::to_string(m_search.stats().strategyTier)
            + " (" + reason + ")";
        return true;
    }

    // If all macro tiers have been tried, re-run from the immutable visible
    // root with a finer engine step.
    if (restartAtFinerResolution()) {
        m_reason = "E) SEARCH ES DEMASIADO LENTO: STALL -> FINER ENGINE RESOLUTION ("
            + reason + ")";
        return true;
    }

    return false;
}

bool UniversalRuntimeSession::beginRecoveryFromToken(UniversalToken token, std::string reason) {
    if (!m_oracle || token == kInvalidUniversalToken) return false;
    if (!m_oracle->restore(token)) return false;

    m_search.reset(m_oracle.get());

    if (m_visibleRoot != kInvalidUniversalToken && m_visibleRoot != token) {
        m_oracle->discard(m_visibleRoot);
    }
    if (m_lastSafePlayback != kInvalidUniversalToken && m_lastSafePlayback != token) {
        m_oracle->discard(m_lastSafePlayback);
    }

    m_lastSafePlayback = kInvalidUniversalToken;
    m_visibleRoot = token;
    m_policy.clear();
    m_expectedStates.clear();
    m_playbackCursor = 0;
    m_accumulator = 0.0;
    ++m_recoveryCount;

    if (!m_search.begin(*m_oracle)) return false;

    m_stage = UniversalRuntimeStage::Searching;
    m_reason = "H) PLAYBACK DIVERGE: RECOVERY REPLAN #"
        + std::to_string(m_recoveryCount) + ": " + reason;
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
    return current->words == m_expectedStates[index].words;
}

void UniversalRuntimeSession::searchSlice(std::size_t expansionBudget) {
    if (!m_oracle || m_stage != UniversalRuntimeStage::Searching) return;

    auto stats = m_search.work(*m_oracle, std::max<std::size_t>(1, expansionBudget));

    if (!m_oracle->restore(m_visibleRoot)) {
        enterError("C) RESTORE INCORRECTO: VISIBLE ROOT RESTORE FAILED: " + m_oracle->lastError());
        return;
    }

    if (m_owner) m_owner->m_isPaused = false;

    if (m_search.ready()) {
        m_policy = m_search.policy();
        m_expectedStates = m_search.replayStates();
        if (m_expectedStates.size() != m_policy.size()) {
            enterError("F) REPLAY FALLA: VERIFIED REPLAY STATE COUNT MISMATCH");
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
            enterError("D) SEARCH NO ENCUENTRA POLICY: SEARCH SPACE EXHAUSTED AT FINEST RESOLUTION");
        }
        return;
    }

    if (stats.stage == UniversalSearchStage::Error) {
        enterError("B) ORACLE STEP NO AVANZA: UNIVERSAL SEARCH ERROR: " + m_oracle->lastError());
    }
}

void UniversalRuntimeSession::startPlayback() {
    if (!m_oracle || m_stage != UniversalRuntimeStage::Ready) return;

    if (!m_oracle->restore(m_visibleRoot)) {
        enterError("C) RESTORE INCORRECTO: ROOT RESTORE BEFORE PLAYBACK FAILED: " + m_oracle->lastError());
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
                enterError("I) COMPLETION NO SE DETECTA: VERIFIED POLICY ENDED BEFORE RUNTIME COMPLETION");
            }
            break;
        }

        if (m_lastSafePlayback != kInvalidUniversalToken) {
            m_oracle->discard(m_lastSafePlayback);
        }

        auto safe = m_oracle->capture();
        if (!safe) {
            enterError("C) RESTORE INCORRECTO: PLAYBACK SAFE SNAPSHOT FAILED: " + m_oracle->lastError());
            break;
        }
        m_lastSafePlayback = *safe;

        const auto expectedIndex = m_playbackCursor;
        const auto observation = m_oracle->step(m_policy[m_playbackCursor++]);
        m_accumulator -= step;

        if (!observation.valid) {
            enterError("B) ORACLE STEP NO AVANZA: RUNTIME OBSERVATION INVALID");
            break;
        }

        if (observation.complete) {
            m_stage = UniversalRuntimeStage::Complete;
            m_reason = "LEVEL COMPLETION REACHED";
            if (m_owner) m_owner->m_isPaused = false;
            break;
        }

        if (observation.dead) {
            const auto recoveryToken = m_lastSafePlayback;
            m_lastSafePlayback = kInvalidUniversalToken;
            if (!beginRecoveryFromToken(recoveryToken, "PLAYBACK DIED BEFORE EXPECTED STATE")) {
                enterError("H) PLAYBACK DIVERGE: DESYNC RECOVERY FAILED AFTER DEATH: " + m_oracle->lastError());
            }
            break;
        }

        auto current = m_oracle->capture();
        if (!current) {
            enterError("C) RESTORE INCORRECTO: PLAYBACK CANONICAL SNAPSHOT FAILED: " + m_oracle->lastError());
            break;
        }

        if (!canonicalMatchesExpected(expectedIndex, *current)) {
            const auto recoveryToken = *current;
            if (!beginRecoveryFromToken(recoveryToken, "EXPECTED CANONICAL STATE MISMATCH")) {
                m_oracle->discard(recoveryToken);
                enterError("H) PLAYBACK DIVERGE: DESYNC RECOVERY FAILED: " + m_oracle->lastError());
            }
            break;
        }

        m_oracle->discard(*current);
    }
}

double UniversalRuntimeSession::stepDt() const {
    return m_oracle ? m_oracle->stepDt() : 0.0;
}

RuntimeOracleValidation UniversalRuntimeSession::validation() const {
    return m_oracle ? m_oracle->validation() : RuntimeOracleValidation{};
}

} // namespace autobot::presolve
