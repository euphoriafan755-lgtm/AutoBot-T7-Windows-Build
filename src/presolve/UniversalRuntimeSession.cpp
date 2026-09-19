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
    layer->m_isPaused = true;
    return true;
}

void UniversalRuntimeSession::reset() {
    if (m_oracle) {
        m_search.reset(m_oracle.get());
        if (m_visibleRoot != kInvalidUniversalToken) m_oracle->discard(m_visibleRoot);
    }
    m_owner = nullptr;
    m_oracle.reset();
    m_search = {};
    m_visibleRoot = kInvalidUniversalToken;
    m_policy.clear();
    m_playbackCursor = 0;
    m_refinement = 0;
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

void UniversalRuntimeSession::searchSlice(std::size_t expansionBudget) {
    if (!m_oracle || m_stage != UniversalRuntimeStage::Searching) return;
    auto stats = m_search.work(*m_oracle, std::max<std::size_t>(1, expansionBudget));
    if (!m_oracle->restore(m_visibleRoot)) {
        enterError("VISIBLE ROOT RESTORE FAILED: " + m_oracle->lastError());
        return;
    }
    if (m_owner) m_owner->m_isPaused = true;

    if (m_search.ready()) {
        m_policy = m_search.policy();
        m_stage = UniversalRuntimeStage::Ready;
        m_reason = "TRUE ENGINE POLICY REPLAY VERIFIED";
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
    m_playbackCursor = 0;
    m_accumulator = 0.0;
    m_stage = UniversalRuntimeStage::Playing;
    m_reason = "AUTOPLAY ENGINE-VERIFIED POLICY";
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
        const auto observation = m_oracle->step(m_policy[m_playbackCursor++]);
        m_accumulator -= step;
        if (!observation.valid || observation.dead) {
            enterError("RUNTIME POLICY DIVERGED OR DIED");
            break;
        }
        if (observation.complete) {
            m_stage = UniversalRuntimeStage::Complete;
            m_reason = "LEVEL COMPLETION REACHED";
            if (m_owner) m_owner->m_isPaused = false;
            break;
        }
    }
}

double UniversalRuntimeSession::stepDt() const {
    return m_oracle ? m_oracle->stepDt() : 0.0;
}

} // namespace autobot::presolve
