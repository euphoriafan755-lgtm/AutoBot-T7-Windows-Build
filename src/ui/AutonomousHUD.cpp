#include "autobot/ui/AutonomousHUD.hpp"

#include <Geode/Geode.hpp>
#include <fmt/format.h>

#include <string>

using namespace geode::prelude;

namespace autobot::ui {

bool AutonomousHUD::attach(PlayLayer* playLayer) {
    if (m_label) return true;
    if (!playLayer) return false;

    m_label = CCLabelBMFont::create("AUTOBOT: WAITING", "chatFont.fnt");
    if (!m_label) return false;

    const auto winSize = CCDirector::get()->getWinSize();
    m_label->setAnchorPoint({0.0f, 1.0f});
    m_label->setPosition({8.0f, winSize.height - 8.0f});
    m_label->setScale(0.31f);
    m_label->setZOrder(1000000);
    m_label->setOpacity(235);
    m_label->setID("autobot-control-hud"_spr);
    playLayer->addChild(m_label, 1000000);
    return true;
}

void AutonomousHUD::update(
    core::GameSnapshot const& snapshot,
    control::AutonomousDecision const& decision,
    bool solverDebug
) {
    if (!m_label) return;

    auto const& plan = decision.plan;
    const auto mode = snapshot.valid ? core::toString(snapshot.player.mode) : "UNKNOWN";

    std::string targetLine = "TARGET: NONE";
    if (decision.targetPrimitiveIndex != world::kInvalidPrimitiveIndex) {
        targetLine = fmt::format(
            "TARGET: PRIM {} ID {} DIST {:.2f}",
            decision.targetPrimitiveIndex,
            decision.targetObjectID,
            decision.targetDistance
        );
    }

    std::string selectedAction = "NONE";
    solver::TrajectoryResult const* selected = nullptr;
    if (plan.selectedTrajectory < plan.trajectories.size()) {
        selected = &plan.trajectories[plan.selectedTrajectory];
        selectedAction = selected->candidate.label;
    }

    const auto& modelError = plan.lastModelError;
    const bool modelMismatch = !modelError.modeMatched
        || !modelError.landingMatched
        || !modelError.collisionMatched;

    std::string text = fmt::format(
        "AUTOBOT: {}\n"
        "CONTROL: {}\n"
        "MODE: {}\n"
        "ACTION: {}\n"
        "CANDIDATES: {}\n"
        "SELECTED ACTION: {}\n"
        "CONFIDENCE: {:.2f}\n"
        "{}\n"
        "GAME STATE READY: {}\n"
        "WORLD READY: {}\n"
        "PHYSICS READY: {}\n"
        "PLANNER READY: {}\n"
        "MODEL ERROR: {:.2f}{}",
        decision.active ? "ACTIVE" : "STOPPED",
        control::toString(decision.ownership),
        mode,
        control::toString(decision.action),
        plan.trajectories.size(),
        selectedAction,
        plan.confidence,
        targetLine,
        plan.gameStateReady ? "YES" : "NO",
        plan.worldReady ? "YES" : "NO",
        plan.physicsReady ? "YES" : "NO",
        plan.plannerReady ? "YES" : "NO",
        modelError.magnitude(),
        modelMismatch ? " DIVERGENCE" : ""
    );

    if (solverDebug) {
        std::string scores;
        for (std::size_t i = 0; i < plan.trajectories.size(); ++i) {
            auto const& trajectory = plan.trajectories[i];
            if (!scores.empty()) scores += " | ";
            scores += fmt::format(
                "{}{}:{:.1f}",
                i == plan.selectedTrajectory ? "*" : "",
                trajectory.candidate.label,
                trajectory.score
            );
        }
        if (scores.empty()) scores = "NONE";

        if (selected) {
            text += fmt::format(
                "\n--- SOLVER DEBUG ---\n"
                "CANDIDATE SCORES: {}\n"
                "PRED COLLISION: {}\n"
                "PRED LANDING/SURFACE: {}\n"
                "CLEARANCE: {:.2f}\n"
                "PORTAL PREDICTION: {}\n"
                "MODE TRANSITION: {} -> {}\n"
                "PLANNER: {:.3f}ms | GEN {:.3f} | SIM {:.3f} | SCORE {:.3f}\n"
                "MODEL DELTA: dx {:.2f} dy {:.2f} dvx {:.2f} dvy {:.2f}",
                scores,
                selected->fatalCollision ? "YES" : "NO",
                selected->landed ? "YES" : "NO",
                selected->minimumClearance,
                selected->portalCrossed ? "YES" : "NO",
                mode,
                core::toString(selected->finalMode),
                plan.plannerDurationMs,
                plan.candidateGenerationMs,
                plan.physicsSimulationMs,
                plan.trajectoryScoringMs,
                modelError.dx,
                modelError.dy,
                modelError.dvx,
                modelError.dvy
            );
        } else {
            text += fmt::format(
                "\n--- SOLVER DEBUG ---\n"
                "CANDIDATE SCORES: {}\n"
                "PLANNER: {:.3f}ms | GEN {:.3f} | SIM {:.3f} | SCORE {:.3f}\n"
                "REASON: {}",
                scores,
                plan.plannerDurationMs,
                plan.candidateGenerationMs,
                plan.physicsSimulationMs,
                plan.trajectoryScoringMs,
                decision.reason
            );
        }
    }

    m_label->setString(text.c_str());
}

} // namespace autobot::ui
