#include "autobot/ui/AutonomousHUD.hpp"
#include "autobot/core/BuildInfo.hpp"

#include <Geode/Geode.hpp>
#include <fmt/format.h>

#include <limits>
#include <string>

using namespace geode::prelude;

namespace autobot::ui {
namespace {

solver::TrajectoryResult const* findTrajectory(
    solver::PlanDecision const& plan,
    char const* label
) {
    for (auto const& trajectory : plan.trajectories) {
        if (trajectory.candidate.label == label) return &trajectory;
    }
    return nullptr;
}

std::string trajectoryTruthLine(
    char const* prefix,
    solver::TrajectoryResult const* trajectory
) {
    if (!trajectory) return fmt::format("{}: NONE", prefix);
    const auto tick = trajectory->collisionTick == std::numeric_limits<std::size_t>::max()
        ? std::string("-")
        : fmt::format("{}", trajectory->collisionTick);
    return fmt::format(
        "{}: {} FATAL={} TICK={} SCORE={:.0f}",
        prefix,
        solver::toString(trajectory->classification),
        trajectory->fatalCollision ? "YES" : "NO",
        tick,
        trajectory->score
    );
}

} // namespace

bool AutonomousHUD::attach(PlayLayer* playLayer) {
    if (m_label) return true;
    if (!playLayer) return false;

    m_label = CCLabelBMFont::create("AUTOBOT: WAITING", "chatFont.fnt");
    if (!m_label) return false;

    const auto winSize = CCDirector::get()->getWinSize();
    m_label->setAnchorPoint({0.0f, 1.0f});
    m_label->setPosition({8.0f, winSize.height - 8.0f});
    m_label->setScale(0.28f);
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
        "AutoBot T7 v{} | Build {} | {}\n"
        "PRE-RUN: {}\n"
        "PRE-RUN REASON: {}\n"
        "FULL POLICY REPLAY: {}\n"
        "SIMULATED COMPLETION: {:.1f}%\n"
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
        "STATUS: {}\n"
        "MODEL ERROR: {:.2f}{}",
        core::build::version(),
        core::build::shortCommit(),
        core::build::buildDate(),
        presolve::toString(decision.preRunStage),
        decision.preRunReason,
        decision.fullPolicyReplayPassed ? "PASS" : "NOT PASSED",
        decision.simulatedCompletion,
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
        decision.reason.empty() ? plan.reason : decision.reason,
        modelError.magnitude(),
        modelMismatch ? " DIVERGENCE" : ""
    );

    if (solverDebug) {
        auto const* noPress = findTrajectory(plan, "NO PRESS");
        auto const* pressNow = findTrajectory(plan, "PRESS NOW");
        const auto& truth = plan.modelTruth;

        double simX = snapshot.player.x;
        double simY = snapshot.player.y;
        if (plan.hasPredictedNextState) {
            simX = plan.predictedNextState.x;
            simY = plan.predictedNextState.y;
        }

        text += fmt::format(
            "\n--- MODEL TRUTH ---\n"
            "REAL X/Y: {:.2f} / {:.2f}\n"
            "SIM NEXT X/Y: {:.2f} / {:.2f}\n"
            "DELTA X/Y: {:.2f} / {:.2f}\n"
            "DT: real {:.5f}s sim {:.3f} ticks ratio {:.2f}\n"
            "RAW VX: {:.3f} | OBS WORLD VX: {:.2f}\n"
            "NEXT HAZARD DISTANCE: {:.2f}\n"
            "{}\n"
            "{}\n"
            "SELECTED: {}\n"
            "FALSE SAFE COUNT: {}",
            snapshot.player.x,
            snapshot.player.y,
            simX,
            simY,
            modelError.dx,
            modelError.dy,
            truth.realDtSeconds,
            truth.simDtNormalized,
            truth.physicsTicksPerSecond,
            truth.rawVelocityX,
            truth.observedWorldVelocityX,
            truth.nearestHazardDistance,
            trajectoryTruthLine("NO PRESS", noPress),
            trajectoryTruthLine("PRESS NOW", pressNow),
            selectedAction,
            decision.falseSafeTotal
        );

        if (selected) {
            text += fmt::format(
                "\nCLEARANCE: {:.2f} | HORIZON: {} {}\n"
                "PLANNER: {:.3f}ms | GEN {:.3f} | SIM {:.3f} | SCORE {:.3f}",
                selected->minimumClearance,
                selected->horizonTicks,
                selected->horizonConclusive ? "CONCLUSIVE" : "INCONCLUSIVE",
                plan.plannerDurationMs,
                plan.candidateGenerationMs,
                plan.physicsSimulationMs,
                plan.trajectoryScoringMs
            );
        }
    }

    m_label->setString(text.c_str());
}

void AutonomousHUD::updatePreRun(
    presolve::PreRunStage stage,
    std::string const& reason,
    bool replayPassed,
    double simulatedCompletion
) {
    if (!m_label) return;
    const auto text = fmt::format(
        "AutoBot T7 v{} | Build {} | {}\n"
        "PRE-RUN: {}\n"
        "REASON: {}\n"
        "FULL POLICY REPLAY: {}\n"
        "SIMULATED COMPLETION: {:.1f}%\n"
        "AUTOBOT: FROZEN UNTIL READY",
        core::build::version(),
        core::build::shortCommit(),
        core::build::buildDate(),
        presolve::toString(stage),
        reason.empty() ? "WORKING" : reason,
        replayPassed ? "PASS" : "NOT PASSED",
        simulatedCompletion
    );
    m_label->setString(text.c_str());
}

} // namespace autobot::ui
