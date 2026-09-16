#include "autobot/solver/GlobalPlanner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace autobot::solver {
namespace {

struct Interval {
    double low = 0.0;
    double high = 0.0;
};

struct BeamState {
    double score = -std::numeric_limits<double>::infinity();
    double minClearance = std::numeric_limits<double>::infinity();
    std::vector<GlobalRouteStep> route;
};

struct RectEdges {
    double left;
    double right;
    double bottom;
    double top;
};

RectEdges edges(world::WorldRect const& r) {
    const double x2 = static_cast<double>(r.x + r.width);
    const double y2 = static_cast<double>(r.y + r.height);
    return {
        std::min<double>(r.x, x2),
        std::max<double>(r.x, x2),
        std::min<double>(r.y, y2),
        std::max<double>(r.y, y2),
    };
}

bool xIntersects(RectEdges const& e, double a, double b) {
    const double low = std::min(a, b);
    const double high = std::max(a, b);
    return e.right >= low && e.left <= high;
}

bool yIntersects(RectEdges const& e, double low, double high) {
    return e.top >= low && e.bottom <= high;
}

double center(Interval const& i) {
    return (i.low + i.high) * 0.5;
}

double width(Interval const& i) {
    return std::max(0.0, i.high - i.low);
}

double mobilityPerWorldUnit(core::GameMode mode) {
    switch (mode) {
        case core::GameMode::Wave: return 1.05;
        case core::GameMode::Ship: return 0.90;
        case core::GameMode::Swing: return 0.80;
        case core::GameMode::Ufo: return 0.65;
        case core::GameMode::Robot: return 0.58;
        case core::GameMode::Cube: return 0.52;
        case core::GameMode::Ball: return 0.48;
        case core::GameMode::Spider: return 1.20;
        case core::GameMode::Unknown: return 0.35;
    }
    return 0.35;
}

std::vector<Interval> freeIntervals(
    GeneralWorldView const& worldView,
    double xNear,
    double xFar,
    double bandLow,
    double bandHigh,
    double playerHalfHeight,
    bool& uncertain
) {
    std::vector<Interval> blocked;
    blocked.reserve(worldView.events.size());

    constexpr double kSafetyMargin = 4.0;
    for (auto const& event : worldView.events) {
        if (event.kind == GlobalEventKind::Portal || event.kind == GlobalEventKind::Interactive) continue;
        const auto e = edges(event.bounds);
        if (!xIntersects(e, xNear, xFar)) continue;

        // Solids, hazards and unknown gameplay geometry are impassable volume
        // for global corridor purposes. A solid surface itself may be touched
        // legally, so only hazards/unknown geometry receive the extra margin.
        const double margin = event.kind == GlobalEventKind::Solid ? 0.0 : kSafetyMargin;
        double low = e.bottom - playerHalfHeight - margin;
        double high = e.top + playerHalfHeight + margin;
        low = std::max(low, bandLow);
        high = std::min(high, bandHigh);
        if (high <= low) continue;
        blocked.push_back({low, high});
        uncertain = uncertain || event.kind == GlobalEventKind::Unknown
            || event.dynamic
            || !event.verifiedGeometry;
    }

    std::sort(blocked.begin(), blocked.end(), [](Interval const& a, Interval const& b) {
        if (a.low != b.low) return a.low < b.low;
        return a.high < b.high;
    });

    std::vector<Interval> merged;
    for (auto const& interval : blocked) {
        if (merged.empty() || interval.low > merged.back().high) {
            merged.push_back(interval);
        } else {
            merged.back().high = std::max(merged.back().high, interval.high);
        }
    }

    std::vector<Interval> free;
    double cursor = bandLow;
    for (auto const& interval : merged) {
        if (interval.low > cursor) free.push_back({cursor, interval.low});
        cursor = std::max(cursor, interval.high);
    }
    if (cursor < bandHigh) free.push_back({cursor, bandHigh});

    const double minimumWidth = std::max(8.0, playerHalfHeight * 0.35);
    free.erase(
        std::remove_if(free.begin(), free.end(), [&](Interval const& i) {
            return width(i) < minimumWidth;
        }),
        free.end()
    );
    return free;
}

bool canReach(
    GlobalRouteStep const& from,
    Interval const& to,
    double horizontalSpan,
    core::GameMode mode
) {
    const double allowance = std::max(24.0, horizontalSpan * mobilityPerWorldUnit(mode));
    const double expandedLow = from.yMin - allowance;
    const double expandedHigh = from.yMax + allowance;
    return to.high >= expandedLow && to.low <= expandedHigh;
}

double transitionCost(GlobalRouteStep const& from, Interval const& to) {
    const double dy = std::abs(from.centerY - center(to));
    const double corridor = width(to);
    return dy * 0.08 - std::min(corridor, 600.0) * 0.02;
}

} // namespace

GlobalRoutePlan GlobalPlanner::plan(
    core::GameSnapshot const& snapshot,
    GeneralWorldView const& worldView,
    world::CollisionWorld const& collisionWorld,
    SearchBudget const& budget
) const {
    GlobalRoutePlan result{};
    if (!snapshot.valid || !worldView.complete || !collisionWorld.ready()) return result;

    result.direction = worldView.direction;
    const std::size_t slices = std::clamp<std::size_t>(budget.globalSlices, 4, 40);
    const std::size_t beamWidth = std::clamp<std::size_t>(budget.globalBeamWidth, 2, 32);
    const double lookahead = std::min(worldView.lookaheadX, budget.globalLookaheadX);
    if (lookahead <= 1.0) return result;

    const double sliceWidth = lookahead / static_cast<double>(slices);
    const double bandLow = snapshot.player.y - worldView.lookaheadY * 0.5;
    const double bandHigh = snapshot.player.y + worldView.lookaheadY * 0.5;
    const double playerHalfHeight = std::max(4.0, snapshot.player.objectBoundsHeight * 0.5);

    std::vector<BeamState> beam;
    beam.reserve(beamWidth);

    for (std::size_t slice = 0; slice < slices; ++slice) {
        const double forwardNear = static_cast<double>(slice) * sliceWidth;
        const double forwardFar = static_cast<double>(slice + 1) * sliceWidth;
        const double xNear = snapshot.player.x + result.direction * forwardNear;
        const double xFar = snapshot.player.x + result.direction * forwardFar;

        bool sliceUncertain = false;
        auto intervals = freeIntervals(
            worldView,
            xNear,
            xFar,
            bandLow,
            bandHigh,
            playerHalfHeight,
            sliceUncertain
        );
        result.uncertain = result.uncertain || sliceUncertain;
        if (intervals.empty()) break;

        std::vector<BeamState> next;
        next.reserve(std::min<std::size_t>(intervals.size() * std::max<std::size_t>(beam.size(), 1), 256));

        if (beam.empty()) {
            for (auto const& interval : intervals) {
                if (snapshot.player.y < interval.low || snapshot.player.y > interval.high) continue;
                BeamState state{};
                GlobalRouteStep step{};
                step.xNear = xNear;
                step.xFar = xFar;
                step.yMin = interval.low;
                step.yMax = interval.high;
                step.centerY = std::clamp(snapshot.player.y, interval.low, interval.high);
                step.clearance = width(interval) * 0.5;
                state.route.push_back(step);
                state.minClearance = step.clearance;
                state.score = forwardFar + std::min(step.clearance, 400.0) * 0.2;
                next.push_back(std::move(state));
                ++result.branchesConsidered;
            }

            // If the player lies on a collision boundary, use the nearest free
            // corridor rather than declaring the entire global plan impossible.
            if (next.empty()) {
                auto nearest = std::min_element(
                    intervals.begin(),
                    intervals.end(),
                    [&](Interval const& a, Interval const& b) {
                        return std::abs(center(a) - snapshot.player.y)
                            < std::abs(center(b) - snapshot.player.y);
                    }
                );
                if (nearest != intervals.end()) {
                    BeamState state{};
                    GlobalRouteStep step{};
                    step.xNear = xNear;
                    step.xFar = xFar;
                    step.yMin = nearest->low;
                    step.yMax = nearest->high;
                    step.centerY = std::clamp(snapshot.player.y, nearest->low, nearest->high);
                    step.clearance = width(*nearest) * 0.5;
                    state.route.push_back(step);
                    state.minClearance = step.clearance;
                    state.score = forwardFar - std::abs(center(*nearest) - snapshot.player.y) * 0.2;
                    next.push_back(std::move(state));
                    result.uncertain = true;
                    ++result.branchesConsidered;
                }
            }
        } else {
            for (auto const& state : beam) {
                auto const& previous = state.route.back();
                for (auto const& interval : intervals) {
                    ++result.branchesConsidered;
                    if (!canReach(previous, interval, sliceWidth, snapshot.player.mode)) continue;

                    BeamState candidate = state;
                    GlobalRouteStep step{};
                    step.xNear = xNear;
                    step.xFar = xFar;
                    step.yMin = interval.low;
                    step.yMax = interval.high;
                    // Preserve the previous vertical lane whenever possible.
                    // Global planning should avoid inventing unnecessary climbs.
                    step.centerY = std::clamp(previous.centerY, interval.low, interval.high);
                    step.clearance = std::min(
                        step.centerY - interval.low,
                        interval.high - step.centerY
                    );
                    candidate.route.push_back(step);
                    candidate.minClearance = std::min(candidate.minClearance, step.clearance);
                    candidate.score += sliceWidth
                        - transitionCost(previous, interval)
                        + std::min(step.clearance, 400.0) * 0.03;
                    next.push_back(std::move(candidate));
                }
            }
        }

        if (next.empty()) break;
        std::stable_sort(next.begin(), next.end(), [](BeamState const& a, BeamState const& b) {
            if (a.route.size() != b.route.size()) return a.route.size() > b.route.size();
            return a.score > b.score;
        });
        if (next.size() > beamWidth) next.resize(beamWidth);
        result.exploredStates += next.size();
        beam = std::move(next);
    }

    if (beam.empty()) return result;
    auto const& best = *std::max_element(
        beam.begin(),
        beam.end(),
        [](BeamState const& a, BeamState const& b) {
            if (a.route.size() != b.route.size()) return a.route.size() < b.route.size();
            return a.score < b.score;
        }
    );
    if (best.route.empty()) return result;

    result.steps = best.route;
    result.minimumCorridorClearance = best.minClearance;
    result.plannedForwardDistance = static_cast<double>(best.route.size()) * sliceWidth;

    // Local MPC receives a near-term corridor target while the complete route
    // remains available for global continuity.
    const std::size_t targetIndex = std::min<std::size_t>(3, best.route.size() - 1);
    auto const& target = best.route[targetIndex];
    result.targetX = (target.xNear + target.xFar) * 0.5;
    result.targetY = target.centerY;
    result.targetYMin = target.yMin;
    result.targetYMax = target.yMax;

    // Find the first portal geometrically intersecting the selected route.
    for (auto const& event : worldView.events) {
        if (event.kind != GlobalEventKind::Portal) continue;
        const auto portal = edges(event.bounds);
        for (auto const& step : result.steps) {
            if (xIntersects(portal, step.xNear, step.xFar)
                && yIntersects(portal, step.yMin, step.yMax)) {
                result.nextPortalPrimitive = event.primitiveIndex;
                break;
            }
        }
        if (result.nextPortalPrimitive != world::kInvalidPrimitiveIndex) break;
    }

    result.valid = best.route.size() >= 2;
    return result;
}

} // namespace autobot::solver
