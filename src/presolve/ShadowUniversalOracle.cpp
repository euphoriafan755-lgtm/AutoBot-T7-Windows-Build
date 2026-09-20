#include "autobot/presolve/ShadowUniversalOracle.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <vector>

namespace autobot::presolve {
namespace {

std::uint64_t bits(double value) {
    return std::bit_cast<std::uint64_t>(value);
}

struct Edges {
    double left;
    double right;
    double bottom;
    double top;
};

Edges edges(world::WorldRect const& rect) {
    const double x2 = static_cast<double>(rect.x + rect.width);
    const double y2 = static_cast<double>(rect.y + rect.height);
    return {
        std::min<double>(rect.x, x2),
        std::max<double>(rect.x, x2),
        std::min<double>(rect.y, y2),
        std::max<double>(rect.y, y2),
    };
}

Edges playerEdges(solver::SimState const& state) {
    const double halfW = std::max(0.5, state.objectWidth * 0.5);
    const double halfH = std::max(0.5, state.objectHeight * 0.5);
    return {
        state.x - halfW,
        state.x + halfW,
        state.y - halfH,
        state.y + halfH,
    };
}

bool intersects(Edges const& a, Edges const& b) {
    return a.left <= b.right && a.right >= b.left
        && a.bottom <= b.top && a.top >= b.bottom;
}

bool canLand(core::GameMode mode) {
    return mode == core::GameMode::Cube
        || mode == core::GameMode::Ball
        || mode == core::GameMode::Robot
        || mode == core::GameMode::Spider;
}

bool resolveSolidContact(
    solver::SimState const& previous,
    solver::SimState& current,
    world::WorldRect const& solidBounds
) {
    if (!canLand(current.mode)) return false;

    const auto solid = edges(solidBounds);
    const auto prevPlayer = playerEdges(previous);
    const auto curPlayer = playerEdges(current);

    if (!current.upsideDown) {
        const bool descending = current.vy <= 0.0;
        const bool crossedTop = prevPlayer.bottom >= solid.top - 1.0
            && curPlayer.bottom <= solid.top + 1.0;
        if (descending && crossedTop) {
            current.y = solid.top + current.objectHeight * 0.5;
            current.vy = 0.0;
            current.grounded = true;
            return true;
        }
    } else {
        const bool ascending = current.vy >= 0.0;
        const bool crossedBottom = prevPlayer.top <= solid.bottom + 1.0
            && curPlayer.top >= solid.bottom - 1.0;
        if (ascending && crossedBottom) {
            current.y = solid.bottom - current.objectHeight * 0.5;
            current.vy = 0.0;
            current.grounded = true;
            return true;
        }
    }
    return false;
}

std::uint64_t mix64(std::uint64_t x) {
    x ^= x >> 30U;
    x *= 0xbf58476d1ce4e5b9ULL;
    x ^= x >> 27U;
    x *= 0x94d049bb133111ebULL;
    x ^= x >> 31U;
    return x;
}

} // namespace

void ShadowUniversalOracle::configure(
    world::CollisionWorld const* collisionWorld,
    solver::PhysicsValidationHarness const* validation,
    double completionBoundaryX,
    bool platformer
) {
    m_collisionWorld = collisionWorld;
    m_validation = validation;
    m_completionBoundaryX = completionBoundaryX;
    m_platformer = platformer;
}

bool ShadowUniversalOracle::setLiveRoot(
    core::GameSnapshot const& snapshot,
    bool p1Holding
) {
    m_snapshots.clear();
    m_nextToken = 0;
    m_error.clear();

    if (!snapshot.valid || snapshot.player.dead || !m_collisionWorld || !m_collisionWorld->ready()) {
        m_ready = false;
        m_error = "SHADOW ROOT INVALID OR WORLD NOT READY";
        return false;
    }

    m_state = {};
    m_state.snapshot = snapshot;
    m_state.holding = p1Holding;
    m_state.snapshot.player.holding = p1Holding;
    m_state.complete = false;
    m_state.shadowTick = 0;

    m_rootX = snapshot.player.x;
    m_rootProgress = static_cast<double>(snapshot.levelProgress);
    m_direction = snapshot.player.velocityX < -0.001 ? -1.0 : 1.0;
    m_ready = true;
    return true;
}

double ShadowUniversalOracle::stepDt() const {
    if (!m_ready) return 0.0;
    // A shadow policy element is one normalized GD physics tick. Keeping this
    // domain fixed prevents policy length and action timing from inheriting the
    // render/sample cadence used to observe the live game.
    return 1.0;
}

solver::SimState ShadowUniversalOracle::makeSimState() const {
    solver::SimState state{};
    auto const& p = m_state.snapshot.player;
    state.x = p.x;
    state.y = p.y;
    state.vx = p.velocityX;
    state.vy = p.velocityY;
    state.mode = p.mode;
    state.mini = p.mini;
    state.grounded = p.grounded;
    state.upsideDown = p.upsideDown;
    state.holding = m_state.holding;
    state.alive = !p.dead;
    state.objectWidth = std::max(1.0, p.objectBoundsWidth);
    state.objectHeight = std::max(1.0, p.objectBoundsHeight);
    state.sampleDt = stepDt();
    state.verticalPositionScale = m_validation
        ? m_validation->calibration(p.mode).yPositionScale()
        : 0.9;
    state.rawGravity = p.gravity;
    state.gravityModifier = p.gravityModifier;
    state.jumpVelocity = p.jumpVelocity;
    state.speedScalar = p.speed;
    return state;
}

void ShadowUniversalOracle::applySimState(solver::SimState const& state) {
    auto& p = m_state.snapshot.player;
    p.x = state.x;
    p.y = state.y;
    p.velocityX = state.vx;
    p.velocityY = state.vy;
    p.mode = state.mode;
    p.mini = state.mini;
    p.grounded = state.grounded;
    p.upsideDown = state.upsideDown;
    p.holding = state.holding;
    p.dead = !state.alive;
}

void ShadowUniversalOracle::resolveBasicCollision(
    solver::SimState const& previous,
    solver::SimState& current
) const {
    if (!m_collisionWorld || !m_collisionWorld->ready()) return;

    const auto query = m_collisionWorld->queryAhead(
        static_cast<float>(current.x),
        static_cast<float>(current.y),
        240.0f,
        240.0f
    );
    auto const& primitives = m_collisionWorld->primitives();

    auto currentEdges = playerEdges(current);
    for (auto primitiveIndex : query.primitiveIndices) {
        if (primitiveIndex >= primitives.size()) continue;
        auto const& primitive = primitives[primitiveIndex];
        if (!primitive.enabled || !primitive.indexable) continue;

        const auto objectEdges = edges(primitive.broadphaseBounds);
        if (!intersects(currentEdges, objectEdges)) continue;

        if (primitive.classification == world::GameplayObjectType::Hazard) {
            current.alive = false;
            return;
        }

        if (primitive.classification == world::GameplayObjectType::Solid) {
            if (!resolveSolidContact(previous, current, primitive.broadphaseBounds)) {
                current.alive = false;
                return;
            }
            currentEdges = playerEdges(current);
        }
    }
}

double ShadowUniversalOracle::progressFor(double x) const {
    const double remaining = m_direction * (m_completionBoundaryX - m_rootX);
    if (remaining > 0.001) {
        const double advanced = m_direction * (x - m_rootX);
        const double fraction = std::clamp(advanced / remaining, 0.0, 1.0);
        return std::clamp(
            m_rootProgress + (100.0 - m_rootProgress) * fraction,
            0.0,
            100.0
        );
    }
    return std::clamp(m_rootProgress, 0.0, 100.0);
}

UniversalObservation ShadowUniversalOracle::observe() const {
    UniversalObservation out{};
    if (!m_ready) return out;

    out.valid = true;
    out.dead = m_state.snapshot.player.dead;
    out.complete = m_state.complete;
    // Build #2 deliberately keeps global shadow search single-player. Dual
    // control remains owned by the existing realtime joint MPC.
    out.dual = false;
    out.platformer = m_platformer;
    out.progress = progressFor(m_state.snapshot.player.x);

    const auto c = canonical(m_state);
    out.fingerprint = c.hash;
    return out;
}

std::optional<UniversalToken> ShadowUniversalOracle::capture() {
    if (!m_ready) return std::nullopt;
    const auto token = ++m_nextToken;
    m_snapshots[token] = m_state;
    return token;
}

bool ShadowUniversalOracle::restore(UniversalToken token) {
    const auto it = m_snapshots.find(token);
    if (it == m_snapshots.end()) {
        m_error = "SHADOW TOKEN NOT FOUND";
        return false;
    }
    m_state = it->second;
    return true;
}

UniversalObservation ShadowUniversalOracle::step(UniversalAction action) {
    if (!m_ready || m_state.snapshot.player.dead || m_state.complete) {
        return observe();
    }

    auto state = makeSimState();
    auto previous = state;

    if (m_platformer) {
        const double speed = std::max(std::abs(state.vx), std::abs(m_state.snapshot.player.velocityX));
        if (action.p1Left && !action.p1Right) state.vx = -speed;
        else if (action.p1Right && !action.p1Left) state.vx = speed;
    }

    const bool desiredHold = action.p1Hold;
    const bool pressEdge = desiredHold && !m_state.holding;
    const bool releaseEdge = !desiredHold && m_state.holding;

    solver::ModeCalibration calibration{};
    if (m_validation) calibration = m_validation->calibration(state.mode);
    const auto dt = stepDt();
    solver::PhysicsStepContext context{calibration, dt};
    m_physics.modelFor(state.mode).step(
        state,
        desiredHold,
        pressEdge,
        releaseEdge,
        context
    );
    state.holding = desiredHold;

    resolveBasicCollision(previous, state);
    applySimState(state);
    m_state.holding = desiredHold;
    ++m_state.shadowTick;
    ++m_simulatedSteps;

    const double physicsTicksPerSecond =
        std::isfinite(calibration.physicsTicksPerSecond)
        && calibration.physicsTicksPerSecond > 1.0
        ? calibration.physicsTicksPerSecond
        : 60.0;
    m_state.snapshot.levelTime += 1.0 / physicsTicksPerSecond;
    m_state.snapshot.levelProgress = static_cast<float>(progressFor(state.x));

    const bool passedBoundary = m_direction >= 0.0
        ? state.x >= m_completionBoundaryX
        : state.x <= m_completionBoundaryX;
    if (m_completionBoundaryX != 0.0 && passedBoundary) {
        m_state.complete = true;
        m_state.snapshot.levelProgress = 100.0f;
    }

    return observe();
}

void ShadowUniversalOracle::discard(UniversalToken token) {
    m_snapshots.erase(token);
}

UniversalCanonicalState ShadowUniversalOracle::canonical(State const& state) const {
    UniversalCanonicalState out{};
    auto const& p = state.snapshot.player;
    out.words = {
        bits(p.x),
        bits(p.y),
        bits(p.velocityX),
        bits(p.velocityY),
        bits(p.gravity),
        bits(p.gravityModifier),
        bits(p.jumpVelocity),
        bits(p.speed),
        static_cast<std::uint64_t>(p.mode),
        static_cast<std::uint64_t>(p.mini),
        static_cast<std::uint64_t>(p.grounded),
        static_cast<std::uint64_t>(p.upsideDown),
        static_cast<std::uint64_t>(state.holding),
        static_cast<std::uint64_t>(p.dead),
        state.shadowTick,
    };

    std::uint64_t lo = 0x534841444f575631ULL;
    std::uint64_t hi = 0x554e495645525341ULL;
    for (std::size_t i = 0; i < out.words.size(); ++i) {
        lo = mix64(lo ^ out.words[i]);
        hi = mix64(hi ^ out.words[out.words.size() - 1U - i] ^ static_cast<std::uint64_t>(i));
    }
    out.hash = {lo, hi};
    out.completeRepresentation = true;
    return out;
}

std::optional<UniversalCanonicalState> ShadowUniversalOracle::canonicalState(
    UniversalToken token
) const {
    const auto it = m_snapshots.find(token);
    if (it == m_snapshots.end()) return std::nullopt;
    return canonical(it->second);
}

std::uint64_t ShadowUniversalOracle::decisionEpoch() const {
    if (!m_ready) return 0;
    auto const& p = m_state.snapshot.player;
    std::uint64_t value = static_cast<std::uint64_t>(p.mode);
    value |= static_cast<std::uint64_t>(p.grounded) << 8U;
    value |= static_cast<std::uint64_t>(p.upsideDown) << 9U;
    value |= static_cast<std::uint64_t>(p.dead) << 10U;
    return value;
}

} // namespace autobot::presolve
