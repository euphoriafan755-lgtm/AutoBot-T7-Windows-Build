#include "autobot/world/TriggerWorldModel.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace autobot::world {
namespace {

struct RectEdges {
    double left;
    double right;
    double bottom;
    double top;
};

RectEdges edges(WorldRect const& r) {
    const double x2 = static_cast<double>(r.x + r.width);
    const double y2 = static_cast<double>(r.y + r.height);
    return {
        std::min<double>(r.x, x2), std::max<double>(r.x, x2),
        std::min<double>(r.y, y2), std::max<double>(r.y, y2),
    };
}

WorldRect lerpRect(WorldRect const& a, WorldRect const& b, double t) {
    const auto mix = [t](double x, double y) { return x + (y - x) * t; };
    return {
        static_cast<float>(mix(a.x, b.x)),
        static_cast<float>(mix(a.y, b.y)),
        static_cast<float>(mix(a.width, b.width)),
        static_cast<float>(mix(a.height, b.height)),
    };
}

WorldRect moved(WorldRect r, double dx, double dy) {
    r.x = static_cast<float>(static_cast<double>(r.x) + dx);
    r.y = static_cast<float>(static_cast<double>(r.y) + dy);
    return r;
}

WorldRect rotateBounds(WorldRect const& base, double pivotX, double pivotY, double radians) {
    const auto e = edges(base);
    const double cx = (e.left + e.right) * 0.5;
    const double cy = (e.bottom + e.top) * 0.5;
    const double relX = cx - pivotX;
    const double relY = cy - pivotY;
    const double c = std::cos(radians);
    const double s = std::sin(radians);
    const double outCx = pivotX + relX * c - relY * s;
    const double outCy = pivotY + relX * s + relY * c;
    const double w = e.right - e.left;
    const double h = e.top - e.bottom;
    const double outW = std::abs(w * c) + std::abs(h * s);
    const double outH = std::abs(w * s) + std::abs(h * c);
    return {
        static_cast<float>(outCx - outW * 0.5),
        static_cast<float>(outCy - outH * 0.5),
        static_cast<float>(outW),
        static_cast<float>(outH),
    };
}

WorldRect scaleBounds(
    WorldRect const& base,
    double pivotX,
    double pivotY,
    double sx,
    double sy,
    bool ownCenter
) {
    const auto e = edges(base);
    const double cx = (e.left + e.right) * 0.5;
    const double cy = (e.bottom + e.top) * 0.5;
    const double pX = ownCenter ? cx : pivotX;
    const double pY = ownCenter ? cy : pivotY;
    const double outCx = pX + (cx - pX) * sx;
    const double outCy = pY + (cy - pY) * sy;
    const double outW = std::max(0.01, (e.right - e.left) * std::abs(sx));
    const double outH = std::max(0.01, (e.top - e.bottom) * std::abs(sy));
    return {
        static_cast<float>(outCx - outW * 0.5),
        static_cast<float>(outCy - outH * 0.5),
        static_cast<float>(outW),
        static_cast<float>(outH),
    };
}

} // namespace

void TriggerWorldModel::reset() {
    m_ready = false;
    m_triggers.clear();
    m_initialPrimitives.clear();
    m_groupPrimitives.clear();
    m_groupTriggers.clear();
    m_gameplayTriggerCount = 0;
}

bool TriggerWorldModel::build(
    StaticWorld const& source,
    CollisionWorld const& collisionWorld
) {
    reset();
    if (!source.parsed || !collisionWorld.ready()) return false;

    m_initialPrimitives = collisionWorld.primitives();
    for (std::size_t sourceIndex = 0; sourceIndex < source.objects.size(); ++sourceIndex) {
        auto const& object = source.objects[sourceIndex];
        const auto primitiveIndex = collisionWorld.primitiveForSource(sourceIndex);
        if (primitiveIndex != kInvalidPrimitiveIndex) {
            for (int group : object.groups) {
                if (group > 0) m_groupPrimitives[group].push_back(primitiveIndex);
            }
        }

        if (!object.trigger.gameplayRelevant) continue;
        TriggerEntry entry{};
        entry.sourceIndex = sourceIndex;
        entry.x = object.x;
        entry.groups = object.groups;
        entry.descriptor = object.trigger;
        entry.enabled = object.enabled && !object.groupDisabled;
        const auto triggerIndex = m_triggers.size();
        m_triggers.push_back(std::move(entry));
        ++m_gameplayTriggerCount;
        for (int group : object.groups) {
            if (group > 0) m_groupTriggers[group].push_back(triggerIndex);
        }
    }

    for (auto& [group, indices] : m_groupPrimitives) {
        (void)group;
        std::sort(indices.begin(), indices.end());
        indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
    }
    for (auto& [group, indices] : m_groupTriggers) {
        (void)group;
        std::sort(indices.begin(), indices.end());
        indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
    }

    m_ready = true;
    return true;
}

TriggerWorldModel::Simulation::Simulation(TriggerWorldModel const* model)
  : m_model(model) {
    if (!m_model || !m_model->ready()) return;
    m_primitiveStates.reserve(m_model->m_initialPrimitives.size());
    for (auto const& primitive : m_model->m_initialPrimitives) {
        PrimitiveState state{};
        state.bounds = primitive.broadphaseBounds;
        state.enabled = primitive.enabled;
        m_primitiveStates.push_back(state);
    }
    m_fired.assign(m_model->m_triggers.size(), false);
}

bool TriggerWorldModel::Simulation::crossed(double a, double b, double x) const {
    if (b >= a) return x > a && x <= b;
    return x < a && x >= b;
}

std::pair<double, double> TriggerWorldModel::Simulation::groupCenter(int groupID) const {
    if (!m_model || groupID <= 0) return {0.0, 0.0};
    auto it = m_model->m_groupPrimitives.find(groupID);
    if (it == m_model->m_groupPrimitives.end() || it->second.empty()) return {0.0, 0.0};
    double sx = 0.0;
    double sy = 0.0;
    std::size_t n = 0;
    for (auto primitiveIndex : it->second) {
        if (primitiveIndex >= m_primitiveStates.size()) continue;
        auto const& r = m_primitiveStates[primitiveIndex].bounds;
        const auto e = edges(r);
        sx += (e.left + e.right) * 0.5;
        sy += (e.bottom + e.top) * 0.5;
        ++n;
    }
    if (n == 0) return {0.0, 0.0};
    return {sx / static_cast<double>(n), sy / static_cast<double>(n)};
}

void TriggerWorldModel::Simulation::activateGroup(int groupID, double delaySeconds) {
    if (!m_model || groupID <= 0) return;
    auto it = m_model->m_groupTriggers.find(groupID);
    if (it == m_model->m_groupTriggers.end()) return;
    for (auto triggerIndex : it->second) {
        if (triggerIndex >= m_fired.size()) continue;
        m_scheduled.push_back({triggerIndex, m_elapsedSeconds + std::max(0.0, delaySeconds)});
    }
}

void TriggerWorldModel::Simulation::activateTrigger(
    std::size_t triggerIndex,
    TriggerFrameEffects& effects
) {
    if (!m_model || triggerIndex >= m_model->m_triggers.size()) return;
    auto const& entry = m_model->m_triggers[triggerIndex];
    auto const& d = entry.descriptor;
    if (!entry.enabled || !d.gameplayRelevant) return;
    if (m_fired[triggerIndex] && !d.multiTriggered) return;
    m_fired[triggerIndex] = true;
    ++effects.activatedTriggers;
    m_uncertain = m_uncertain || d.uncertain;

    auto groupPrimitives = [&](int groupID) -> std::vector<std::size_t> {
        auto it = m_model->m_groupPrimitives.find(groupID);
        return it == m_model->m_groupPrimitives.end() ? std::vector<std::size_t>{} : it->second;
    };

    switch (d.kind) {
        case TriggerKind::Toggle: {
            for (auto primitiveIndex : groupPrimitives(d.targetGroupID)) {
                if (primitiveIndex >= m_primitiveStates.size()) continue;
                auto& state = m_primitiveStates[primitiveIndex];
                state.enabled = d.toggleOn;
                state.causal = true;
            }
            break;
        }
        case TriggerKind::Spawn:
            activateGroup(d.targetGroupID, d.spawnDelaySeconds);
            break;
        case TriggerKind::Stop: {
            if (d.commandMode == TriggerCommandMode::Pause) {
                m_pausedGroups.insert(d.targetGroupID);
            } else if (d.commandMode == TriggerCommandMode::Resume) {
                m_pausedGroups.erase(d.targetGroupID);
            } else {
                for (auto& command : m_commands) {
                    if (command.targetGroup == d.targetGroupID) command.done = true;
                }
                m_pausedGroups.erase(d.targetGroupID);
            }
            break;
        }
        case TriggerKind::TimeWarp:
            m_timeScale = std::clamp(d.timeWarp, 0.05, 10.0);
            break;
        case TriggerKind::Gravity:
            m_gravityValue = d.gravityValue;
            m_gravityChanged = true;
            effects.gravityChanged = true;
            effects.gravityValue = m_gravityValue;
            break;
        case TriggerKind::Move:
        case TriggerKind::Rotate:
        case TriggerKind::Scale:
        case TriggerKind::Follow:
        case TriggerKind::FollowPlayerY: {
            ActiveCommand command{};
            command.targetGroup = d.targetGroupID;
            command.centerGroup = d.centerGroupID;
            command.duration = std::max(0.0, d.durationSeconds);
            command.descriptor = d;
            command.primitiveIndices = groupPrimitives(d.targetGroupID);
            command.baseBounds.reserve(command.primitiveIndices.size());
            for (auto primitiveIndex : command.primitiveIndices) {
                if (primitiveIndex < m_primitiveStates.size()) {
                    command.baseBounds.push_back(m_primitiveStates[primitiveIndex].bounds);
                    m_primitiveStates[primitiveIndex].causal = true;
                } else {
                    command.baseBounds.push_back({});
                }
            }
            switch (d.kind) {
                case TriggerKind::Move: command.kind = CommandKind::Move; break;
                case TriggerKind::Rotate: command.kind = CommandKind::Rotate; break;
                case TriggerKind::Scale: command.kind = CommandKind::Scale; break;
                case TriggerKind::Follow: command.kind = CommandKind::Follow; break;
                case TriggerKind::FollowPlayerY: command.kind = CommandKind::FollowPlayerY; break;
                default: break;
            }
            if (command.kind == CommandKind::Follow) {
                const auto center = groupCenter(d.centerGroupID);
                command.followCenterX = center.first;
                command.followCenterY = center.second;
                command.followCenterValid = true;
            }
            m_commands.push_back(std::move(command));
            break;
        }
        case TriggerKind::None:
            break;
    }
}

void TriggerWorldModel::Simulation::updateCommands(double playerY, double dtSeconds) {
    if (!m_model) return;
    constexpr double kPi = 3.14159265358979323846;

    for (auto& command : m_commands) {
        if (command.done) continue;
        if (m_pausedGroups.contains(command.targetGroup)) {
            command.paused = true;
            continue;
        }
        command.paused = false;

        const double previousElapsed = command.elapsed;
        const double effectiveDt = std::max(0.0, dtSeconds) * m_timeScale;
        command.elapsed += effectiveDt;
        const double duration = command.duration;
        const double previousT = duration <= 0.0 ? 0.0 : std::clamp(previousElapsed / duration, 0.0, 1.0);
        const double t = duration <= 0.0 ? 1.0 : std::clamp(command.elapsed / duration, 0.0, 1.0);

        if (command.kind == CommandKind::Move) {
            WorldRect offset{};
            offset.x = static_cast<float>(command.descriptor.moveX);
            offset.y = static_cast<float>(command.descriptor.moveY);
            for (std::size_t i = 0; i < command.primitiveIndices.size(); ++i) {
                const auto primitiveIndex = command.primitiveIndices[i];
                if (primitiveIndex >= m_primitiveStates.size()) continue;
                auto target = moved(command.baseBounds[i], offset.x, offset.y);
                m_primitiveStates[primitiveIndex].bounds = lerpRect(command.baseBounds[i], target, t);
            }
        } else if (command.kind == CommandKind::Rotate) {
            const auto pivot = groupCenter(command.centerGroup);
            const double radians = command.descriptor.rotationDegrees * t * kPi / 180.0;
            for (std::size_t i = 0; i < command.primitiveIndices.size(); ++i) {
                const auto primitiveIndex = command.primitiveIndices[i];
                if (primitiveIndex >= m_primitiveStates.size()) continue;
                auto const& base = command.baseBounds[i];
                const auto baseEdges = edges(base);
                const double ownX = (baseEdges.left + baseEdges.right) * 0.5;
                const double ownY = (baseEdges.bottom + baseEdges.top) * 0.5;
                const double px = command.centerGroup > 0 ? pivot.first : ownX;
                const double py = command.centerGroup > 0 ? pivot.second : ownY;
                m_primitiveStates[primitiveIndex].bounds = rotateBounds(base, px, py, radians);
                m_primitiveStates[primitiveIndex].uncertain = true; // AABB broadphase after rotation.
            }
        } else if (command.kind == CommandKind::Scale) {
            const auto pivot = groupCenter(command.centerGroup);
            const double finalScaleX = command.descriptor.divideScaleX
                ? 1.0 / std::max(0.001, std::abs(command.descriptor.scaleX))
                : command.descriptor.scaleX;
            const double finalScaleY = command.descriptor.divideScaleY
                ? 1.0 / std::max(0.001, std::abs(command.descriptor.scaleY))
                : command.descriptor.scaleY;
            const double sx = 1.0 + (finalScaleX - 1.0) * t;
            const double sy = 1.0 + (finalScaleY - 1.0) * t;
            for (std::size_t i = 0; i < command.primitiveIndices.size(); ++i) {
                const auto primitiveIndex = command.primitiveIndices[i];
                if (primitiveIndex >= m_primitiveStates.size()) continue;
                m_primitiveStates[primitiveIndex].bounds = scaleBounds(
                    command.baseBounds[i], pivot.first, pivot.second, sx, sy,
                    command.centerGroup <= 0
                );
                m_primitiveStates[primitiveIndex].uncertain = true;
            }
        } else if (command.kind == CommandKind::Follow) {
            const auto center = groupCenter(command.centerGroup);
            if (command.followCenterValid) {
                const double dx = (center.first - command.followCenterX) * command.descriptor.followXMod;
                const double dy = (center.second - command.followCenterY) * command.descriptor.followYMod;
                for (auto primitiveIndex : command.primitiveIndices) {
                    if (primitiveIndex >= m_primitiveStates.size()) continue;
                    m_primitiveStates[primitiveIndex].bounds = moved(
                        m_primitiveStates[primitiveIndex].bounds, dx, dy
                    );
                }
            }
            command.followCenterX = center.first;
            command.followCenterY = center.second;
            command.followCenterValid = true;
        } else if (command.kind == CommandKind::FollowPlayerY) {
            const auto targetCenter = groupCenter(command.targetGroup);
            double dy = playerY + command.descriptor.followYOffset - targetCenter.second;
            const double gain = std::clamp(command.descriptor.followYSpeed * effectiveDt, 0.0, 1.0);
            dy *= gain;
            if (command.descriptor.followYMaxSpeed > 0.0) {
                const double maxDelta = command.descriptor.followYMaxSpeed * effectiveDt;
                dy = std::clamp(dy, -maxDelta, maxDelta);
            }
            if (command.elapsed >= command.descriptor.followYDelay) {
                for (auto primitiveIndex : command.primitiveIndices) {
                    if (primitiveIndex >= m_primitiveStates.size()) continue;
                    m_primitiveStates[primitiveIndex].bounds = moved(
                        m_primitiveStates[primitiveIndex].bounds, 0.0, dy
                    );
                }
            }
        }

        // Zero-duration commands complete immediately; Follow variants remain
        // active for their duration because their effect depends on future state.
        const bool persistentFollow = command.kind == CommandKind::Follow
            || command.kind == CommandKind::FollowPlayerY;
        if (duration <= 0.0 && !persistentFollow) command.done = true;
        else if (duration > 0.0 && t >= 1.0) command.done = true;
        (void)previousT;
    }
}

TriggerFrameEffects TriggerWorldModel::Simulation::step(
    double previousPlayerX,
    double currentPlayerX,
    double playerY,
    double dtSeconds
) {
    TriggerFrameEffects effects{};
    effects.timeScale = m_timeScale;
    effects.gravityValue = m_gravityValue;
    effects.gravityChanged = m_gravityChanged;
    if (!m_model || !m_model->ready()) return effects;

    for (std::size_t i = 0; i < m_model->m_triggers.size(); ++i) {
        auto const& trigger = m_model->m_triggers[i];
        auto const& descriptor = trigger.descriptor;
        if (!trigger.enabled || !descriptor.gameplayRelevant) continue;
        if (descriptor.spawnTriggered || descriptor.touchTriggered) continue;
        if (m_fired[i] && !descriptor.multiTriggered) continue;
        if (crossed(previousPlayerX, currentPlayerX, trigger.x)) {
            activateTrigger(i, effects);
        }
    }

    bool progressed = true;
    std::size_t guard = 0;
    while (progressed && guard++ < 64) {
        progressed = false;
        for (auto it = m_scheduled.begin(); it != m_scheduled.end();) {
            if (it->dueTime <= m_elapsedSeconds + 1e-9) {
                activateTrigger(it->triggerIndex, effects);
                it = m_scheduled.erase(it);
                progressed = true;
            } else {
                ++it;
            }
        }
    }
    if (guard >= 64) m_uncertain = true;

    updateCommands(playerY, dtSeconds);
    m_elapsedSeconds += std::max(0.0, dtSeconds) * m_timeScale;
    effects.timeScale = m_timeScale;
    effects.gravityValue = m_gravityValue;
    effects.gravityChanged = m_gravityChanged;
    effects.uncertain = m_uncertain;
    return effects;
}

TriggerPrimitivePrediction TriggerWorldModel::Simulation::predictPrimitive(
    std::size_t primitiveIndex
) const {
    TriggerPrimitivePrediction result{};
    if (!m_model || primitiveIndex >= m_primitiveStates.size()) return result;
    auto const& state = m_primitiveStates[primitiveIndex];
    result.bounds = state.bounds;
    result.available = true;
    result.enabled = state.enabled;
    result.causal = state.causal;
    result.uncertain = state.uncertain || m_uncertain;
    return result;
}

} // namespace autobot::world
