#pragma once

#include "autobot/world/CollisionWorld.hpp"
#include "autobot/world/WorldObject.hpp"

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autobot::world {

struct TriggerPrimitivePrediction {
    WorldRect bounds{};
    bool available = false;
    bool enabled = true;
    bool causal = false;
    bool uncertain = false;
};

struct TriggerFrameEffects {
    double timeScale = 1.0;
    double gravityValue = 1.0;
    bool gravityChanged = false;
    bool uncertain = false;
    std::size_t activatedTriggers = 0;
};

class TriggerWorldModel final {
public:
    struct TriggerEntry {
        std::size_t sourceIndex = 0;
        double x = 0.0;
        std::vector<int> groups;
        TriggerDescriptor descriptor{};
        bool enabled = true;
    };

    class Simulation final {
    public:
        Simulation() = default;

        TriggerFrameEffects step(
            double previousPlayerX,
            double currentPlayerX,
            double playerY,
            double dtSeconds
        );

        [[nodiscard]] TriggerPrimitivePrediction predictPrimitive(
            std::size_t primitiveIndex
        ) const;
        [[nodiscard]] double timeScale() const { return m_timeScale; }
        [[nodiscard]] bool uncertain() const { return m_uncertain; }

    private:
        friend class TriggerWorldModel;

        struct PrimitiveState {
            WorldRect bounds{};
            bool enabled = true;
            bool causal = false;
            bool uncertain = false;
        };

        enum class CommandKind { Move, Rotate, Scale, Follow, FollowPlayerY };
        struct ActiveCommand {
            CommandKind kind = CommandKind::Move;
            int targetGroup = 0;
            int centerGroup = 0;
            double duration = 0.0;
            double elapsed = 0.0;
            bool paused = false;
            bool done = false;
            TriggerDescriptor descriptor{};
            std::vector<std::size_t> primitiveIndices;
            std::vector<WorldRect> baseBounds;
            double followCenterX = 0.0;
            double followCenterY = 0.0;
            bool followCenterValid = false;
        };

        struct ScheduledTrigger {
            std::size_t triggerIndex = 0;
            double dueTime = 0.0;
        };

        explicit Simulation(TriggerWorldModel const* model);
        void activateTrigger(std::size_t triggerIndex, TriggerFrameEffects& effects);
        void activateGroup(int groupID, double delaySeconds);
        void updateCommands(double playerY, double dtSeconds);
        [[nodiscard]] std::pair<double, double> groupCenter(int groupID) const;
        [[nodiscard]] bool crossed(double a, double b, double x) const;

        TriggerWorldModel const* m_model = nullptr;
        std::vector<PrimitiveState> m_primitiveStates;
        std::vector<bool> m_fired;
        std::vector<ActiveCommand> m_commands;
        std::vector<ScheduledTrigger> m_scheduled;
        std::unordered_set<int> m_pausedGroups;
        double m_elapsedSeconds = 0.0;
        double m_timeScale = 1.0;
        double m_gravityValue = 1.0;
        bool m_gravityChanged = false;
        bool m_uncertain = false;
    };

    bool build(StaticWorld const& source, CollisionWorld const& collisionWorld);
    void reset();

    [[nodiscard]] bool ready() const { return m_ready; }
    [[nodiscard]] std::size_t triggerCount() const { return m_triggers.size(); }
    [[nodiscard]] std::size_t gameplayTriggerCount() const { return m_gameplayTriggerCount; }
    [[nodiscard]] Simulation createSimulation() const { return Simulation(this); }

private:
    bool m_ready = false;
    std::vector<TriggerEntry> m_triggers;
    std::vector<CollisionPrimitive> m_initialPrimitives;
    std::unordered_map<int, std::vector<std::size_t>> m_groupPrimitives;
    std::unordered_map<int, std::vector<std::size_t>> m_groupTriggers;
    std::size_t m_gameplayTriggerCount = 0;
};

} // namespace autobot::world
