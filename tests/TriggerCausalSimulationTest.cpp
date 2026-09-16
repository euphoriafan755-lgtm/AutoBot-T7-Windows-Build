#include "autobot/world/CollisionWorld.hpp"
#include "autobot/world/TriggerWorldModel.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using namespace autobot::world;

namespace {

WorldObject collider(int id, float x, float y, float w, float h, int group) {
    WorldObject o{};
    o.objectID = id;
    o.uniqueID = id;
    o.rawGameObjectType = 0;
    o.type = GameplayObjectType::Hazard;
    o.v01Support = V01Support::Supported;
    o.x = x;
    o.y = y;
    o.nodeX = x;
    o.nodeY = y;
    o.objectRect = {x, y, w, h};
    o.enabled = true;
    o.groupCount = 1;
    o.groups = {group};
    return o;
}

WorldObject trigger(int id, double x, TriggerDescriptor d, int ownGroup = 0) {
    WorldObject o{};
    o.objectID = id;
    o.uniqueID = id + 10000;
    o.type = GameplayObjectType::Decoration;
    o.v01Support = V01Support::NonGameplay;
    o.x = static_cast<float>(x);
    o.y = 0.0f;
    o.nodeX = o.x;
    o.nodeY = 0.0f;
    o.enabled = true;
    o.trigger = d;
    if (ownGroup > 0) {
        o.groupCount = 1;
        o.groups = {ownGroup};
    }
    return o;
}

struct Fixture {
    StaticWorld world;
    CollisionWorld collision;
    TriggerWorldModel model;

    void build() {
        world.parsed = true;
        assert(collision.build(world, 0.0));
        assert(model.build(world, collision));
    }
};

double centerY(WorldRect const& r) {
    return static_cast<double>(r.y) + static_cast<double>(r.height) * 0.5;
}

} // namespace

int main() {
    {
        Fixture f{};
        f.world.objects.push_back(collider(1, 100, 0, 20, 20, 10));
        TriggerDescriptor move{};
        move.kind = TriggerKind::Move;
        move.gameplayRelevant = true;
        move.targetGroupID = 10;
        move.durationSeconds = 1.0;
        move.moveY = 60.0;
        f.world.objects.push_back(trigger(901, 10, move));
        f.build();
        auto sim = f.model.createSimulation();
        const auto p = f.collision.primitiveForSource(0);
        const auto before = sim.predictPrimitive(p).bounds;
        sim.step(0, 15, 0, 0.5);
        const auto halfway = sim.predictPrimitive(p);
        assert(halfway.causal);
        assert(centerY(halfway.bounds) > centerY(before) + 20.0);
        sim.step(15, 16, 0, 0.5);
        const auto after = sim.predictPrimitive(p).bounds;
        assert(centerY(after) > centerY(before) + 55.0);
    }

    {
        Fixture f{};
        f.world.objects.push_back(collider(2, 100, 0, 20, 10, 10));
        TriggerDescriptor scale{};
        scale.kind = TriggerKind::Scale;
        scale.gameplayRelevant = true;
        scale.targetGroupID = 10;
        scale.scaleX = 2.0;
        scale.scaleY = 3.0;
        f.world.objects.push_back(trigger(2067, 10, scale));
        f.build();
        auto sim = f.model.createSimulation();
        const auto p = f.collision.primitiveForSource(0);
        sim.step(0, 15, 0, 0.01);
        auto scaled = sim.predictPrimitive(p);
        assert(scaled.bounds.width > 39.0f);
        assert(scaled.bounds.height > 29.0f);
        assert(scaled.uncertain);
    }

    {
        Fixture f{};
        f.world.objects.push_back(collider(3, 100, 0, 20, 10, 10));
        f.world.objects.push_back(collider(4, 50, 50, 10, 10, 30));
        TriggerDescriptor rotate{};
        rotate.kind = TriggerKind::Rotate;
        rotate.gameplayRelevant = true;
        rotate.targetGroupID = 10;
        rotate.centerGroupID = 30;
        rotate.rotationDegrees = 90.0;
        f.world.objects.push_back(trigger(1346, 10, rotate));
        f.build();
        auto sim = f.model.createSimulation();
        const auto p = f.collision.primitiveForSource(0);
        const auto before = sim.predictPrimitive(p).bounds;
        sim.step(0, 15, 0, 0.01);
        const auto after = sim.predictPrimitive(p);
        assert(std::abs(after.bounds.x - before.x) > 1.0f || std::abs(after.bounds.y - before.y) > 1.0f);
        assert(after.uncertain);
    }

    {
        Fixture f{};
        f.world.objects.push_back(collider(5, 100, 0, 20, 20, 10));
        TriggerDescriptor nestedToggle{};
        nestedToggle.kind = TriggerKind::Toggle;
        nestedToggle.gameplayRelevant = true;
        nestedToggle.spawnTriggered = true;
        nestedToggle.targetGroupID = 10;
        nestedToggle.toggleOn = false;
        f.world.objects.push_back(trigger(1049, 200, nestedToggle, 20));
        TriggerDescriptor spawn{};
        spawn.kind = TriggerKind::Spawn;
        spawn.gameplayRelevant = true;
        spawn.targetGroupID = 20;
        spawn.spawnDelaySeconds = 0.0;
        f.world.objects.push_back(trigger(1268, 10, spawn));
        f.build();
        auto sim = f.model.createSimulation();
        const auto p = f.collision.primitiveForSource(0);
        sim.step(0, 15, 0, 0.01);
        auto predicted = sim.predictPrimitive(p);
        assert(predicted.available && !predicted.enabled && predicted.causal);
    }

    {
        Fixture f{};
        f.world.objects.push_back(collider(6, 100, 0, 20, 20, 10));
        TriggerDescriptor followY{};
        followY.kind = TriggerKind::FollowPlayerY;
        followY.gameplayRelevant = true;
        followY.targetGroupID = 10;
        followY.durationSeconds = 10.0;
        followY.followYSpeed = 10.0;
        f.world.objects.push_back(trigger(1814, 10, followY));
        TriggerDescriptor timeWarp{};
        timeWarp.kind = TriggerKind::TimeWarp;
        timeWarp.gameplayRelevant = true;
        timeWarp.timeWarp = 0.5;
        f.world.objects.push_back(trigger(1935, 20, timeWarp));
        TriggerDescriptor gravity{};
        gravity.kind = TriggerKind::Gravity;
        gravity.gameplayRelevant = true;
        gravity.gravityValue = -1.0;
        f.world.objects.push_back(trigger(2066, 30, gravity));
        f.build();
        auto sim = f.model.createSimulation();
        const auto p = f.collision.primitiveForSource(0);
        const auto initialY = centerY(sim.predictPrimitive(p).bounds);
        sim.step(0, 15, 100, 0.1);
        assert(centerY(sim.predictPrimitive(p).bounds) > initialY + 5.0);
        auto timeEffects = sim.step(15, 25, 100, 0.1);
        assert(std::abs(timeEffects.timeScale - 0.5) < 0.001);
        auto gravityEffects = sim.step(25, 35, 100, 0.1);
        assert(gravityEffects.gravityChanged);
        assert(std::abs(gravityEffects.gravityValue + 1.0) < 0.001);
    }

    std::cout << "TRIGGER_CAUSAL_SIMULATION_TEST=PASS\n";
    return 0;
}
