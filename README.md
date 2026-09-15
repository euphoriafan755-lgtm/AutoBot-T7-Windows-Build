# AutoBot T7 V0.1 — Zero-Shot Core (first implementation delivery)

This snapshot intentionally implements only the foundation requested for the first delivery:

- Geode C++ project structure for Geometry Dash 2.2081 / Geode 5.10.1
- direct `GameStateReader`
- `GameSnapshot` / `PlayerState`
- in-game diagnostic HUD
- initial `LevelParser`
- independent `WorldObject` / `StaticWorld` copies

Not implemented yet in this snapshot:

- collision spatial hash
- physics validation harness
- Cube physics simulator
- trajectory generator
- Show Trajectory renderer
- planner / search
- realtime replanning
- tick-bound input execution
- AttemptBudgetManager
- FailureAnalyzer

The parser classifies game objects but classification is not a claim that every classified object is simulated yet.

## Build

Set `GEODE_SDK` to a Geode 5.10.1 SDK checkout/install, then:

```sh
cmake -S . -B build
cmake --build build --config RelWithDebInfo
```

or, with the Geode CLI configured:

```sh
geode build
```

## Verification caveat

`GameSnapshot::gameTick` is currently a monotonic `postUpdate` sample sequence. The HUD labels it explicitly as an unverified exact GD tick. Exact physics-tick synchronization is intentionally deferred until runtime timing validation, rather than being guessed.

The parser marks objects outside the declared Cube V0.1 subset as `V01Support::NotSupported`; classification never implies simulation support.
