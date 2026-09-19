# AutoBot T7

AutoBot T7 is my Geometry Dash autoplay / solver project built as a Geode mod.

The idea is simple: instead of storing a click pattern for one level, the bot reads the level, builds a world model, searches for a route before the run starts and then plays that solution back in the real game.

I'm building it mostly to see how far I can push the idea.

## Current build

- Version: **0.1.1**
- Geometry Dash: **2.2081 (Windows)**
- Geode: **5.10.1**
- Current runtime target: **Stereo Madness**

The 0.1.1 build comes from GitHub Actions and is also the build submitted to the Geode Index.

## What it does right now

When a level starts, AutoBot T7 can:

1. parse the level
2. build its collision/world state
3. search while the visible player stays frozen
4. verify the solution before starting the run
5. replay the result through the normal game input path

The current work is focused on making that whole path work reliably on Stereo Madness from 0% to 100%.

If the search is running correctly, the runtime debug info should show real search activity such as changing frontier size, depth and expansion count.

## Project layout

`src/presolve` — pre-run search and replay policy  
`src/world` — parsed level and collision state  
`src/solver` — planning, simulation and action generation  
`src/control` — playback/input side  
`tests` — solver and world-model regression tests

## Status

This is still an experimental project. The CI tests cover a lot of the solver pieces, but passing CI is not the same thing as completing a level in the real game.

So for now the test that matters is pretty straightforward:

**Stereo Madness → 100%.**

Once that works consistently, I'll move on from there.

— Thiago7TV
