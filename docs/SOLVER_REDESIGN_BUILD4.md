# AutoBot T7 — Build #4 solver redesign

## Architecture decision

Build #4 replaces the active AutoBot solving path instead of extending the old
RealtimePlanner / GlobalPlanner / UniversalSearch stack.

The executable solver is adapted from **gdsolver/gdsolver**, pinned to:

`4fb6a2365d3623b211913a5a21de308e49c5fce2`

The old AutoBot sources remain in the repository for audit/history but are not
compiled into the AutoBotT7 target. There is one gameplay input authority.

## Why ADAPT / PORT

The pinned solver already implements the architecture required for Build #4:

- layered reachability search in physics-tick space;
- exact representative states with quantized search keys used for dedup only;
- candidate reconstruction into deterministic input edges;
- real Geometry Dash replay as the authority/verifier;
- run-local divergence capture, re-anchor and suffix repair;
- plan rejoin and adaptive solve horizons;
- dynamic geometry / trigger / mode state carried by the model;
- final verified replay driven only by legal gameplay input.

AutoBot keeps its own package identity while compiling the pinned solver source
directly into the mod.

## Local adaptations

Only integration/identity defaults are changed during configure:

1. UI mode defaults to **Solve** rather than Normal, so entering a level starts
   a cold solve without user gameplay input.
2. The panel label is changed from GDSOLVER to AUTOBOT T7.
3. AutoBot's own `mod.json` and package id remain `thiago7tv.autobot-t7`.

No level-specific click table, percentage rule, x-position input table or
preloaded Stereo Madness solution is added.

## Licensing

gdsolver is MIT licensed. The required copyright and MIT text are retained in
`THIRD_PARTY_GDSOLVER_LICENSE.txt`.

Secondary projects were studied for integration and replay ideas but their code
is not copied into AutoBot T7. In particular, code without a redistribution
license is not imported.

## Active flow

```text
loaded Geometry Dash level
→ export gameplay-relevant level model from the real PlayLayer
→ layered reachability search (shadow/DP only)
→ candidate physical-tick input plan
→ replay one candidate in real Geometry Dash
→ compare/observe the real run
→ death/divergence: keep verified prefix + capture real anchor
→ solve/repair suffix (+ rejoin when valid)
→ repeat until real GD clears alive
→ replay the verified final plan visually
```

Search branching does not call reversible PlayLayer/GJBaseGameLayer updates.
The game is used to verify a candidate plan, not as a branch oracle.
