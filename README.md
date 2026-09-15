# AutoBot T7 — V0.1 Zero-Shot Core

Current development gate: **Collision World audit / repair**.

Runtime-verified baseline remains frozen around:
- GameStateReader
- DiagnosticHUD
- initial LevelParser behavior
- PlayLayer postUpdate hook

Collision World work in this gate adds:
- copied/stable collision identities (no `GameObject*` persistence)
- SpatialHash broad-phase indexing
- detailed query-stage diagnostics
- CollisionTrace end-to-end pipeline tracing
- classification/consistency audits
- temporary collider/object-label overlay
- parse stability fingerprints

Important: `GameObject::getObjectRect()` is treated as **OBJECT BOUNDS / broad-phase geometry only**. Exact gameplay hitboxes remain **NOT VERIFIED** and are not invented in this gate.

No PhysicsEngine, trajectory generation, planner, or input controller is included in this gate.
