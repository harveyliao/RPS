## Living RPS — Working Init (Bevy/ECS, file-config)

This document captures the current, working implementation of the "Living Rock–Paper–Scissors" simulation after initial development and debugging. It updates the original design to match what is actually built in `src/main.rs` using Bevy 0.16 and Rust 2024, without any in-game UI. All simulation parameters are configured via a TOML file.

### Platform & Engine
- **Engine**: Bevy 0.16 (Rust 2024, ECS)
- **Update model**: Fixed timestep at 60 Hz (`Time::<Fixed>::from_hz(60.0)`)
- **Rendering**: 2D, sprites as colored circles (emoji/textures can be swapped later)
- **Config**: File-based (`config.toml`), no egui UI

### Entities & Components
Each agent is an entity with:
- **`Agent { kind: Kind }`**: `Kind` ∈ { Rock, Paper, Scissors }
- **`Transform`**: position
- **`Velocity(Vec2)`**: linear velocity vector
- **`DesiredDir(Vec2)`**: desired heading updated every physics tick
- **`Radius(f32)`**: circle radius
- **`Speed(f32)`**: constant-speed marker (present but not currently read directly; speed comes from `SimConfig`)
- **`Collider`**: marker for collision logic

### Resources
- **`SimConfig`** (loaded from `config.toml` on startup):
  - `mode`: `Blend` | `Dominant`
  - `speed`: constant speed for all agents
  - `vision_radius`: retained for compatibility; currently ignored due to global vision (see below)
  - `threat_radius`: used only in `Dominant` mode for flee priority
  - `sep_radius`: radius for same-kind separation
  - `arena_size`: logical width/height
  - `entity_radius`: sprite circle radius
  - `initial_count`: coerced to multiple of 3, minimum 9
  - `cell_size`: spatial hash cell size (still used by collision; not used by steering when global vision is enabled)
- **`Weights`**: steering weights used in `Blend` mode (`chase`, `flee`, `separation`)
- **`RunState`**: `Running` | `Finished(Kind)` when a single kind dominates
- **`SpatialHash`**: uniform grid used for collision broadphase

### Systems & Order (FixedUpdate)
1. `spatial_hash_update_system`
2. `steering_system`
3. `movement_system`
4. `boundary_system`
5. `collision_transform_system`
6. `same_kind_nonoverlap_system` (only active in `Dominant` mode)
7. `winner_check_system`

Startup systems:
- `setup_camera`
- `initial_spawn`

### Current Mechanics
- **Chase / Flee**:
  - Rock chases Scissors, flees Paper
  - Scissors chases Paper, flees Rock
  - Paper chases Rock, flees Scissors
- **Global vision (implemented)**:
  - Every agent considers all other agents globally for nearest prey/predator selection.
  - As a result, `vision_radius` is not used for chase/flee in the current build.
- **Modes**:
  - `Blend`: desired direction is a weighted blend of chase, flee, and same-kind separation.
  - `Dominant`: priority logic — flee if a predator is within `threat_radius`, else chase prey if any, else separation.
- **Separation**:
  - Same-kind separation within `sep_radius` to reduce crowding; occasional overlaps are acceptable.
- **Boundary behavior**:
  - Agents bounce off arena edges by reflecting velocity, and the desired direction is also reflected inward to reduce corner clustering.
- **Collision & Transform**:
  - When two different kinds touch (center distance ≤ sum of radii), apply Rock–Paper–Scissors:
    - Rock > Scissors, Scissors > Paper, Paper > Rock.
  - The loser transforms into the winner’s kind. Velocity/direction are re-derived next tick via steering.
- **Same-kind non-overlap in Dominant mode** (implemented):
  - After collisions, overlapping same-kind agents are separated by equal and opposite offsets to resolve overlaps.
- **End condition**:
  - When only one kind remains, `RunState` switches to `Finished(kind)`, effectively stopping steering/movement logic.

### Initialization & Spawn
- **Balanced counts**: `initial_count` is coerced to a multiple of 3 and split evenly across kinds.
- **Placement**: grid-with-jitter across the arena bounding box.
- **Improved mixing** (implemented): kinds are assigned from a balanced, shuffled pool to avoid horizontal banding and produce a more uniform initial mix.

### Configuration File (`config.toml`)
Edit once before running. Example with commonly tuned fields:

```toml
mode = "Dominant"            # "Blend" or "Dominant"
speed = 140.0
vision_radius = 160.0         # currently ignored (global vision)
threat_radius = 120.0
sep_radius = 45.0
arena = [1000.0, 700.0]
entity_radius = 10.0
initial_count = 60            # coerced to multiple of 3; min 9
cell_size = 100.0             # affects spatial hash for collisions

[weights]                     # only used in Blend mode
chase = 1.0
flee = 1.2
separation = 0.35
```

### Running
- Use `run.ps1`, which calls `cargo run` (dynamic linking feature for Bevy).
- There is no in-game UI; change parameters via `config.toml` and re-run.

### Performance Notes
- Steering is currently O(n²) due to global vision. This is fine for ~50–100 entities. For larger counts, we can reintroduce spatially-limited vision using the existing `SpatialHash` to keep neighbor queries O(n).
- Keep `cell_size` approximately max of `sep_radius` and contact radius to maintain efficient collision passes.

### Future Enhancements (optional)
- Toggle between global vision and hashed neighbor vision via config.
- Use emojis or textured sprites instead of colored circles.
- Short flash on transform to visualize conversions.
- Add small cross-kind separation at very short ranges to further reduce local clustering.

### Summary of Changes from Original Description
- Removed in-game UI; all configuration comes from `config.toml`.
- Implemented global vision for chase/flee (ignores `vision_radius`).
- Improved initial distribution by shuffling kinds to avoid layered bands.
- Boundary reflections also update desired heading to reduce corner clumping.
- Added same-kind non-overlap resolution in `Dominant` mode.

