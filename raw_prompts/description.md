<user>
Help me create this game. Here's a description and detail of the game. Ask me clarifying question before start if necessary.

This game is a dynamic simulation based on the classic Rock-Paper-Scissors rules, but with sophisticated entity behaviors and a unique transformation mechanic. It begins with the title "Living RPS Game". Initially, the screen is populated with a mix of three distinct entity types: **Rocks**, **Paper**, and **Scissors**.

The entities exhibit **predator-prey dynamics** and **boid-like separation**:
* **Scissors actively chase Paper** (their prey) while simultaneously **fleeing from Rocks** (their predators).
* **Paper actively chases Rock** while **fleeing from Scissors**.
* **Rock actively chases Scissors** while **fleeing from Paper**.
* Entities of the **same type avoid colliding with each other**, maintaining a certain separation distance, similar to the "separation" rule in boid simulations, preventing clumping.

The core difference is in the interaction outcome: when one entity type eliminates another, **the eliminated entity transforms into the type of its predator**. For example, if a Scissor encounters a Paper, the Paper entity doesn't just disappear; it turns into a Scissor.

The simulation progresses until one entity type dominates the entire screen, having transformed all other opposing types into its own kind.

---

## Recreate the "Living Rock-Paper-Scissors" Game

Design and implement a visual simulation game based on the principles of Rock-Paper-Scissors, incorporating advanced movement behaviors and a type-transformation mechanic.

**Game Title:** Living RPS

**Credits:**
* **Creator:** Harvey

### Visual Elements:

For simplicity, entities can be represented as circles with distinct colors or their corresponding emojis:

1.  **Rock:** A grey circle or an emoji 🪨
2.  **Paper:** A brown circle or an emoji 📄
3.  **Scissors:** A red circle or an emoji ✂️

***

### Gameplay Mechanics:

1.  **Initial State:**
    * The game begins on a black background.
    * Randomly scatter an approximately equal number of Rock, Paper, and Scissors entities across the screen. The initial distribution should be somewhat dense but allow for movement and interaction.

2.  **Movement Mechanics (Inspired by Boids/Predator-Prey):**
    * All entities should exhibit continuous movement within the game area.
    * Movement is not purely random but influenced by other entities:
        * **Chasing Prey:** Each entity type should actively move towards and attempt to "chase" its specific "prey" type (e.g., Scissors chase Paper).
        * **Fleeing Predators:** Each entity type should actively move away from and "flee" its specific "predator" type (e.g., Scissors flee from Rock).
        * **Separation (Same Kind):** Entities of the same type should exhibit a separation behavior, attempting to avoid direct collisions with each other and maintain a small personal space, similar to the "separation" rule in boid simulations. This prevents them from clumping together excessively.
        * **Boundary Avoidance:** Entities should bounce off or turn away from the edges of the game screen.

3.  **Interaction and Transformation:**
    * When two different types of entities come into sufficient contact (or are within a very close proximity), the Rock-Paper-Scissors rules apply with a transformation outcome:
        * **Rock vs. Scissors:** If a Rock touches a Scissors, the **Scissors entity transforms into a Rock**. The original Rock entity remains.
        * **Scissors vs. Paper:** If a Scissors touches a Paper, the **Paper entity transforms into a Scissors**. The original Scissors entity remains.
        * **Paper vs. Rock:** If a Paper touches a Rock, the **Rock entity transforms into a Paper**. The original Paper entity remains.
    * The transformation should be visually represented (e.g., the visual sprite/color changes immediately or with a quick animation).
    * Entities of the same type that come into contact should not interact or transform, only apply the separation rule.

4.  **Dominance and Resolution:**
    * The simulation should continue as long as there are at least two different types of entities present.
    * Eventually, one type of entity will dominate by transforming all others into its own kind.
    * Once only one type of entity remains on the screen (all entities are of the same type), the simulation can continue with the surviving entities moving according to their internal rules (e.g., still applying separation), or it can enter a "stable state" where no further transformations occur.

***

### Technical Considerations (for implementation by a suitable environment/tool):

* **Visuals:** Simple, clear 2D graphics for the entities, with easy color/emoji switching for transformations.
* **Movement Logic:** Implement the chasing, fleeing, and separation behaviors using vectors or similar physics-based approaches. This involves calculating forces based on the positions and types of nearby entities.
* **Collision Detection:** Efficient method for detecting when entities touch or are close enough to interact. A simple circular bounding box collision can work.
* **Performance:** The simulation should run smoothly, even with a moderate number of entities (e.g., 50-100 initial entities), requiring optimized collision checks and movement updates.

### Desired Output:

A detailed conceptual outline or pseudo-code for implementing the game, focusing on the movement logic and the transformation rules. Explain how the "chase," "flee," and "separation" behaviors would be calculated and applied to each entity's movement vector. Provide example code snippets for key mechanics where appropriate. Emphasize the dynamic and emergent nature of the Rock-Paper-Scissors interactions driven by these behaviors and the unique transformation mechanic.

</user>

<assistant>
Got it — before I start designing the full outline and pseudo-code, I need to clarify a few details so we avoid ambiguity in how the simulation behaves and how we’ll implement it.

Here are my key clarifying questions:

---

### **1. Platform & Engine**

* Are you planning to build this in **Unity (C#)**, **Pygame (Python)**, or another framework? This affects how movement, collisions, and rendering are implemented.
* Do you want **frame-based updates** (typical in games) or **time-step-based physics**?

---

### **2. Representation of Entities**

* Do you want to **use emojis** (🪨 📄 ✂️) as sprites, or just **colored circles** for simplicity in the first version?
* Should the entity size be **uniform for all types** (e.g., all circles same radius) or can they vary?

---

### **3. Movement Parameters**

* How fast should entities move?

  * Constant speed for all types, or can speed differ between Rock/Paper/Scissors?
* For **chase/flee/separation**:

  * Do you want **vector blending** (e.g., sum of weighted chase, flee, separation vectors) or do you want one behavior to dominate depending on proximity (e.g., if predator is close, ignore chasing prey)?
* Should entities have **smooth turning** (boid-like) or instantly change direction toward their goal?

---

### **4. Collision & Transformation**

* What counts as "contact" — center-to-center distance ≤ 2×radius, or some other threshold?
* Should transformation happen **instantly upon contact** or after a very short “attack” delay?
* After transformation, should the new entity **inherit** the velocity/direction of the old one, or start fresh?

---

### **5. Separation Rule**

* How strong should the separation force be compared to chase/flee forces?
* Do you want same-type entities to **completely avoid touching** or just reduce crowding?

---

### **6. End Condition**

* When one type dominates:

  * Do we **stop the simulation** and show “Winner: Rock/Paper/Scissors”?
  * Or keep running endlessly with only separation movement?

---

### **7. Scale & Performance**

* Initial entity count: around **50–100** total, equally split?
* Should entity spawn positions be **completely random** or spaced out to avoid overlaps at start?

---

Once you answer these, I can give you:

* A **structured outline** of the game loop.
* **Pseudo-code** for movement (chase, flee, separation).
* **Collision and transformation logic** ready to implement in your chosen engine.

Which environment are you leaning toward — Unity or Pygame?
That will decide if I structure the pseudo-code closer to C#/MonoBehaviour or Python.

</assistant>



<user>
1. I want to try Bevy engine in Rust, with ECS. I'm intermediate in Rust. (long-term goal to put it in Bevy engine in Rust, with ECS). fame-based physics
2. colored circles for simplicity, but make sure the design allows me to swap it to emoji later. entitiy size should be uniform for all types. 
3. constant speed for all types. Provide switch for vector blending and domination. instant change direction
4. center-to-center distance ≤ 2×radius sounds fine. transformation happen instantly upon contact. velocity/direction start fresh
5. separation force is relatively small compared to chase/flee forces. just reduce crowding, Occational overlapping entities is acceptable.
6. stop the simulation and show “Winner: Rock/Paper/Scissors”"
7. provide a slider to adjust initial entity count (multiple of 3). equally split. spaced out spawn positions
</user>


<assistant>

Awesome — Bevy + ECS is a great fit. Below is a tight game design + implementation plan tailored to your choices, with Bevy-style pseudo/Rust that you can drop into a project and flesh out. I’ll keep things modular so you can later swap circles → emoji, add fancier UI, or optimize the neighbor queries.

---

# High-level architecture (ECS-first)

## Entities (all circles, uniform size)

* `Transform`, `Velocity`, `Speed`, `Radius`
* `Kind(Rock | Paper | Scissors)`
* `Collider` (marker)
* `Renderable` (circle; later swap to emoji)

## Resources / Assets

* `SimConfig` (tunable at runtime via UI)

  * `mode: SteeringMode` = `Blend` | `Dominant`
  * `speed: f32` (constant speed for all)
  * `vision_radius: f32` (how far to look for prey/predator)
  * `threat_radius: f32` (dominant mode threshold)
  * `sep_radius: f32` (same-type separation neighborhood)
  * `weights: Weights { chase, flee, separation }` (used if `Blend`)
  * `arena_size: Vec2` (logical bounds)
  * `entity_radius: f32`
  * `initial_count: usize` (multiple of 3; UI slider)
* `RunState` (`Running` | `Finished(Kind)`)
* `SpatialHash` (for fast neighbor queries; simple uniform grid)

## Systems (order matters)

1. `ui_system` (sliders, toggles, Reset/Start, show winner)
2. `spawn_system` (spaced-out spawn; runs on Reset or first run)
3. `spatial_hash_update_system`
4. `steering_system` (compute desired dir per entity)
5. `movement_system` (set velocity to `speed * desired_dir`)
6. `boundary_system` (bounce or turn away)
7. `collision_transform_system` (RPS resolve + transform)
8. `winner_check_system` (stop when only one Kind remains)

Use a `FixedUpdate` schedule for deterministic frame-based physics (e.g., 60 Hz).

---

# Core mechanics (math + rules)

## RPS transform rule

```
Rock beats Scissors
Scissors beats Paper
Paper beats Rock
```

When contact happens (center distance ≤ 2×radius), **the loser transforms into the winner’s type**. The winner remains as-is.

```rust
fn beats(a: Kind, b: Kind) -> bool {
    matches!(
        (a, b),
        (Kind::Rock, Kind::Scissors) |
        (Kind::Scissors, Kind::Paper) |
        (Kind::Paper, Kind::Rock)
    )
}
```

## Steering — your two modes

### Mode A: **Blend** (vector blending)

For each entity, find:

* `nearest_prey` (within `vision_radius`)
* `nearest_predator` (within `vision_radius`)
* `same_type_neighbors` (within `sep_radius`)

Compute direction vectors:

```text
dir_chase = normalize(prey.pos - me.pos)         (0 if none)
dir_flee  = -normalize(predator.pos - me.pos)    (0 if none)
sep_vec   = Σ_over_same (me.pos - n.pos) / dist^2   (0 if none); then normalize
desired   = w_chase*dir_chase + w_flee*dir_flee + w_sep*sep_vec
desired   = normalize(desired)   // if nonzero; else keep last dir or add tiny jitter
```

Instant turning: set velocity = `speed * desired`.

### Mode B: **Dominant** (priority logic)

```text
if nearest_predator within threat_radius:
    desired = dir_flee
else if nearest_prey within vision_radius:
    desired = dir_chase
else:
    desired = separation (if any) else keep current dir
```

## Boundary

Bounce: if `|x| > half_width - r` → invert `vx`; same for `y`.

---

# File layout

```
src/
  main.rs
  components.rs
  resources.rs
  systems/
    ui.rs
    spawn.rs
    spatial.rs
    steering.rs
    movement.rs
    boundary.rs
    collision.rs
    winner.rs
  util.rs
Cargo.toml
```

---

# Cargo.toml (suggested)

> Tip: bevy\_egui must match your Bevy version — check its README for the right pair, or use `cargo add bevy bevy_egui@<matching>`. Start simple; if UI versions fight you, use Bevy’s built-in UI first.

```toml
[package]
name = "living_rps"
version = "0.1.0"
edition = "2021"

[dependencies]
bevy = { version = "0.14", features = ["dynamic"] }     # or latest stable
bevy_egui = "0.x"                                       # match Bevy version
rand = "0.8"
```

---

# Components & resources (Rust)

```rust
// components.rs
use bevy::prelude::*;

#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub enum Kind { Rock, Paper, Scissors }

#[derive(Component)]
pub struct Agent {
    pub kind: Kind,
}

#[derive(Component, Deref, DerefMut, Default)]
pub struct Velocity(pub Vec2);

#[derive(Component)]
pub struct Speed(pub f32);

#[derive(Component)]
pub struct Radius(pub f32);

#[derive(Component)]
pub struct Collider; // marker

#[derive(Component)]
pub struct DesiredDir(pub Vec2); // computed each frame

// resources.rs
use bevy::prelude::*;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum SteeringMode { Blend, Dominant }

#[derive(Resource)]
pub struct Weights {
    pub chase: f32,
    pub flee: f32,
    pub separation: f32,
}

#[derive(Resource)]
pub struct SimConfig {
    pub mode: SteeringMode,
    pub speed: f32,
    pub vision_radius: f32,
    pub threat_radius: f32,
    pub sep_radius: f32,
    pub weights: Weights,
    pub arena_size: Vec2,
    pub entity_radius: f32,
    pub initial_count: usize, // multiple of 3
}

#[derive(Resource)]
pub enum RunState {
    Running,
    Finished(Kind),
}

// Very simple uniform grid spatial hash
#[derive(Resource, Default)]
pub struct SpatialHash {
    pub cell_size: f32,
    pub buckets: std::collections::HashMap<IVec2, Vec<Entity>>,
}
```

---

# App setup (main.rs)

```rust
use bevy::prelude::*;
use bevy::time::common_conditions::on_timer;
mod components; use components::*;
mod resources;  use resources::*;
mod systems;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        // fixed timestep for physics-y updates
        .insert_resource(Time::<Fixed>::from_hz(60.0))
        .insert_resource(SimConfig {
            mode: SteeringMode::Blend,
            speed: 120.0,
            vision_radius: 140.0,
            threat_radius: 100.0,
            sep_radius: 40.0,
            weights: Weights { chase: 1.0, flee: 1.6, separation: 0.3 },
            arena_size: Vec2::new(1200., 800.),
            entity_radius: 8.0,
            initial_count:  ninety_multiple_of_three( ninety:  ninety? ), // set in UI; seed with 60
        })
        .insert_resource(RunState::Running)
        .insert_resource(SpatialHash { cell_size: 80.0, ..Default::default() })
        .add_systems(Startup, systems::spawn::setup_camera)
        .add_systems(Startup, systems::spawn::initial_spawn)
        // UI every frame
        .add_systems(Update, systems::ui::ui_system)
        // Physics/sim on fixed step
        .add_systems(
            FixedUpdate,
            (
                systems::spatial::spatial_hash_update_system,
                systems::steering::steering_system,
                systems::movement::movement_system,
                systems::boundary::boundary_system,
                systems::collision::collision_transform_system,
                systems::winner::winner_check_system,
            ).chain()
        )
        .run();
}
```

*(Replace the placeholder initializer for `initial_count` — start with `60` and expose via UI.)*

---

# Key systems (snippets)

## Spaced-out spawn (grid jitter; equal split; no overlaps)

```rust
// systems/spawn.rs
use bevy::prelude::*;
use rand::Rng;
use crate::{SimConfig, Agent, Kind, Radius, Speed, Velocity, Collider};

pub fn setup_camera(mut commands: Commands) {
    commands.spawn(Camera2dBundle::default());
}

pub fn initial_spawn(
    mut commands: Commands,
    config: Res<SimConfig>,
) {
    let total = config.initial_count.max(3) / 3 * 3;
    let each = total / 3;
    let kinds = [Kind::Rock, Kind::Paper, Kind::Scissors];

    // Simple grid jitter placement
    let cols = (total as f32).sqrt().ceil() as i32;
    let rows = ((total as f32) / cols as f32).ceil() as i32;
    let cell_w = config.arena_size.x / cols as f32;
    let cell_h = config.arena_size.y / rows as f32;

    let mut rng = rand::thread_rng();
    let mut idx = 0;

    for r in 0..rows {
        for c in 0..cols {
            if idx >= total { break; }
            let k = if idx < each { kinds[0] }
                    else if idx < 2*each { kinds[1] }
                    else { kinds[2] };

            let jitter = Vec2::new(
                rng.gen_range(-0.3..0.3) * cell_w,
                rng.gen_range(-0.3..0.3) * cell_h,
            );

            let half = config.arena_size * 0.5;
            let pos = Vec3::new(
                -half.x + (c as f32 + 0.5) * cell_w + jitter.x,
                -half.y + (r as f32 + 0.5) * cell_h + jitter.y,
                0.0,
            );

            commands.spawn((
                SpriteBundle {
                    sprite: Sprite {
                        color: color_of(k),
                        custom_size: Some(Vec2::splat(config.entity_radius * 2.0)),
                        ..default()
                    },
                    transform: Transform::from_translation(pos),
                    ..default()
                },
                Agent { kind: k },
                Radius(config.entity_radius),
                Speed(config.speed),
                Velocity(random_dir() * config.speed),
                Collider,
            ));

            idx += 1;
        }
    }
}

fn color_of(k: Kind) -> Color {
    match k {
        Kind::Rock => Color::srgb(0.6, 0.6, 0.65),
        Kind::Paper => Color::srgb(0.7, 0.55, 0.35),
        Kind::Scissors => Color::srgb(0.9, 0.2, 0.2),
    }
}

fn random_dir() -> Vec2 {
    let a = rand::random::<f32>() * std::f32::consts::TAU;
    Vec2::new(a.cos(), a.sin())
}
```

> Later swap circles→emoji: replace `SpriteBundle` with a `Text2dBundle` or a textured quad; the rest (ECS logic) stays the same.

---

## Spatial hash (fast neighbor lookups)

```rust
// systems/spatial.rs
use bevy::prelude::*;
use std::collections::HashMap;
use crate::{SpatialHash, Radius};

pub fn spatial_hash_update_system(
    mut hash: ResMut<SpatialHash>,
    q: Query<(Entity, &Transform)>,
) {
    hash.buckets.clear();
    for (e, t) in q.iter() {
        let p = t.translation.truncate();
        let cell = IVec2::new(
            (p.x / hash.cell_size).floor() as i32,
            (p.y / hash.cell_size).floor() as i32,
        );
        hash.buckets.entry(cell).or_insert_with(Vec::new).push(e);
    }
}

// helper to iterate neighbor cells (current ±1 in both axes)
pub fn neighbor_cells(p: Vec2, cell_size: f32) -> [IVec2; 9] {
    let cx = (p.x / cell_size).floor() as i32;
    let cy = (p.y / cell_size).floor() as i32;
    let mut out = [IVec2::ZERO; 9];
    let mut i = 0;
    for dy in -1..=1 {
        for dx in -1..=1 {
            out[i] = IVec2::new(cx + dx, cy + dy);
            i += 1;
        }
    }
    out
}
```

---

## Steering (Blend & Dominant)

```rust
// systems/steering.rs
use bevy::prelude::*;
use crate::{
    components::{Agent, Kind, DesiredDir, Velocity, Speed, Radius},
    resources::{SimConfig, SpatialHash, SteeringMode},
};
use super::spatial::neighbor_cells;

pub fn steering_system(
    config: Res<SimConfig>,
    hash: Res<SpatialHash>,
    mut q: Query<(Entity, &mut DesiredDir, &Transform, &Agent)>,
    agents: Query<(&Transform, &Agent, &Radius)>,
) {
    let cell_size = hash.cell_size;

    for (_e, mut desired, tf, me) in q.iter_mut() {
        let pos = tf.translation.truncate();
        let mut nearest_prey: Option<(f32, Vec2)> = None;
        let mut nearest_pred: Option<(f32, Vec2)> = None;
        let mut sep_acc = Vec2::ZERO;

        let prey_kind = prey_of(me.kind);
        let pred_kind = predator_of(me.kind);

        for cell in neighbor_cells(pos, cell_size) {
            if let Some(bucket) = hash.buckets.get(&cell) {
                for &other_e in bucket {
                    if let Ok((otf, oagent, orad)) = agents.get(other_e) {
                        if other_e == _e { continue; }
                        let op = otf.translation.truncate();
                        let d = op - pos;
                        let dist2 = d.length_squared();
                        if dist2 == 0.0 { continue; }
                        let dist = dist2.sqrt();

                        if oagent.kind == prey_kind && dist <= config.vision_radius {
                            if nearest_prey.map_or(true, |(best, _)| dist < best) {
                                nearest_prey = Some((dist, d));
                            }
                        }
                        if oagent.kind == pred_kind && dist <= config.vision_radius {
                            if nearest_pred.map_or(true, |(best, _)| dist < best) {
                                nearest_pred = Some((dist, d));
                            }
                        }
                        if oagent.kind == me.kind && dist <= config.sep_radius {
                            // inverse square push
                            sep_acc += (pos - op) / (dist2 + 1.0);
                        }
                    }
                }
            }
        }

        let dir_chase = nearest_prey.map(|(_, d)| d.normalize()).unwrap_or(Vec2::ZERO);
        let dir_flee  = nearest_pred.map(|(_, d)| (-d).normalize()).unwrap_or(Vec2::ZERO);
        let dir_sep   = if sep_acc.length_squared() > 0.0 { sep_acc.normalize() } else { Vec2::ZERO };

        let d = match config.mode {
            SteeringMode::Blend => {
                let w = &config.weights;
                let v = w.chase*dir_chase + w.flee*dir_flee + w.separation*dir_sep;
                if v.length_squared() > 0.0001 { v.normalize() } else { Vec2::ZERO }
            }
            SteeringMode::Dominant => {
                if let Some((dist, d)) = nearest_pred {
                    if dist <= config.threat_radius {
                        (-d).normalize()
                    } else if let Some((_, dprey)) = nearest_prey {
                        dprey.normalize()
                    } else if dir_sep != Vec2::ZERO {
                        dir_sep
                    } else { Vec2::ZERO }
                } else if let Some((_, dprey)) = nearest_prey {
                    dprey.normalize()
                } else if dir_sep != Vec2::ZERO {
                    dir_sep
                } else { Vec2::ZERO }
            }
        };

        desired.0 = if d == Vec2::ZERO { desired.0 } else { d };
    }
}

fn prey_of(k: Kind) -> Kind {
    match k {
        Kind::Rock     => Kind::Scissors,
        Kind::Scissors => Kind::Paper,
        Kind::Paper    => Kind::Rock,
    }
}
fn predator_of(k: Kind) -> Kind {
    match k {
        Kind::Rock     => Kind::Paper,
        Kind::Scissors => Kind::Rock,
        Kind::Paper    => Kind::Scissors,
    }
}
```

---

## Movement (instant direction, constant speed)

```rust
// systems/movement.rs
use bevy::prelude::*;
use crate::{components::{DesiredDir, Velocity, Speed}};

pub fn movement_system(
    time: Res<Time>,
    mut q: Query<(&DesiredDir, &Speed, &mut Velocity, &mut Transform)>,
) {
    let dt = time.delta_seconds();
    for (desired, speed, mut vel, mut tf) in q.iter_mut() {
        if desired.0.length_squared() > 0.0 {
            vel.0 = desired.0 * speed.0;
        }
        tf.translation.x += vel.0.x * dt;
        tf.translation.y += vel.0.y * dt;
    }
}
```

---

## Boundary bounce

```rust
// systems/boundary.rs
use bevy::prelude::*;
use crate::{resources::SimConfig, components::{Velocity, Radius}};

pub fn boundary_system(
    config: Res<SimConfig>,
    mut q: Query<(&mut Transform, &mut Velocity, &Radius)>,
) {
    let half = config.arena_size * 0.5;

    for (mut tf, mut vel, r) in q.iter_mut() {
        let mut p = tf.translation.truncate();
        let rr = r.0;

        if p.x < -half.x + rr {
            p.x = -half.x + rr;
            vel.0.x = vel.0.x.abs();
        } else if p.x > half.x - rr {
            p.x = half.x - rr;
            vel.0.x = -vel.0.x.abs();
        }
        if p.y < -half.y + rr {
            p.y = -half.y + rr;
            vel.0.y = vel.0.y.abs();
        } else if p.y > half.y - rr {
            p.y = half.y - rr;
            vel.0.y = -vel.0.y.abs();
        }
        tf.translation.x = p.x;
        tf.translation.y = p.y;
    }
}
```

---

## Collision + transform (instant on contact)

To avoid double-processing in a frame, we’ll collect a list of `(winner, loser)` and then apply.

```rust
// systems/collision.rs
use bevy::prelude::*;
use std::collections::HashSet;
use crate::{
    components::{Agent, Kind, Radius},
    resources::{SpatialHash, SimConfig},
};
use super::spatial::neighbor_cells;

pub fn collision_transform_system(
    config: Res<SimConfig>,
    hash: Res<SpatialHash>,
    mut q: Query<(Entity, &Transform, &mut Agent, &Radius)>,
    mut sprites: Query<&mut Sprite>, // to recolor on transform
) {
    let mut to_convert: Vec<(Entity, Kind)> = Vec::new();
    let mut seen_pairs: HashSet<(Entity, Entity)> = HashSet::new();

    for (e, tf, agent, r) in q.iter_mut() {
        let pos = tf.translation.truncate();
        let rr = r.0;
        for cell in neighbor_cells(pos, hash.cell_size) {
            if let Some(bucket) = hash.buckets.get(&cell) {
                for &other in bucket {
                    if other == e { continue; }
                    let key = if e.index() < other.index() { (e, other) } else { (other, e) };
                    if !seen_pairs.insert(key) { continue; }

                    if let Ok((_, otf, mut oagent, orad)) = q.get_mut(other).map(|(_, t, a, rad)| (other, t, a, rad)) {
                        if agent.kind == oagent.kind { continue; }
                        let d = otf.translation.truncate() - pos;
                        let dist2 = d.length_squared();
                        let threshold = (rr + orad.0) * 1.0; // both radii; you set uniform
                        if dist2 <= (threshold * threshold) {
                            // resolve RPS
                            let (k1, k2) = (agent.kind, oagent.kind);
                            if beats(k1, k2) {
                                to_convert.push((other, k1));
                            } else if beats(k2, k1) {
                                to_convert.push((e, k2));
                            }
                        }
                    }
                }
            }
        }
    }

    // apply conversions
    for (entity, new_kind) in to_convert.into_iter() {
        if let Ok((_, _, mut agent, _)) = q.get_mut(entity) {
            agent.kind = new_kind;
            // recolor immediately
            if let Ok(mut sprite) = sprites.get_mut(entity) {
                sprite.color = super::super::spawn::color_of(new_kind);
            }
            // velocity/direction "start fresh": handled by steering/movement next frame
        }
    }
}

fn beats(a: Kind, b: Kind) -> bool {
    matches!(
        (a, b),
        (Kind::Rock, Kind::Scissors) |
        (Kind::Scissors, Kind::Paper) |
        (Kind::Paper, Kind::Rock)
    )
}
```

---

## Winner detection

```rust
// systems/winner.rs
use bevy::prelude::*;
use crate::{components::Agent, resources::RunState, components::Kind};

pub fn winner_check_system(
    mut run: ResMut<RunState>,
    q: Query<&Agent>,
) {
    if matches!(*run, RunState::Finished(_)) { return; }
    let mut counts = [0u32; 3];
    for a in q.iter() {
        match a.kind {
            Kind::Rock => counts[0]+=1,
            Kind::Paper => counts[1]+=1,
            Kind::Scissors => counts[2]+=1,
        }
    }
    let nonzero = counts.iter().filter(|&&c| c>0).count();
    if nonzero == 1 {
        let winner = if counts[0]>0 { Kind::Rock } else if counts[1]>0 { Kind::Paper } else { Kind::Scissors };
        *run = RunState::Finished(winner);
        // You can also pause FixedUpdate by gating systems on a run-state condition.
    }
}
```

---

## UI (minimal)

Start simple with Bevy UI (no extra deps). Later, switch to `bevy_egui` for sliders.

* Show: title “Living RPS”, counts per type, mode toggle (Blend/Dominant), weights sliders (active only in Blend), initial count slider (multiple of 3), **Reset** button (despawn & respawn), **Start** button (set `RunState::Running`).
* When finished: overlay “Winner: …” and disable systems via a run-state condition (or just stop steering/collision).

*(Omitted here for brevity; shout if you want me to drop a ready-to-run Bevy UI panel — I’ll match whichever Bevy version you pick.)*

---

# Performance notes (50–100 agents target)

* The uniform-grid `SpatialHash` + 3×3 neighbor cell scans keeps neighbor checks roughly O(n).
* Keep `cell_size` ≈ max(vision\_radius, sep\_radius). Tune.
* If you push to 1k+ agents, we can move to a quadtree or a two-phase pass.

---

# Next steps (you can follow this order)

1. **Scaffold project** (`cargo new living_rps`), add Bevy; get a window + camera + a few sprites moving.
2. Add **components/resources** above.
3. Implement **spawn** (grid jitter; equal split).
4. Add **steering + movement + boundary**. Verify they chase/flee.
5. Add **collision\_transform**. Verify transforms correctly (watch colors flip).
6. Add **winner\_check** and display overlay.
7. Add **UI** for:

   * initial count (multiple of 3),
   * mode toggle (Blend/Dominant),
   * weights sliders (chase/flee/sep),
   * reset/start.
8. Optional polish: emoji sprites, tiny transform “flash”, trails, or speed jitter.

---

If you want, I can:

* Pin this to a **specific Bevy / bevy\_egui version** and give you a *compilable* `main.rs` with UI.
* Swap circles → emojis (Text2D or textured quads).
* Add a **Poisson disk spawn** so initial spacing is \~uniform.

Tell me your Bevy version (or let me pick) and I’ll ship a runnable file next.

</assistant>

