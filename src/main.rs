use bevy::prelude::*;
use bevy::sprite::Sprite;
use rand::Rng;
use rand::seq::SliceRandom;
use serde::Deserialize;
use std::collections::{HashMap, HashSet};
use std::fs;

/// ====== Types & Components ======
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
enum Kind {
    Rock,
    Paper,
    Scissors,
}

#[derive(Component)]
struct Agent {
    kind: Kind,
}

#[derive(Component, Deref, DerefMut, Default)]
struct Velocity(Vec2);

#[derive(Component)]
struct Speed(f32);

#[derive(Component)]
struct Radius(f32);

#[derive(Component)]
struct DesiredDir(Vec2);

#[derive(Component)]
struct Collider;

/// ====== Resources ======
#[derive(Clone, Copy, PartialEq, Eq)]
enum SteeringMode {
    Blend,
    Dominant,
}

#[derive(Resource)]
struct Weights {
    chase: f32,
    flee: f32,
    separation: f32,
}

#[derive(Resource)]
struct SimConfig {
    mode: SteeringMode,
    speed: f32,
    vision_radius: f32,
    threat_radius: f32,
    sep_radius: f32,
    arena_size: Vec2,
    entity_radius: f32,
    initial_count: usize,
    cell_size: f32,
}

#[derive(Resource)]
enum RunState {
    Running,
    Finished(Kind),
}

#[derive(Resource, Default)]
struct SpatialHash {
    cell_size: f32,
    buckets: HashMap<IVec2, Vec<Entity>>,
}

/// ====== App entry ======
fn main() {
    // Load configuration from file once at startup
    let (sim_config, weights) = load_or_default_config("config.toml");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Living RPS".into(),
                resolution: (1280., 900.).into(),
                ..Default::default()
            }),
            ..Default::default()
        }))
        .insert_resource(Time::<Fixed>::from_hz(60.0))
        .insert_resource(sim_config)
        .insert_resource(RunState::Running)
        .insert_resource(SpatialHash::default())
        .insert_resource(weights)
        .add_systems(Startup, (setup_camera, initial_spawn))
        .add_systems(
            FixedUpdate,
            (
                spatial_hash_update_system,
                steering_system,
                movement_system,
                boundary_system,
                collision_transform_system,
                same_kind_nonoverlap_system,
                winner_check_system,
            )
                .chain(),
        )
        .run();
}

/// ====== Startup ======
fn setup_camera(mut commands: Commands) {
    commands.spawn(Camera2d);
}

fn initial_spawn(mut commands: Commands, config: Res<SimConfig>) {
    spawn_agents(&mut commands, &config);
}

fn spawn_agents(commands: &mut Commands, config: &SimConfig) {
    let total = (config.initial_count.max(3) / 3) * 3;
    let each = total / 3;
    let kinds = [Kind::Rock, Kind::Paper, Kind::Scissors];

    // Grid + jitter to space out
    let cols = (total as f32).sqrt().ceil() as i32;
    let rows = ((total as f32) / cols as f32).ceil() as i32;
    let cell_w = config.arena_size.x / cols as f32;
    let cell_h = config.arena_size.y / rows as f32;

    let mut rng = rand::rng();
    // Create a balanced pool of kinds and shuffle to avoid spatial banding
    let mut shuffled_kinds: Vec<Kind> = Vec::with_capacity(total);
    for &k in &kinds {
        for _ in 0..each {
            shuffled_kinds.push(k);
        }
    }
    shuffled_kinds.shuffle(&mut rng);
    let mut idx = 0usize;
    for r in 0..rows {
        for c in 0..cols {
            if idx >= total {
                break;
            }
            let k = shuffled_kinds[idx];

            let jitter = Vec2::new(
                rng.random_range(-0.3..0.3) * cell_w,
                rng.random_range(-0.3..0.3) * cell_h,
            );
            let half = config.arena_size * 0.5;
            let pos = Vec3::new(
                -half.x + (c as f32 + 0.5) * cell_w + jitter.x,
                -half.y + (r as f32 + 0.5) * cell_h + jitter.y,
                0.0,
            );

            commands.spawn((
                Sprite::from_color(color_of(k), Vec2::splat(config.entity_radius * 2.0)),
                Transform::from_translation(pos),
                Agent { kind: k },
                Radius(config.entity_radius),
                Speed(config.speed),
                Velocity(random_dir() * config.speed),
                DesiredDir(random_dir()),
                Collider,
            ));

            idx += 1;
        }
    }
}

fn color_of(k: Kind) -> Color {
    match k {
        Kind::Rock => Color::srgb(0.6, 0.6, 0.68),
        Kind::Paper => Color::srgb(0.72, 0.56, 0.36),
        Kind::Scissors => Color::srgb(0.9, 0.22, 0.22),
    }
}

fn random_dir() -> Vec2 {
    let a = rand::random::<f32>() * std::f32::consts::TAU;
    Vec2::new(a.cos(), a.sin())
}

// UI removed; configuration is loaded once from config.toml at startup.

/// ====== Spatial Hash ======
fn spatial_hash_update_system(
    mut hash: ResMut<SpatialHash>,
    config: Res<SimConfig>,
    q: Query<(Entity, &Transform), With<Agent>>,
) {
    hash.cell_size = config.cell_size;
    hash.buckets.clear();
    for (e, t) in q.iter() {
        let p = t.translation.truncate();
        let cell = IVec2::new(
            (p.x / hash.cell_size).floor() as i32,
            (p.y / hash.cell_size).floor() as i32,
        );
        hash.buckets.entry(cell).or_default().push(e);
    }
}

fn neighbor_cells(p: Vec2, cell_size: f32) -> [IVec2; 9] {
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

/// ====== Steering ======
fn steering_system(
    run: Res<RunState>,
    config: Res<SimConfig>,
    weights: Res<Weights>,
    mut q_me: Query<(Entity, &Agent, &Transform, &mut DesiredDir), With<Agent>>,
    q_all: Query<(Entity, &Transform, &Agent), With<Agent>>,
) {
    if matches!(*run, RunState::Finished(_)) {
        return;
    }

    for (e, me, tf, mut desired) in q_me.iter_mut() {
        let pos = tf.translation.truncate();
        let mut nearest_prey: Option<(f32, Vec2)> = None;
        let mut nearest_pred: Option<(f32, Vec2)> = None;
        let mut sep_acc = Vec2::ZERO;

        let prey_kind = prey_of(me.kind);
        let pred_kind = predator_of(me.kind);

        // Global vision: consider all other agents in the world (no distance clamp)
        for (other, otf, oagent) in q_all.iter() {
            if other == e { continue; }
            let op = otf.translation.truncate();
            let d = op - pos;
            let dist2 = d.length_squared();
            if dist2 == 0.0 { continue; }
            let dist = dist2.sqrt();

            if oagent.kind == prey_kind
                && nearest_prey.map_or(true, |(best, _)| dist < best)
            {
                nearest_prey = Some((dist, d));
            }
            if oagent.kind == pred_kind
                && nearest_pred.map_or(true, |(best, _)| dist < best)
            {
                nearest_pred = Some((dist, d));
            }
            if oagent.kind == me.kind && dist <= config.sep_radius {
                sep_acc += (pos - op) / (dist2 + 1.0);
            }
        }

        let dir_chase = nearest_prey
            .map(|(_, d)| d.normalize())
            .unwrap_or(Vec2::ZERO);
        let dir_flee = nearest_pred
            .map(|(_, d)| (-d).normalize())
            .unwrap_or(Vec2::ZERO);
        let dir_sep = if sep_acc.length_squared() > 0.0 {
            sep_acc.normalize()
        } else {
            Vec2::ZERO
        };

        let out = match config.mode {
            SteeringMode::Blend => {
                let v = weights.chase * dir_chase
                    + weights.flee * dir_flee
                    + weights.separation * dir_sep;
                if v.length_squared() > 1e-6 {
                    v.normalize()
                } else {
                    desired.0
                }
            }
            SteeringMode::Dominant => {
                if let Some((dist, d)) = nearest_pred {
                    if dist <= config.threat_radius {
                        (-d).normalize()
                    } else if let Some((_, dp)) = nearest_prey {
                        dp.normalize()
                    } else if dir_sep != Vec2::ZERO {
                        dir_sep
                    } else {
                        desired.0
                    }
                } else if let Some((_, dp)) = nearest_prey {
                    dp.normalize()
                } else if dir_sep != Vec2::ZERO {
                    dir_sep
                } else {
                    desired.0
                }
            }
        };

        desired.0 = out;
    }
}

fn prey_of(k: Kind) -> Kind {
    match k {
        Kind::Rock => Kind::Scissors,
        Kind::Scissors => Kind::Paper,
        Kind::Paper => Kind::Rock,
    }
}
fn predator_of(k: Kind) -> Kind {
    match k {
        Kind::Rock => Kind::Paper,
        Kind::Scissors => Kind::Rock,
        Kind::Paper => Kind::Scissors,
    }
}

/// ====== Movement & Boundary ======
fn movement_system(
    run: Res<RunState>,
    time: Res<Time>,
    config: Res<SimConfig>,
    mut q: Query<(&DesiredDir, &mut Velocity, &mut Transform), With<Agent>>,
) {
    if matches!(*run, RunState::Finished(_)) {
        return;
    }
    let dt = time.delta_secs();
    for (desired, mut vel, mut tf) in q.iter_mut() {
        if desired.0.length_squared() > 0.0 {
            vel.0 = desired.0 * config.speed;
        }
        tf.translation.x += vel.0.x * dt;
        tf.translation.y += vel.0.y * dt;
    }
}

fn boundary_system(
    config: Res<SimConfig>,
    mut q: Query<(
        &mut Transform,
        &mut Velocity,
        &Radius,
        &mut DesiredDir,
    ), With<Agent>>,
) {
    let half = config.arena_size * 0.5;
    for (mut tf, mut vel, r, mut desired) in q.iter_mut() {
        let mut p = tf.translation.truncate();
        let rr = r.0;

        if p.x < -half.x + rr {
            p.x = -half.x + rr;
            vel.0.x = vel.0.x.abs();
            desired.0.x = desired.0.x.abs();
        } else if p.x > half.x - rr {
            p.x = half.x - rr;
            vel.0.x = -vel.0.x.abs();
            desired.0.x = -desired.0.x.abs();
        }
        if p.y < -half.y + rr {
            p.y = -half.y + rr;
            vel.0.y = vel.0.y.abs();
            desired.0.y = desired.0.y.abs();
        } else if p.y > half.y - rr {
            p.y = half.y - rr;
            vel.0.y = -vel.0.y.abs();
            desired.0.y = -desired.0.y.abs();
        }
        // Keep desired direction normalized if non-zero after reflection
        if desired.0.length_squared() > 1e-6 {
            desired.0 = desired.0.normalize();
        }
        tf.translation.x = p.x;
        tf.translation.y = p.y;
    }
}

/// ====== Collision + Transform ======
fn collision_transform_system(
    run: Res<RunState>,
    _config: Res<SimConfig>,
    hash: Res<SpatialHash>,
    mut q: Query<(Entity, &Transform, &mut Agent, &Radius), With<Agent>>,
    mut q_sprite: Query<&mut Sprite>,
) {
    if matches!(*run, RunState::Finished(_)) {
        return;
    }

    // Collect conversions to apply after scanning
    let mut to_convert: Vec<(Entity, Kind)> = Vec::new();
    let mut seen_pairs: HashSet<(Entity, Entity)> = HashSet::new();

    // Snapshot to allow two-phase logic (borrow checker friendly)
    let snapshot: Vec<(Entity, Vec2, Kind, f32)> = q
        .iter_mut()
        .map(|(e, tf, a, r)| (e, tf.translation.truncate(), a.kind, r.0))
        .collect();

    for (e, pos, kind, rr) in snapshot.iter() {
        for cell in neighbor_cells(*pos, hash.cell_size) {
            if let Some(bucket) = hash.buckets.get(&cell) {
                for &other in bucket {
                    if other == *e {
                        continue;
                    }
                    let key = if e.index() < other.index() {
                        (*e, other)
                    } else {
                        (other, *e)
                    };
                    if !seen_pairs.insert(key) {
                        continue;
                    }

                    if let Some((_, op, okind, orr)) =
                        snapshot.iter().find(|(id, _, _, _)| *id == other)
                    {
                        if *okind == *kind {
                            continue;
                        }
                        let d = *op - *pos;
                        let dist2 = d.length_squared();
                        let thresh = *rr + *orr;
                        if dist2 <= thresh * thresh {
                            if beats(*kind, *okind) {
                                to_convert.push((other, *kind));
                            } else if beats(*okind, *kind) {
                                to_convert.push((*e, *okind));
                            }
                        }
                    }
                }
            }
        }
    }

    // Apply conversions
    for (entity, new_kind) in to_convert.into_iter() {
        if let Ok((_, _, mut agent, _)) = q.get_mut(entity) {
            agent.kind = new_kind;
            if let Ok(mut sprite) = q_sprite.get_mut(entity) {
                sprite.color = color_of(new_kind);
            }
            // velocity/direction "start fresh": next steering tick sets it
        }
    }
}

/// Enforce non-overlapping same-kind agents in Dominant mode by pushing them apart
fn same_kind_nonoverlap_system(
    run: Res<RunState>,
    config: Res<SimConfig>,
    mut q: Query<(Entity, &mut Transform, &Agent, &Radius), With<Agent>>,
) {
    if matches!(*run, RunState::Finished(_)) {
        return;
    }
    if !matches!(config.mode, SteeringMode::Dominant) {
        return;
    }

    // Snapshot positions to compute pairwise overlaps
    let mut snapshot: Vec<(Entity, Vec2, Kind, f32)> = Vec::new();
    for (e, tf, a, r) in q.iter_mut() {
        snapshot.push((e, tf.translation.truncate(), a.kind, r.0));
    }

    let mut offsets: HashMap<Entity, Vec2> = HashMap::new();
    let n = snapshot.len();
    for i in 0..n {
        let (ei, pi, ki, ri) = snapshot[i];
        for j in (i + 1)..n {
            let (ej, pj, kj, rj) = snapshot[j];
            if ki != kj {
                continue;
            }
            let d = pj - pi;
            let dist2 = d.length_squared();
            let min_dist = ri + rj;
            if dist2 < min_dist * min_dist {
                let dist = dist2.sqrt();
                let dir = if dist > 1e-6 { d / dist } else { random_dir() };
                let overlap = min_dist - dist;
                let push = 0.5 * overlap * dir;
                offsets
                    .entry(ei)
                    .and_modify(|v| *v -= push)
                    .or_insert(-push);
                offsets
                    .entry(ej)
                    .and_modify(|v| *v += push)
                    .or_insert(push);
            }
        }
    }

    if offsets.is_empty() {
        return;
    }

    // Apply accumulated offsets
    for (e, mut tf, _a, _r) in q.iter_mut() {
        if let Some(dp) = offsets.get(&e) {
            tf.translation.x += dp.x;
            tf.translation.y += dp.y;
        }
    }
}

fn beats(a: Kind, b: Kind) -> bool {
    matches!(
        (a, b),
        (Kind::Rock, Kind::Scissors) | (Kind::Scissors, Kind::Paper) | (Kind::Paper, Kind::Rock)
    )
}

/// ====== Winner Detection ======
fn winner_check_system(mut run: ResMut<RunState>, q: Query<&Agent, With<Agent>>) {
    if matches!(*run, RunState::Finished(_)) {
        return;
    }
    let mut counts = [0u32; 3];
    for a in q.iter() {
        match a.kind {
            Kind::Rock => counts[0] += 1,
            Kind::Paper => counts[1] += 1,
            Kind::Scissors => counts[2] += 1,
        }
    }
    let nonzero = counts.iter().filter(|&&c| c > 0).count();
    if nonzero == 1 {
        let winner = if counts[0] > 0 {
            Kind::Rock
        } else if counts[1] > 0 {
            Kind::Paper
        } else {
            Kind::Scissors
        };
        *run = RunState::Finished(winner);
    }
}

/// ====== Helpers ======
fn ninety_safe(x: usize) -> usize {
    let y = (x / 3) * 3;
    if y < 9 { 9 } else { y }
}

/// ====== Config file loading ======
#[derive(Deserialize)]
struct FileWeights {
    chase: Option<f32>,
    flee: Option<f32>,
    separation: Option<f32>,
}

#[derive(Deserialize)]
struct FileConfig {
    mode: Option<String>,
    speed: Option<f32>,
    vision_radius: Option<f32>,
    threat_radius: Option<f32>,
    sep_radius: Option<f32>,
    arena: Option<[f32; 2]>,
    entity_radius: Option<f32>,
    initial_count: Option<usize>,
    cell_size: Option<f32>,
    weights: Option<FileWeights>,
}

fn load_or_default_config(path: &str) -> (SimConfig, Weights) {
    let defaults_cfg = SimConfig {
        mode: SteeringMode::Blend,
        speed: 140.0,
        vision_radius: 160.0,
        threat_radius: 120.0,
        sep_radius: 45.0,
        arena_size: Vec2::new(1200., 800.),
        entity_radius: 10.0,
        initial_count: ninety_safe(60),
        cell_size: 160.0,
    };
    let defaults_w = Weights {
        chase: 1.0,
        flee: 1.6,
        separation: 0.3,
    };

    let Ok(s) = fs::read_to_string(path) else {
        return (defaults_cfg, defaults_w);
    };
    let Ok(mut fc) = toml::from_str::<FileConfig>(&s) else {
        return (defaults_cfg, defaults_w);
    };

    let weights = fc
        .weights
        .take()
        .map(|w| Weights {
            chase: w.chase.unwrap_or(defaults_w.chase),
            flee: w.flee.unwrap_or(defaults_w.flee),
            separation: w.separation.unwrap_or(defaults_w.separation),
        })
        .unwrap_or(defaults_w);

    let arena_arr = fc
        .arena
        .unwrap_or([defaults_cfg.arena_size.x, defaults_cfg.arena_size.y]);
    let arena_size = Vec2::new(arena_arr[0], arena_arr[1]);
    let mode = match fc
        .mode
        .unwrap_or("Blend".to_string())
        .to_lowercase()
        .as_str()
    {
        "blend" => SteeringMode::Blend,
        "dominant" => SteeringMode::Dominant,
        _ => SteeringMode::Blend,
    };

    let sim = SimConfig {
        mode,
        speed: fc.speed.unwrap_or(defaults_cfg.speed),
        vision_radius: fc.vision_radius.unwrap_or(defaults_cfg.vision_radius),
        threat_radius: fc.threat_radius.unwrap_or(defaults_cfg.threat_radius),
        sep_radius: fc.sep_radius.unwrap_or(defaults_cfg.sep_radius),
        arena_size,
        entity_radius: fc.entity_radius.unwrap_or(defaults_cfg.entity_radius),
        initial_count: ninety_safe(fc.initial_count.unwrap_or(defaults_cfg.initial_count)),
        cell_size: fc.cell_size.unwrap_or(defaults_cfg.cell_size),
    };

    (sim, weights)
}
