# Repository Guidelines

## Project Structure & Modules
- `src/main.rs`: Bevy app entry, systems, components, and config loading.
- `config.toml`: Runtime simulation settings (mode, speeds, radii, counts, weights).
- `demo/`: Example videos of the simulation.
- `raw_prompts/`: Reference docs and assets for the project description.
- `target/`: Cargo build artifacts (generated).

## Build, Run, and Dev
- Build: `cargo build` — compiles the project.
- Run (Windows, recommended): `./run.ps1` — runs with `bevy/dynamic_linking` for faster iteration.
- Run (generic): `cargo run --features bevy/dynamic_linking`.
- Release build: `cargo build --release`.
- Lints: `cargo clippy -- -D warnings`.
- Format: `cargo fmt`.

## Coding Style & Naming
- Rust 2024 edition; keep code `rustfmt`-clean (`cargo fmt`).
- Prefer small, focused systems and clear ECS component names.
- Naming: `snake_case` for modules/functions, `CamelCase` for types, `SCREAMING_SNAKE_CASE` for consts.
- File layout: the app is single-file today; if splitting, group by domain (e.g., `systems/`, `components/`, `config/`).

## Testing Guidelines
- Framework: Rust built-in tests. Add `#[cfg(test)] mod tests { ... }` near the code under test.
- Run tests: `cargo test`.
- Naming: unit tests live next to code; integration tests go under `tests/` with descriptive file names (e.g., `winner_detection.rs`).
- Aim for deterministic helpers (seed RNG) when asserting behavior.

## Commit & PR Guidelines
- Commits: concise imperative subject lines ("Add winner check"), include rationale in the body when needed.
- Scope small, one logical change per commit.
- PRs: include summary, behavior changes, screenshots or short clip if visuals change, and mention config impacts (`config.toml` keys, defaults).
- Link related issues and describe manual test steps (build/run commands).

## Configuration Tips
- Key fields: `mode` (Blend|Dominant), `speed`, `vision_radius`, `threat_radius`, `sep_radius`, `arena`, `entity_radius`, `initial_count`, `cell_size`, `[weights]`.
- Keep `initial_count` a multiple of 3; `cell_size` ≈ max(vision, sep) for spatial hashing.
