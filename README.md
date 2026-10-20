# Smart Road Intersection

A native Rust simulation of a four-way intersection used by autonomous vehicles. There are no traffic lights. A central reservation system decides who may enter the crossing, and each vehicle follows one fixed lane.

**Project type:** Individual project, developed by Hasan Abbas.

## My work

I built the simulation in Rust, including the twelve vehicle routes, intersection reservations, movement and following behavior, collision checks, rendering, statistics and automated scenarios.

## What it does

The intersection has north, south, east, and west approaches. Each approach has three lanes:

- right turn
- straight
- left turn

A vehicle is placed in one of those lanes and stays on that route. Right and left turns follow a curve, and the vehicle's heading changes along the curve. Cars keep a gap from the vehicle ahead and slow down when the crossing is not available. Routes that do not cross may go through together. Routes that cross take turns.

The window is 1000×1000 pixels. Distances are pixels and time is seconds: movement is `velocity × dt`, and speed changes by acceleration or braking times `dt`. One pixel represents 0.1 m, so a speed in pixels per second converts to km/h by multiplying by 0.36. The screen shows both units.

## How crossing is managed

The paved square is split into a grid of cells. Each of the 12 routes occupies the cells its body passes through.

1. A vehicle asks for those cells before it reaches the square.
2. The request is granted when the cells are free, a short cooldown has elapsed since the last conflicting vehicle, and an older conflicting vehicle is not already waiting.
3. Several vehicles may cross at once when their cells do not overlap.
4. A vehicle that is refused brakes early and stops at the line instead of entering.
5. The cells are released when the whole vehicle has left the square.

Waiting vehicles are served in arrival order. A vehicle that has been waiting is not skipped by a newer one on a crossing route.

## Safety distance

| Constant | Value |
| --- | --- |
| Vehicle length | 36 px |
| Vehicle width | 18 px |
| Minimum bumper gap on the same route | 32 px |
| Close-call gap | below 14 px |
| Collision gap | below 2 px |

Overlap uses the vehicle rectangle, not only the distance between centers. A following car reduces speed as the gap shrinks, stops short of the leader, and pulls away again when the lane opens. If a planned step would enter the collision gap, that step is refused.

Desired speeds are 62, 102, and 142 px/s (about 22, 37, and 51 km/h). Acceleration is 72 px/s² and comfortable braking is 150 px/s².

## Colors

- Right turn: orange
- Straight: cyan
- Left turn: magenta

Small shade changes are only visual variants. The route color stays the same.

## Controls

| Key | Action |
| --- | --- |
| Up | Spawn from the south, toward the intersection |
| Down | Spawn from the north |
| Right | Spawn from the west |
| Left | Spawn from the east |
| R | Turn continuous random spawning on or off |
| Esc | Stop the simulation and open the statistics screen |
| Enter, Space, or Esc | Leave the statistics screen |

Each arrow spawn picks a random legal route and a random speed for that direction. Random mode also varies direction, route, color variant, and speed. A spawn is refused when that lane's entrance is still occupied. Other lanes can still receive a vehicle. The corner panel shows whether random generation is ON or OFF, plus the live vehicle count, total spawned, total completed, and rejected or deferred spawns.

`Min velocity` is the lowest recorded speed, including zero when a vehicle is stopped. `Min moving velocity` ignores those stops. If nobody has finished a trip, traversal times are shown as `N/A`.

## Requirements

Install Rust from [https://rustup.rs](https://rustup.rs). The project uses the stable toolchain. No game engine, SDL library, or extra system package is required.

This is a desktop program. It does not open a web server, and it does not need localhost or an internet connection after Rust and the crates are installed. The first build downloads crates. Later builds and normal runs work offline.

## Build and run

From the project directory:

```bash
cargo run --release
```

The same command is used on Windows, Linux, and macOS. On Windows, open PowerShell or Command Prompt in this folder. On Linux or macOS, open a terminal in this folder.

Useful checks:

```bash
cargo fmt --check
cargo check
cargo clippy --all-targets --all-features -- -D warnings
cargo test
cargo build --release
```

`cargo test` includes a one-minute random run. A longer five-seed run is ignored by default:

```bash
cargo test -- --ignored --test scenarios longer_random_seeds_stay_safe
```

Release binary:

```bash
cargo build --release
```

The executable is `target/release/smart_road` on Linux and macOS, and `target/release/smart_road.exe` on Windows. Textures are embedded in that executable, so another person does not need a separate assets folder to start the program. The `assets/` directory is still part of the source tree.

Give someone else the source directory, or a build made for their operating system. A Windows `.exe` does not run on Linux or macOS. They install Rust, open a terminal in the project, and run `cargo run --release`.

## Project layout

```text
src/main.rs          window, keyboard, screen change
src/render.rs        roads, vehicles, counters, statistics
src/simulation.rs    spawning, motion, lifecycle
src/vehicle.rs       per-vehicle state
src/routes.rs        12 fixed paths and conflict cells
src/intersection.rs  reservation locks
src/collision.rs     rectangle gap checks
src/statistics.rs    trip summary
src/config.rs        units and distances
assets/              asphalt, car, and arrow images
tests/scenarios.rs   headless scenarios
```

If an image cannot be decoded, the view falls back to drawn shapes and the simulation keeps running.

## Statistics

The statistics screen includes:

- Max number of vehicles that passed the intersection
- Max velocity
- Min velocity
- Max time that took a vehicle to pass the intersection
- Min time that took a vehicle to pass the intersection
- Close calls

It also shows spawned vehicles, vehicles still on screen, collisions, average traversal time, average moving speed, the busiest moment inside the square, the longest queue overall and per lane, the smallest recorded gap, rejected or deferred spawns, and the run duration.

The timer for a vehicle starts when it is created and stops when it has fully left the window. Vehicles still on screen when Esc is pressed are not treated as finished.

## Troubleshooting

- `cargo` is not recognized: install Rust from rustup and open a new terminal so the PATH update is picked up.
- The first build is slow: it compiles the graphics library. Later release builds are faster.
- The window does not appear: update the GPU driver. The program uses a normal desktop OpenGL window.
- Cars will not spawn: that lane is still occupied. Wait for a gap, or spawn from another direction.
- Random mode fills the approaches: spawning slows down while a lane is crowded. Cars already on the road are not removed.
