# Cone Detection with Livox HAP (TX)

Real-time traffic cone detection and track boundary fitting using a Livox HAP LiDAR sensor. Built for [FastTube](https://fasttube.de) Formula Student autonomous driving.

![Viewer Screenshot](assets/viewer.png)

## Features

- **Live LiDAR streaming** via Livox HAP UDP protocol
- **3D cone detection** using voxel-based clustering with height/radius filtering
- **Track boundary fitting** with KNN graph + DFS pathfinding for left/right boundaries
- **Virtual cone interpolation** to fill gaps between detected cones
- **Center line computation** from matched left/right boundary pairs
- **Real-time 3D viewer** with camera controls and toggle overlays
- **Cone tracking** with exponential smoothing across frames

## Requirements

- Rust 1.85+ (edition 2024)
- Livox HAP LiDAR sensor connected via Ethernet
- Linux (tested on Fedora)

## Setup

### 1. Network Configuration

Connect the Livox HAP sensor via Ethernet and configure your network interface:

```bash
# Set a static IP on the interface connected to the LiDAR
sudo ip addr add 192.168.14.82/24 dev <interface>
sudo ip link set <interface> up
```

The default LiDAR IP is `192.168.14.69`. If your sensor has a different IP, update `LIDAR_IP` in `src/main.rs`.

### 2. Build

```bash
cargo build --release
```

### 3. Run

```bash
cargo run --release
```

The viewer window opens automatically. The system connects to the LiDAR, starts streaming points, and begins detection.

## Viewer Controls

| Key | Action |
|-----|--------|
| **LMB drag** | Rotate camera |
| **RMB drag** | Pan camera |
| **Scroll** | Zoom in/out |
| **W/A/S/D** | Move camera target |
| **Q/E** | Move camera up/down |
| **R** | Reset camera |
| **1/2/3** | Preset views (top/front/side) |
| **P** | Toggle point cloud display (all vs near-cone only) |
| **T** | Toggle outlier cone display |
| **C** | Toggle cone detection overlay |
| **B** | Toggle pathfinding (track boundaries) |
| **V** | Toggle virtual cone interpolation |
| **F** | Toggle auto-center camera |

### Toggle Dependencies

The three detection layers have automatic dependencies:

```
Cones (C) --> Pathfinding (B) --> Virtual Cones (V)
```

- Enabling **V** automatically enables **B** and **C**
- Enabling **B** automatically enables **C**
- Disabling **C** automatically disables **B** and **V**
- Disabling **B** automatically disables **V**

## Architecture

```
src/
  main.rs                  -- Entry point, LiDAR connection, main loop
  detection.rs             -- Voxel-based cone detection from point clouds
  point.rs                 -- Point3D type
  viewer.rs                -- Real-time 3D renderer (minifb)
  coordinates/
    mod.rs                 -- CoordinateSystem: tracking, update, toggles
    types.rs               -- ConePosition, TrackedCone, constants
    graph.rs               -- KNN graph construction
    pathfind.rs            -- DFS path search, start cone selection
    extensions.rs          -- Forward extension, cross-track guided extension
    fitting.rs             -- Track fitting pipeline (orchestrates all steps)
    persistence.rs         -- Track stability, reconstruction, virtual cones
    conflicts.rs           -- Left/right conflict resolution, path validation
    cost.rs                -- Path cost function, track overlap, width estimation
  hap/
    mod.rs                 -- HAP protocol module
    protocol.rs            -- Livox HAP packet parsing
    commands.rs            -- LiDAR control commands
    connection.rs          -- UDP connection management
```

## Resource Usage

Measured on an Intel i5-13600K, single-threaded:

| Metric | Value |
|--------|-------|
| Detection interval | 500 ms |
| Render rate | ~30 FPS |
| Max point buffer | 20,000 points |
| Detection latency | < 10 ms per cycle (typical) |
| Pathfinding latency | < 5 ms per cycle (typical) |
| Memory usage | ~10-20 MB RSS |
| Binary size (release) | ~1.5 MB (with LTO) |

### Detection Pipeline Timing

```
Point accumulation     500 ms (configurable via DETECT_INTERVAL)
  |
  v
Voxel clustering       O(n) where n = point count
  |
  v
Cone extraction        O(k) where k = cluster count
  |
  v
Tracking (EMA)         O(c * t) where c = detections, t = tracked cones
  |
  v
KNN graph              O(n^2) where n = cone count (typically < 30)
  |
  v
DFS path search        O(b^d) bounded by MAX_PATH_LEN=16
  |
  v
Conflict resolution    O(l * r) where l, r = left/right path lengths
  |
  v
Rendering              ~33 ms budget at 30 FPS
```

### Dependencies

| Crate | Purpose |
|-------|---------|
| `minifb` | Minimal framebuffer window for 3D viewer |
| `crc` | CRC checksums for Livox HAP protocol |
| `ctrlc` | Graceful Ctrl+C shutdown |
| `libc` | Low-level socket operations for UDP |

No GPU, no OpenCV, no ROS -- pure Rust with zero heavy dependencies.

## License

You are free to use, modify, and build upon this project. If you do, please:

1. Make your repository **publicly available**
2. Open an **issue** on [this repo](https://github.com/tsc-home/livox-cone-tracker) with a link to your project
3. Add a link back to this repository in your README
