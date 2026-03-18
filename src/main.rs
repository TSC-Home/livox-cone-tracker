mod coordinates;
mod detection;
mod hap;
mod point;
mod sim_connection;
mod viewer;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use coordinates::CoordinateSystem;
use detection::DetectionConfig;
use hap::connection::LivoxConnection;
use point::Point3D;
use sim_connection::SimConnection;

const LIDAR_IP: &str = "192.168.14.69";
const HOST_IP: [u8; 4] = [192, 168, 14, 82];
const RENDER_INTERVAL: Duration = Duration::from_millis(33);   // ~30 fps
const DETECT_INTERVAL: Duration = Duration::from_millis(500);  // detection cadence
const MAX_POINTS: usize = 20_000;
const PRINT_INTERVAL: Duration = Duration::from_secs(3);

// ── Simulation paths (relative to project root / cargo run CWD) ──────────────
const SIM_WORLD_STRAIGHT: &str = "sim/worlds/track.world";
const SIM_WORLD_OVAL:     &str = "sim/worlds/oval.world";
const SIM_BRIDGE: &str = "sim/bridge.py";
const SIM_MODELS: &str = "sim/models";

/// Unified connection abstraction – real hardware or simulator.
enum Connection {
    Real(LivoxConnection),
    Sim(SimConnection),
}

impl Connection {
    fn maintain(&mut self) {
        match self { Self::Real(c) => c.maintain(), Self::Sim(c) => c.maintain() }
    }
    fn receive_points(&mut self, pts: &mut Vec<[f32; 4]>) -> usize {
        match self { Self::Real(c) => c.receive_points(pts), Self::Sim(c) => c.receive_points(pts) }
    }
    fn stop(&mut self) {
        match self { Self::Real(c) => c.stop(), Self::Sim(c) => c.stop() }
    }
}

fn main() {
    // ── Parse flags ──────────────────────────────────────────────────────────
    let sim_mode    = std::env::args().any(|a| a == "--sim");
    let circle_mode = std::env::args().any(|a| a == "--circle");

    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        println!("\nCtrl+C received, shutting down...");
        r.store(false, Ordering::Relaxed);
    })
    .expect("Failed to set Ctrl+C handler");

    // ── Launch Gazebo + Python bridge in sim mode ─────────────────────────────
    let mut child_gazebo: Option<std::process::Child> = None;
    let mut child_bridge: Option<std::process::Child> = None;

    if sim_mode {
        // Kill any stale Gazebo instance first
        let _ = std::process::Command::new("pkill").args(["-f", "gzserver"]).status();
        std::thread::sleep(Duration::from_millis(500));

        let world_file = if circle_mode { SIM_WORLD_OVAL } else { SIM_WORLD_STRAIGHT };
        println!("[sim] Starting Gazebo with {} …", world_file);
        match std::process::Command::new("gazebo")
            .arg("--verbose")
            .arg(world_file)
            .env("GAZEBO_MODEL_PATH", SIM_MODELS)
            .spawn()
        {
            Ok(ch) => { child_gazebo = Some(ch); }
            Err(e) => { eprintln!("[sim] Could not start Gazebo: {e}"); }
        }

        // Give Gazebo 1 s to initialise before starting the bridge
        std::thread::sleep(Duration::from_secs(1));

        println!("[sim] Starting LiDAR bridge (bridge.py) …");
        let mut bridge_cmd = std::process::Command::new("python3");
        bridge_cmd.arg(SIM_BRIDGE);
        if circle_mode { bridge_cmd.arg("--circle"); }
        match bridge_cmd.spawn()
        {
            Ok(ch) => { child_bridge = Some(ch); }
            Err(e) => { eprintln!("[sim] Could not start bridge.py: {e}"); }
        }

        // Give the bridge a moment to start sending
        std::thread::sleep(Duration::from_millis(500));
    }

    // ── Connect ──────────────────────────────────────────────────────────────
    let mut conn: Connection = if sim_mode {
        match SimConnection::new() {
            Ok(c)  => Connection::Sim(c),
            Err(e) => { eprintln!("Sim connection failed: {e}"); std::process::exit(1); }
        }
    } else {
        println!("Connecting to Livox HAP at {LIDAR_IP}...");
        match LivoxConnection::connect(LIDAR_IP, HOST_IP) {
            Ok(c)  => Connection::Real(c),
            Err(e) => { eprintln!("Connection failed: {e}"); std::process::exit(1); }
        }
    };

    if sim_mode {
        println!("[sim] Simulation running. Press Ctrl+C or close window to stop.");
    } else {
        println!("Connected. Press Ctrl+C or close window to stop.");
    }

    let mut viewer = match viewer::Viewer::new() {
        Ok(v) => v,
        Err(e) => {
            eprintln!("Failed to create viewer: {e}");
            conn.stop();
            std::process::exit(1);
        }
    };

    let config = DetectionConfig::default();
    let mut coord_system = CoordinateSystem::new();
    let mut raw_points: Vec<[f32; 4]> = Vec::with_capacity(MAX_POINTS);
    let mut points: Vec<Point3D> = Vec::with_capacity(MAX_POINTS);
    let mut last_render = Instant::now();
    let mut last_detect = Instant::now();
    let mut last_print = Instant::now();
    let mut total_received: u64 = 0;

    while viewer.is_open() && running.load(Ordering::Relaxed) {
        conn.maintain();

        let new = conn.receive_points(&mut raw_points);
        total_received += new as u64;

        if raw_points.len() > MAX_POINTS {
            raw_points.drain(..raw_points.len() - MAX_POINTS);
        }

        // Detection runs less often (expensive)
        if last_detect.elapsed() >= DETECT_INTERVAL {
            last_detect = Instant::now();

            points.clear();
            points.extend(
                raw_points.iter()
                    .map(Point3D::from_raw)
                    .filter(Point3D::is_valid)
            );

            let cones = if points.len() >= 10 {
                detection::detect_cones(&points, &config)
            } else {
                Vec::new()
            };

            coord_system.update(&cones);

            if last_print.elapsed() >= PRINT_INTERVAL {
                last_print = Instant::now();
                println!(
                    "Points: {} | Total: {} | Cones: {}",
                    points.len(), total_received, cones.len()
                );
                coord_system.print_positions();
            }
        }

        // Render runs at ~30fps (cheap)
        if last_render.elapsed() >= RENDER_INTERVAL {
            last_render = Instant::now();

            // Only rebuild points vec if not recently done by detection
            if points.is_empty() && !raw_points.is_empty() {
                points.clear();
                points.extend(
                    raw_points.iter()
                        .map(Point3D::from_raw)
                        .filter(Point3D::is_valid)
                );
            }

            viewer.sync_size();
            viewer.render(&points, &mut coord_system);
        }

        std::thread::sleep(Duration::from_millis(1));
    }

    println!("Stopping LiDAR...");
    conn.stop();

    // ── Kill simulation subprocesses ──────────────────────────────────────
    if let Some(mut ch) = child_bridge {
        let _ = ch.kill();
        println!("[sim] bridge.py stopped.");
    }
    if let Some(mut ch) = child_gazebo {
        let _ = ch.kill();
        println!("[sim] Gazebo stopped.");
    }
    // Kill any lingering gzserver / gzclient that survive the launcher kill
    if sim_mode {
        let _ = std::process::Command::new("pkill").args(["-f", "gzserver"]).status();
        let _ = std::process::Command::new("pkill").args(["-f", "gzclient"]).status();
    }

    println!("Done.");
}
