mod coordinates;
mod detection;
mod hap;
mod point;
mod viewer;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use coordinates::CoordinateSystem;
use detection::DetectionConfig;
use hap::connection::LivoxConnection;
use point::Point3D;

const LIDAR_IP: &str = "192.168.14.69";
const HOST_IP: [u8; 4] = [192, 168, 14, 82];
const FRAME_INTERVAL: Duration = Duration::from_millis(100);
const MAX_POINTS: usize = 50_000;
const PRINT_INTERVAL: Duration = Duration::from_secs(2);

fn main() {
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        println!("\nCtrl+C received, shutting down...");
        r.store(false, Ordering::Relaxed);
    })
    .expect("Failed to set Ctrl+C handler");

    println!("Connecting to Livox HAP at {LIDAR_IP}...");

    let mut conn = match LivoxConnection::connect(LIDAR_IP, HOST_IP) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("Connection failed: {e}");
            std::process::exit(1);
        }
    };

    println!("Connected. Press Ctrl+C or close window to stop.");

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
    let mut last_frame = Instant::now();
    let mut last_print = Instant::now();
    let mut total_received: u64 = 0;

    while viewer.is_open() && running.load(Ordering::Relaxed) {
        conn.maintain();

        let new = conn.receive_points(&mut raw_points);
        total_received += new as u64;

        if raw_points.len() > MAX_POINTS {
            raw_points.drain(..raw_points.len() - MAX_POINTS);
        }

        if last_frame.elapsed() >= FRAME_INTERVAL {
            last_frame = Instant::now();

            points.clear();
            points.extend(raw_points.iter().map(Point3D::from_raw));

            let cones = if points.len() >= 10 {
                detection::detect_cones(&points, &config)
            } else {
                Vec::new()
            };

            coord_system.update(&cones);

            if last_print.elapsed() >= PRINT_INTERVAL {
                last_print = Instant::now();
                println!(
                    "Points: {} | Total recv: {} | Cones: {}",
                    points.len(), total_received, cones.len()
                );
                coord_system.print_positions();
            }

            viewer.sync_size();
            viewer.render(&points, &coord_system.cones);
        }

        std::thread::sleep(Duration::from_millis(1));
    }

    println!("Stopping LiDAR...");
    conn.stop();
    println!("Done.");
}
