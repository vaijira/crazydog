//! # go2_detector
//!
//! **Intruder detection node** using LiDAR point cloud clustering and
//! camera color-based detection for the Unitree Go2 guard dog.
//!
//! ## Sensor Fusion
//!
//! - **LiDAR** (`/lidar/points`): clusters nearby points, filters ground,
//!   identifies person-sized obstacles (0.3–0.8m wide, 0.5–2.0m tall).
//! - **Camera** (`/camera/image_raw`): detects bright red regions (the
//!   intruder's distinctive color) and computes bearing offset.
//!
//! ## Published
//!
//! | Topic                     | Type                    | Description                  |
//! |---------------------------|-------------------------|------------------------------|
//! | `/go2/intruder_bearing`   | `geometry_msgs/Vector3` | x=distance, y=bearing, z=vel |
//! | `/go2/intruder_detected`  | `std_msgs/Bool`         | true when intruder is visible |

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::Result;
use log::info;

use geometry_msgs::msg::Vector3;
use rclrs::{Context, CreateBasicExecutor};
use sensor_msgs::msg::{Image, PointCloud2};
use std_msgs::msg::Bool;

// ============================================================
// Detection State
// ============================================================

/// Snapshot of detection data from both sensors.
#[derive(Debug, Clone, Default)]
struct DetectionState {
    // --- LiDAR ---
    /// Whether LiDAR sees a person-sized cluster.
    lidar_detected: bool,
    /// Distance to the closest person-sized cluster (m).
    lidar_distance: f64,
    /// Bearing angle to the cluster (rad, 0 = forward, + = left).
    lidar_bearing: f64,
    /// Diagnostic: total points in last LiDAR message.
    lidar_total_points: usize,
    /// Diagnostic: points that passed filtering.
    lidar_filtered_points: usize,
    /// Diagnostic: number of LiDAR messages received.
    lidar_msg_count: u64,

    // --- Camera ---
    /// Whether the camera sees the intruder's red color.
    camera_detected: bool,
    /// Horizontal bearing offset from camera center (rad).
    camera_bearing: f64,
    /// Fraction of image covered by red pixels (0..1).
    camera_red_ratio: f64,
    /// Diagnostic: red pixel count in last frame.
    camera_red_count: usize,
    /// Diagnostic: number of camera messages received.
    camera_msg_count: u64,

    // --- Fused ---
    /// Previous distance measurement for velocity estimation.
    prev_distance: f64,
    /// Timestamp of previous distance measurement.
    prev_time: Option<Instant>,
    /// Smoothed distance (EMA filtered).
    smoothed_distance: f64,
    /// Smoothed bearing (EMA filtered).
    smoothed_bearing: f64,
    /// Smoothed velocity (EMA filtered).
    smoothed_velocity: f64,
}

// ============================================================
// LiDAR Processing
// ============================================================

/// Simple point cloud clustering for intruder detection.
///
/// Strategy: Parse PointCloud2 data, filter out ground points (z < 0.15),
/// find points within a "person height" band (0.3–2.0m), and cluster
/// spatially. Return the bearing and distance to the largest person-sized
/// cluster.
fn process_lidar(msg: &PointCloud2) -> (usize, usize, Option<(f64, f64)>) {
    // Returns: (total_points, filtered_points, detection_result)
    let point_step = msg.point_step as usize;
    let data = &msg.data;

    if point_step == 0 || data.is_empty() {
        return (0, 0, None);
    }

    // Find field offsets for x, y, z
    let (mut x_off, mut y_off, mut z_off) = (0usize, 4usize, 8usize);
    for field in &msg.fields {
        match field.name.as_str() {
            "x" => x_off = field.offset as usize,
            "y" => y_off = field.offset as usize,
            "z" => z_off = field.offset as usize,
            _ => {}
        }
    }

    let num_points = data.len() / point_step;
    let mut person_points: Vec<(f64, f64, f64)> = Vec::new();
    let mut _nan_count = 0usize;
    let mut _dist_filtered = 0usize;
    let mut _z_filtered = 0usize;

    for i in 0..num_points {
        let base = i * point_step;
        if base + z_off + 4 > data.len() {
            break;
        }

        let x = f32::from_le_bytes([
            data[base + x_off],
            data[base + x_off + 1],
            data[base + x_off + 2],
            data[base + x_off + 3],
        ]) as f64;
        let y = f32::from_le_bytes([
            data[base + y_off],
            data[base + y_off + 1],
            data[base + y_off + 2],
            data[base + y_off + 3],
        ]) as f64;
        let z = f32::from_le_bytes([
            data[base + z_off],
            data[base + z_off + 1],
            data[base + z_off + 2],
            data[base + z_off + 3],
        ]) as f64;

        if x.is_nan() || y.is_nan() || z.is_nan() {
            _nan_count += 1;
            continue;
        }
        // Only look forward (x > 0) — ignore points behind the robot
        // Min distance 0.8m to skip robot's own body/legs (~0.3m away)
        let dist = (x * x + y * y).sqrt();
        if dist > 20.0 || dist < 0.8 || x < 0.0 {
            _dist_filtered += 1;
            continue;
        }
        // Person-height band: LiDAR is at ~0.42m above ground.
        // Ground is at z ≈ -0.42, person torso at z ≈ -0.1 to +1.2
        if z < -0.5 || z > 1.8 {
            _z_filtered += 1;
            continue;
        }

        person_points.push((x, y, z));
    }

    let filtered_count = person_points.len();

    if person_points.is_empty() {
        return (num_points, 0, None);
    }

    // Simple clustering: find the densest region.
    let mut sectors: Vec<(f64, f64, usize)> = Vec::new();
    let sector_width = 10.0_f64.to_radians();

    for &(x, y, _z) in &person_points {
        let bearing = y.atan2(x);
        let dist = (x * x + y * y).sqrt();
        let sector_idx = ((bearing + std::f64::consts::PI) / sector_width) as usize;

        if let Some(s) = sectors.iter_mut().find(|s| {
            let s_idx = ((s.1 + std::f64::consts::PI) / sector_width) as usize;
            s_idx == sector_idx
        }) {
            s.0 += dist;
            s.2 += 1;
        } else {
            sectors.push((dist, bearing, 1));
        }
    }

    let best = match sectors.iter().max_by_key(|s| s.2) {
        Some(b) => b,
        None => return (num_points, filtered_count, None),
    };

    if best.2 < 3 {
        info!("[go2_detector] LiDAR: best sector has only {} points (need ≥3)", best.2);
        return (num_points, filtered_count, None);
    }

    let avg_dist = best.0 / best.2 as f64;
    (num_points, filtered_count, Some((avg_dist, best.1)))
}

// ============================================================
// Camera Processing
// ============================================================

/// Detect bright red regions in the camera image.
///
/// The intruder model is colored bright red (R≈0.95, G≈0.2, B≈0.1).
/// We scan the RGB image for pixels where R > 150 && G < 100 && B < 100
/// and compute the horizontal centroid → bearing offset.
fn process_camera(msg: &Image) -> Option<(f64, f64)> {
    let width = msg.width as usize;
    let height = msg.height as usize;

    if width == 0 || height == 0 || msg.data.is_empty() {
        return None;
    }

    // Determine pixel stride based on encoding
    let channels: usize = match msg.encoding.as_str() {
        "rgb8" | "bgr8" => 3,
        "rgba8" | "bgra8" => 4,
        _ => 3, // default guess
    };
    let is_bgr = msg.encoding.starts_with("bgr");

    let mut red_sum_x: f64 = 0.0;
    let mut red_count: usize = 0;
    let step = msg.step as usize;

    for row in 0..height {
        for col in 0..width {
            let idx = row * step + col * channels;
            if idx + 2 >= msg.data.len() {
                continue;
            }

            let (r, g, b) = if is_bgr {
                (msg.data[idx + 2], msg.data[idx + 1], msg.data[idx])
            } else {
                (msg.data[idx], msg.data[idx + 1], msg.data[idx + 2])
            };

            // Red detection threshold
            if r > 150 && g < 100 && b < 100 {
                red_sum_x += col as f64;
                red_count += 1;
            }
        }
    }

    if red_count < 20 {
        return None; // Too few red pixels
    }

    let red_ratio = red_count as f64 / (width * height) as f64;
    let centroid_x = red_sum_x / red_count as f64;

    // Convert pixel centroid to bearing angle.
    // Camera HFOV = 1.5 rad (from URDF), image center = width/2
    let hfov = 1.5_f64;
    let center_x = width as f64 / 2.0;
    let bearing = -(centroid_x - center_x) / (width as f64) * hfov;

    Some((bearing, red_ratio))
}

// ============================================================
// ROS 2 Node
// ============================================================

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let ctx = Context::default_from_env()?;
    let mut executor = ctx.create_basic_executor();
    let node = executor.create_node("go2_detector")?;

    // ---- Shared state ----
    let state = Arc::new(Mutex::new(DetectionState::default()));

    // ---- Publishers ----
    let bearing_pub = node.create_publisher::<Vector3>("/go2/intruder_bearing")?;
    let detected_pub = node.create_publisher::<Bool>("/go2/intruder_detected")?;

    // ---- LiDAR Subscriber ----
    let state_lidar = Arc::clone(&state);
    let _lidar_sub = node.create_subscription::<PointCloud2, _>(
        "/lidar/points",
        move |msg: PointCloud2| {
            let (total, filtered, result) = process_lidar(&msg);
            let mut s = state_lidar.lock().unwrap();
            s.lidar_total_points = total;
            s.lidar_filtered_points = filtered;
            s.lidar_msg_count += 1;
            if let Some((distance, bearing)) = result {
                s.lidar_detected = true;
                s.lidar_distance = distance;
                s.lidar_bearing = bearing;
            } else {
                s.lidar_detected = false;
            }
        },
    )?;

    // ---- Camera Subscriber ----
    let state_camera = Arc::clone(&state);
    let _camera_sub = node.create_subscription::<Image, _>(
        "/camera/image_raw",
        move |msg: Image| {
            let result = process_camera(&msg);
            let mut s = state_camera.lock().unwrap();
            s.camera_msg_count += 1;
            if let Some((bearing, ratio)) = result {
                s.camera_detected = true;
                s.camera_bearing = bearing;
                s.camera_red_ratio = ratio;
                s.camera_red_count = (ratio * (msg.width as f64 * msg.height as f64)) as usize;
            } else {
                s.camera_detected = false;
                s.camera_red_count = 0;
            }
        },
    )?;

    // ---- Fusion Timer — 10 Hz ----
    let state_tick = Arc::clone(&state);
    let mut last_log = Instant::now();

    let _timer = node.create_timer_repeating(Duration::from_millis(100), move || {
        let mut s = state_tick.lock().unwrap();

        // Fuse: require at least one sensor to detect
        let detected = s.lidar_detected || s.camera_detected;

        // Compute raw velocity from distance changes
        let now = Instant::now();
        let mut raw_velocity = 0.0;
        if let Some(prev_t) = s.prev_time {
            let dt = now.duration_since(prev_t).as_secs_f64();
            if dt > 0.05 && s.lidar_detected {
                raw_velocity = (s.lidar_distance - s.prev_distance) / dt;
                // Clamp raw velocity to physically plausible range
                raw_velocity = raw_velocity.clamp(-5.0, 5.0);
            }
        }
        if s.lidar_detected {
            s.prev_distance = s.lidar_distance;
            s.prev_time = Some(now);
        }

        // Choose best bearing: if both detect, prefer camera for bearing
        let raw_bearing = if s.lidar_detected && s.camera_detected {
            0.3 * s.lidar_bearing + 0.7 * s.camera_bearing
        } else if s.camera_detected {
            s.camera_bearing
        } else if s.lidar_detected {
            s.lidar_bearing
        } else {
            s.smoothed_bearing // hold previous
        };

        let raw_distance = s.lidar_distance;

        // Reject bearing outliers: if bearing jumps >60° from smoothed, it's
        // likely the LiDAR picked a different cluster — ignore the jump.
        let bearing_to_use = if s.smoothed_bearing != 0.0
            && (raw_bearing - s.smoothed_bearing).abs() > 1.05
        {
            s.smoothed_bearing // hold previous bearing
        } else {
            raw_bearing
        };

        // Apply heavy exponential moving average smoothing (alpha = 0.1)
        // The LiDAR bearing is very noisy — needs aggressive filtering
        let alpha = 0.1;
        if s.smoothed_distance == 0.0 {
            // First measurement — initialize
            s.smoothed_distance = raw_distance;
            s.smoothed_bearing = bearing_to_use;
            s.smoothed_velocity = 0.0;
        } else {
            s.smoothed_distance = alpha * raw_distance + (1.0 - alpha) * s.smoothed_distance;
            s.smoothed_bearing = alpha * bearing_to_use + (1.0 - alpha) * s.smoothed_bearing;
            s.smoothed_velocity = alpha * raw_velocity + (1.0 - alpha) * s.smoothed_velocity;
        }

        // Publish detection state
        let _ = detected_pub.publish(Bool { data: detected });

        if detected {
            let msg = Vector3 {
                x: s.smoothed_distance,   // distance (m)
                y: s.smoothed_bearing,    // bearing (rad)
                z: s.smoothed_velocity,   // velocity estimate (m/s)
            };
            let _ = bearing_pub.publish(msg);
        }

        if last_log.elapsed() > Duration::from_secs(2) {
            info!(
                "[go2_detector] STATUS: detected={} | LiDAR: msgs={} pts={}/{} det={} | Camera: msgs={} red_px={} det={}",
                detected,
                s.lidar_msg_count,
                s.lidar_filtered_points,
                s.lidar_total_points,
                s.lidar_detected,
                s.camera_msg_count,
                s.camera_red_count,
                s.camera_detected
            );
            if detected {
                info!(
                    "[go2_detector] INTRUDER: dist={:.2}m bearing={:.1}° vel={:.2}m/s (smooth)",
                    s.smoothed_distance,
                    s.smoothed_bearing.to_degrees(),
                    s.smoothed_velocity,
                );
            }
            last_log = Instant::now();
        }
    })?;

    info!("[go2_detector] Node started — LiDAR + Camera intruder detection");
    info!("  Subscribing: /lidar/points, /camera/image_raw");
    info!("  Publishing:  /go2/intruder_bearing, /go2/intruder_detected");

    let _ = executor.spin(rclrs::SpinOptions::default());
    Ok(())
}
