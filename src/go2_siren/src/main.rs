//! # go2_siren
//!
//! **Siren light controller** for the Unitree Go2 guard dog.
//!
//! Subscribes to `/go2/siren_cmd` (`std_msgs/Bool`) and toggles a flashing
//! red/blue marker on the robot's siren_link in Gazebo at ~3 Hz.
//!
//! Uses the Gazebo `/marker` service to create a glowing sphere attached
//! to the siren_link that changes color.
//!
//! ## Topics
//!
//! | Direction  | Topic               | Type            |
//! |------------|----------------------|-----------------|
//! | Subscribes | `/go2/siren_cmd`     | `std_msgs/Bool` |
//! | Publishes  | `/go2/siren_state`   | `std_msgs/Bool` |

use std::process::Command as ProcessCommand;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::Result;
use log::info;

use rclrs::{Context, CreateBasicExecutor};
use std_msgs::msg::Bool;
use std_msgs::msg::String as RosString;

// ============================================================
// Siren State
// ============================================================

#[derive(Debug, Clone)]
struct SirenState {
    /// Whether the siren is commanded ON.
    enabled: bool,
    /// Flash toggle (alternates for visual effect).
    flash_on: bool,
    /// Counter for flash timing.
    flash_counter: u32,
    /// Track previous state to only call gz service on changes.
    prev_flash_on: bool,
    prev_enabled: bool,
}

impl Default for SirenState {
    fn default() -> Self {
        Self {
            enabled: false,
            flash_on: false,
            flash_counter: 0,
            prev_flash_on: false,
            prev_enabled: false,
        }
    }
}

// ============================================================
// Gazebo marker control
// ============================================================

/// Create or update the siren marker sphere on the robot via the `/marker` service.
fn set_siren_marker(r: f32, g: f32, b: f32, visible: bool) {
    let scale = if visible { 0.06 } else { 0.001 }; // Shrink to invisible when off
    let req = format!(
        concat!(
            "action: ADD_MODIFY, ns: \"siren\", id: 1, type: SPHERE, ",
            "parent: \"go2::base_link\", ",
            "pose: {{position: {{x: -0.05, y: 0, z: 0.12}}}}, ",
            "scale: {{x: {s}, y: {s}, z: {s}}}, ",
            "material: {{",
            "ambient: {{r: {r}, g: {g}, b: {b}, a: 1}}, ",
            "diffuse: {{r: {r}, g: {g}, b: {b}, a: 1}}, ",
            "emissive: {{r: {er}, g: {eg}, b: {eb}, a: 1}}}}"
        ),
        s = scale,
        r = r, g = g, b = b,
        er = r, eg = g, eb = b
    );

    // Fire-and-forget — don't block on the response
    let _ = std::thread::spawn(move || {
        let output = ProcessCommand::new("gz")
            .args([
                "service",
                "-s", "/marker",
                "--reqtype", "gz.msgs.Marker",
                "--reptype", "gz.msgs.Empty",
                "--timeout", "200",
                "--req", &req,
            ])
            .output();
        if let Err(e) = output {
            eprintln!("[go2_siren] gz marker error: {}", e);
        }
    });
}

// ============================================================
// ROS 2 Node
// ============================================================

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let ctx = Context::default_from_env()?;
    let mut executor = ctx.create_basic_executor();
    let node = executor.create_node("go2_siren")?;

    // ---- Shared state ----
    let state = Arc::new(Mutex::new(SirenState::default()));

    // ---- Publishers ----
    let state_pub = node.create_publisher::<Bool>("/go2/siren_state")?;
    let visual_pub = node.create_publisher::<RosString>("/go2/siren_visual")?;

    // ---- Subscriber: siren on/off command ----
    let state_cmd = Arc::clone(&state);
    let _cmd_sub = node.create_subscription::<Bool, _>(
        "/go2/siren_cmd",
        move |msg: Bool| {
            let mut s = state_cmd.lock().unwrap();
            let was_enabled = s.enabled;
            s.enabled = msg.data;
            if msg.data && !was_enabled {
                info!("[go2_siren] 🚨 SIREN ACTIVATED");
                s.flash_counter = 0;
            } else if !msg.data && was_enabled {
                info!("[go2_siren] Siren deactivated");
                s.flash_on = false;
            }
        },
    )?;

    // ---- Flash timer — 10 Hz (toggles at ~1.7 Hz for visible flashing) ----
    let state_tick = Arc::clone(&state);
    let mut last_status_log = Instant::now();

    let _timer = node.create_timer_repeating(Duration::from_millis(100), move || {
        let mut s = state_tick.lock().unwrap();

        if s.enabled {
            s.flash_counter += 1;
            // Toggle every 3 ticks = 300ms = ~1.7 Hz flash rate
            if s.flash_counter % 3 == 0 {
                s.flash_on = !s.flash_on;
            }

            // Only call gz service when flash state changes
            if s.flash_on != s.prev_flash_on {
                if s.flash_on {
                    // Bright RED flash
                    set_siren_marker(1.0, 0.0, 0.0, true);
                } else {
                    // Bright BLUE flash
                    set_siren_marker(0.0, 0.2, 1.0, true);
                }
                s.prev_flash_on = s.flash_on;
            }

            let color = if s.flash_on { "RED" } else { "BLUE" };
            let _ = visual_pub.publish(RosString {
                data: format!("SIREN:{}", color),
            });
        } else {
            // Siren OFF — hide marker (only on transition)
            if s.prev_enabled {
                set_siren_marker(0.0, 0.0, 0.0, false);
                s.prev_flash_on = false;
            }
            s.flash_on = false;
            let _ = visual_pub.publish(RosString {
                data: "SIREN:OFF".to_string(),
            });
        }

        s.prev_enabled = s.enabled;

        // Publish siren state
        let _ = state_pub.publish(Bool { data: s.enabled });

        // Periodic log
        if s.enabled && last_status_log.elapsed() > Duration::from_secs(3) {
            info!(
                "[go2_siren] 🚨 Siren ACTIVE — flash={}",
                if s.flash_on { "RED" } else { "BLUE" }
            );
            last_status_log = Instant::now();
        }
    })?;

    info!("[go2_siren] Node started — siren light controller with Gazebo marker flash");
    info!("  Subscribing: /go2/siren_cmd");
    info!("  Publishing:  /go2/siren_state, /go2/siren_visual");

    let _ = executor.spin(rclrs::SpinOptions::default());
    Ok(())
}
