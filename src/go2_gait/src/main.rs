//! # go2_gait
//!
//! **Trot gait generator** and **2-link inverse-kinematics** stub for the
//! Unitree Go2.
//!
//! ## What it does
//!
//! 1. Subscribes to `/cmd_vel` (`geometry_msgs/Twist`)
//! 2. Runs a **trot gait scheduler** at 500 Hz — produces foot target
//!    positions in the body frame via a sinusoidal step trajectory.
//! 3. Solves **per-leg IK** (2-link planar, hip abduction decoupled) to
//!    convert foot targets → joint angles (hip, thigh, calf).
//! 4. Publishes a `Float64MultiArray` to
//!    `/go2_joint_controller/commands` (12 values, same joint order as
//!    `ros2_controllers.yaml`).
//!
//! ## Joint order (matches controller config)
//! ```text
//!  0  FL_hip    1  FL_thigh   2  FL_calf
//!  3  FR_hip    4  FR_thigh   5  FR_calf
//!  6  RL_hip    7  RL_thigh   8  RL_calf
//!  9  RR_hip   10  RR_thigh  11  RR_calf
//! ```
//!
//! ## Trot diagonal pairs
//! ```text
//!  Pair A (swing together): FL + RR
//!  Pair B (swing together): FR + RL
//! ```

use std::f64::consts::PI;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::Result;
use log::{debug, info};

use geometry_msgs::msg::Twist;
use rclrs::{Context, CreateBasicExecutor};
use std_msgs::msg::Float64MultiArray;

// ============================================================
// Physical constants (metres)
// ============================================================

/// Thigh link length (L1)
const L1: f64 = 0.213;
/// Calf link length (L2)
const L2: f64 = 0.213;
/// Lateral hip offset from body centre-line to hip joint
const HIP_Y_OFFSET: f64 = 0.08;
/// Nominal body-to-ground height during stance (m)
const NOMINAL_HEIGHT: f64 = 0.30;
/// Step height during swing phase (m)
const STEP_HEIGHT: f64 = 0.06;

// ============================================================
// Gait parameters
// ============================================================

/// Trot duty cycle (fraction of cycle in stance)
const DUTY_CYCLE: f64 = 0.5;

// ============================================================
// 2-Link Planar IK
// ============================================================

/// Solve the sagittal-plane (Y-Z) IK for one leg.
///
/// # Arguments
/// * `lateral` — signed lateral offset of foot from hip joint (+ = outward, m)
/// * `forward` — forward offset in body frame (m)
/// * `down`    — downward offset from hip joint (positive = below, m)
///
/// # Returns
/// `(hip_angle, thigh_angle, calf_angle)` in radians.
///
/// Returns the default stand pose if the target is unreachable.
fn solve_ik(lateral: f64, forward: f64, down: f64) -> (f64, f64, f64) {
    // Decouple hip (abduction) from the sagittal plane
    let hip = lateral.atan2(down.abs().max(1e-6));

    // Effective reach in the sagittal plane after hip rotation
    let r_sag = (forward * forward + down * down).sqrt();

    // Clamp to reachable workspace
    let r_clamped = r_sag.clamp(0.01, L1 + L2 - 0.01);

    // Law of cosines: knee angle
    let cos_calf = (r_clamped * r_clamped - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
    let calf = -(cos_calf.clamp(-1.0, 1.0).acos()); // negative = knee bends backward

    // Thigh angle: positive rotation moves link BACKWARDS (-X) because it points -Z.
    // To reach forward (alpha > 0), we must rotate LESS backward, so we subtract alpha.
    let alpha = forward.atan2(down.abs().max(1e-6));
    let beta_cos = (r_clamped * r_clamped + L1 * L1 - L2 * L2) / (2.0 * r_clamped * L1);
    let beta = beta_cos.clamp(-1.0, 1.0).acos();
    let thigh = -alpha + beta;

    (hip, thigh, calf)
}

// ============================================================
// Trot Gait Scheduler
// ============================================================

/// Phase offset for each leg in the trot gait (normalised 0..1).
/// Trot: FL+RR in phase, FR+RL offset by 0.5.
#[derive(Clone, Copy)]
enum Leg {
    FL = 0,
    FR = 1,
    RL = 2,
    RR = 3,
}

const LEG_PHASE_OFFSET: [f64; 4] = [
    0.0, // FL
    0.5, // FR
    0.5, // RL
    0.0, // RR
];

/// Lateral sign for each leg (positive = left body side)
const LEG_LATERAL_SIGN: [f64; 4] = [1.0, -1.0, 1.0, -1.0]; // FL, FR, RL, RR

pub struct GaitController {
    /// Normalised gait phase [0, 1)
    phase: f64,
    /// Gait cycle frequency (Hz) — derived from cmd_vel
    freq: f64,
    /// Desired forward velocity (m/s)
    vel_x: f64,
    /// Desired angular velocity (rad/s)
    ang_z: f64,
}

impl GaitController {
    pub fn new() -> Self {
        Self {
            phase: 0.0,
            freq: 2.0,  // 2 Hz trot by default
            vel_x: 0.0,
            ang_z: 0.0,
        }
    }

    pub fn update_command(&mut self, vel_x: f64, ang_z: f64) {
        self.vel_x = vel_x;
        self.ang_z = ang_z;
        // Scale frequency with speed: 1–4 Hz over 0–0.6 m/s
        let speed = vel_x.abs().max(ang_z.abs() * 0.2);
        self.freq = (1.0 + speed * 5.0).clamp(1.0, 4.0);
    }

    /// Advance the gait phase by `dt` seconds and compute 12 joint angles.
    pub fn step(&mut self, dt: f64) -> [f64; 12] {
        let moving = self.vel_x.abs() > 0.01 || self.ang_z.abs() > 0.01;
        if moving {
            self.phase = (self.phase + self.freq * dt) % 1.0;
        } else {
            self.phase = 0.0; // Reset to standing pose when not moving
        }

        let mut joints = [0.0f64; 12];

        for leg_idx in [Leg::FL, Leg::FR, Leg::RL, Leg::RR] {
            let idx = leg_idx as usize;
            let leg_phase = (self.phase + LEG_PHASE_OFFSET[idx]) % 1.0;
            let lat_sign = LEG_LATERAL_SIGN[idx];

            // ---- Foot trajectory in body frame ---
            let (foot_fwd, foot_lat, foot_down) = if !moving {
                (0.0, lat_sign * HIP_Y_OFFSET, NOMINAL_HEIGHT)
            } else if leg_phase < DUTY_CYCLE {
                // Stance: foot sweeps backward proportional to velocity
                let t = leg_phase / DUTY_CYCLE; // 0→1 through stance
                // Turn using lat_sign for differential drive (left side sweeps opposite to right)
                let fwd = self.vel_x * (0.5 - t) * 0.1
                    - lat_sign * self.ang_z * 0.05 * (0.5 - t);
                (fwd, lat_sign * HIP_Y_OFFSET, NOMINAL_HEIGHT)
            } else {
                // Swing: sinusoidal arc
                let t = (leg_phase - DUTY_CYCLE) / (1.0 - DUTY_CYCLE); // 0→1 through swing
                let fwd = self.vel_x * (t - 0.5) * 0.1
                    - lat_sign * self.ang_z * 0.05 * (t - 0.5);
                let height = NOMINAL_HEIGHT - STEP_HEIGHT * (PI * t).sin();
                (fwd, lat_sign * HIP_Y_OFFSET, height)
            };

            let (hip, thigh, calf) = solve_ik(foot_lat, foot_fwd, foot_down);

            let base = idx * 3;
            joints[base]     = hip;
            joints[base + 1] = thigh;
            joints[base + 2] = calf;
        }

        joints
    }
}

// ============================================================
// ROS 2 Node
// ============================================================

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let ctx = Context::default_from_env()?;
    let mut executor = ctx.create_basic_executor();
    let node: rclrs::Node = executor.create_node("go2_gait")?;

    // ---- Shared state -------------------------------------------
    let gait = Arc::new(Mutex::new(GaitController::new()));

    // ---- Publisher: joint position commands ---------------------
    let joint_pub = node.create_publisher::<Float64MultiArray>(
        "/go2_joint_controller/commands",
    )?;

    // ---- Subscriber: cmd_vel ------------------------------------
    let gait_cmd = Arc::clone(&gait);
    let _cmd_sub = node.create_subscription::<Twist, _>(
        "/go2/cmd_vel",
        move |msg: Twist| {
            debug!("[go2_gait] Received cmd_vel: x={:.2}, z={:.2}", msg.linear.x, msg.angular.z);
            gait_cmd
                .lock()
                .unwrap()
                .update_command(msg.linear.x, msg.angular.z);
        },
    )?;

    // ---- Control timer at 500 Hz --------------------------------
    let gait_tick = Arc::clone(&gait);
    let _dt = 1.0 / 500.0_f64;

    let mut last_log = Instant::now();
    let _timer = node.create_timer_repeating(Duration::from_micros(2000), move || {
        let joints = gait_tick.lock().unwrap().step(0.002);

        if last_log.elapsed() > Duration::from_secs(1) {
            debug!("[go2_gait] Publishing 12 joints. Joint 0: {:.3}", joints[0]);
            last_log = Instant::now();
        }

        let msg = Float64MultiArray {
            data: joints.to_vec(),
            ..Default::default()
        };

        let _ = joint_pub.publish(msg);
    })?;

    info!("[go2_gait] Node started — trot gait generator running at 500 Hz");
    info!("  Subscribing: /cmd_vel");
    info!("  Publishing:  /go2_joint_controller/commands (12 joints)");

    let _ = executor.spin(rclrs::SpinOptions::default());
    Ok(())
}

// ============================================================
// Unit tests — run with: cargo test -p go2_gait
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ik_nominal_stance() {
        // At the default stance position the foot should be directly below
        // the hip at NOMINAL_HEIGHT with no lateral offset.
        let (hip, thigh, calf) = solve_ik(0.0, 0.0, NOMINAL_HEIGHT);
        // Hip should be ~0 when no lateral offset
        assert!(hip.abs() < 0.01, "hip={}", hip);
        // Knee must be negative (bending)
        assert!(calf < 0.0, "calf={}", calf);
        // Verify round-trip: reconstructed foot position
        let y = L1 * thigh.sin() + L2 * (thigh + calf).sin();
        let z = L1 * thigh.cos() + L2 * (thigh + calf).cos();
        assert!((z - NOMINAL_HEIGHT).abs() < 0.005, "z={}", z);
        assert!(y.abs() < 0.01, "y={}", y);
    }

    #[test]
    fn test_gait_produces_12_joints() {
        let mut g = GaitController::new();
        g.update_command(0.3, 0.0);
        let joints = g.step(0.002);
        assert_eq!(joints.len(), 12);
    }

    #[test]
    fn test_ik_lateral_offset() {
        // A lateral offset should produce a non-zero hip angle
        let (hip, _, _) = solve_ik(0.05, 0.0, NOMINAL_HEIGHT);
        assert!(hip.abs() > 0.01, "hip should reflect lateral offset");
    }
}
