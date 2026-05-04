//! # go2_behavior
//!
//! High-level **Behavior Finite-State Machine** for the Unitree Go2 quadruped.
//!
//! ## States
//!
//! ```text
//!                    ┌──────────────┐
//!            ┌──────►│     Idle     │◄──────────────────────────┐
//!            │       └──────┬───────┘                           │
//!            │   cmd:stand  │                              e-stop clear
//!            │              ▼                                    │
//!            │       ┌──────────────┐                    ┌──────┴───────┐
//!            │       │   StandUp    │──── any danger ───►│ EmergencyStop│
//!            │       └──────┬───────┘                    └──────────────┘
//!            │   stable     │                                    ▲
//!            │              ▼                                    │
//!            │       ┌──────────────┐                     tilt/collision
//!            │       │     Trot     │──────────────────────────►│
//!            │       └──────┬───────┘
//!            │  cmd:patrol  │
//!            │              ▼
//!            │       ┌──────────────┐
//!            └───────│   Patrol     │
//!        cmd:idle     └──────────────┘
//! ```
//!
//! ## Topics
//!
//! | Direction  | Topic                    | Type                        |
//! |------------|--------------------------|-----------------------------|
//! | Subscribes | `/imu/data`              | `sensor_msgs/Imu`           |
//! | Subscribes | `/odom`                  | `nav_msgs/Odometry`         |
//! | Subscribes | `/joint_states`          | `sensor_msgs/JointState`    |
//! | Subscribes | `/go2/cmd_behavior`      | `std_msgs/String`           |
//! | Publishes  | `/cmd_vel`               | `geometry_msgs/Twist`       |
//! | Publishes  | `/go2/behavior_state`    | `std_msgs/String`           |

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::Result;
use log::{info, warn};

use geometry_msgs::msg::Twist;
use nav_msgs::msg::Odometry;
use rclrs::{Context, CreateBasicExecutor};
use sensor_msgs::msg::Imu;
use sensor_msgs::msg::JointState;
use std_msgs::msg::String as RosString;

// ============================================================
// Behavior FSM State
// ============================================================

/// All possible high-level behavior states.
#[derive(Debug, Clone, PartialEq)]
pub enum BehaviorState {
    /// Motors relaxed, robot sitting or lying.
    Idle,
    /// Robot is rising to a standing pose.
    StandUp,
    /// Robot is trotting (open-loop velocity following).
    Trot,
    /// Executing a pre-defined patrol waypoint sequence.
    Patrol { waypoint_index: usize },
    /// Safety halt — disable all motion immediately.
    EmergencyStop { reason: std::string::String },
    /// Pass-through mode for manual teleop.
    Manual,
}

impl BehaviorState {
    pub fn label(&self) -> &'static str {
        match self {
            BehaviorState::Idle => "Idle",
            BehaviorState::StandUp => "StandUp",
            BehaviorState::Trot => "Trot",
            BehaviorState::Patrol { .. } => "Patrol",
            BehaviorState::EmergencyStop { .. } => "EmergencyStop",
            BehaviorState::Manual => "Manual",
        }
    }
}

// ============================================================
// Shared sensor state
// ============================================================

/// Snapshot of the latest sensor readings, shared across ROS callbacks.
#[derive(Debug, Default, Clone)]
pub struct SensorState {
    /// Roll angle from IMU (radians).
    pub roll: f64,
    /// Pitch angle from IMU (radians).
    pub pitch: f64,
    /// Linear velocity from odometry.
    pub lin_vel_x: f64,
    pub lin_vel_y: f64,
    /// Whether the robot is upright enough for locomotion.
    pub is_stable: bool,
}

impl SensorState {
    /// Safety check: abort if the robot tilts more than 30 degrees.
    pub fn check_tilt(&self) -> bool {
        self.roll.abs() < 0.52 && self.pitch.abs() < 0.52
    }
}

// ============================================================
// Behavior FSM
// ============================================================

pub struct BehaviorFsm {
    state: BehaviorState,
    /// Timestamp when we entered the current state.
    state_entered_at: Instant,
    /// Desired forward velocity while trotting (m/s).
    target_vel_x: f64,
    /// Desired angular velocity while trotting (rad/s).
    target_ang_z: f64,
    /// Patrol waypoints: (target_yaw, duration_secs).
    patrol_waypoints: Vec<(f64, f64)>,
    /// Velocity from manual teleop
    manual_vel: Twist,
}

impl BehaviorFsm {
    pub fn new() -> Self {
        Self {
            state: BehaviorState::Idle,
            state_entered_at: Instant::now(),
            target_vel_x: 0.3,
            target_ang_z: 0.0,
            patrol_waypoints: vec![
                (0.0, 3.0),
                (std::f64::consts::PI / 2.0, 2.0),
                (std::f64::consts::PI, 3.0),
                (-std::f64::consts::PI / 2.0, 2.0),
            ],
            manual_vel: Twist::default(),
        }
    }

    /// Attempt a state transition from the outside (e.g., operator command).
    pub fn transition(&mut self, cmd: &str) {
        let next = match (cmd.trim(), &self.state) {
            ("stand", BehaviorState::Idle) => Some(BehaviorState::StandUp),
            ("trot", BehaviorState::StandUp) => Some(BehaviorState::Trot),
            ("patrol", BehaviorState::Trot) => {
                Some(BehaviorState::Patrol { waypoint_index: 0 })
            }
            ("manual", _) => Some(BehaviorState::Manual),
            ("idle", _) => Some(BehaviorState::Idle),
            ("estop", _) => Some(BehaviorState::EmergencyStop {
                reason: "operator command".into(),
            }),
            ("clear", BehaviorState::EmergencyStop { .. }) => Some(BehaviorState::Idle),
            _ => {
                warn!(
                    "[BehaviorFSM] Ignored command '{}' from state '{}'",
                    cmd,
                    self.state.label()
                );
                None
            }
        };

        if let Some(next_state) = next {
            info!(
                "[BehaviorFSM] {} → {}",
                self.state.label(),
                next_state.label()
            );
            self.state = next_state;
            self.state_entered_at = Instant::now();
        }
    }

    /// Called every control tick. Returns the `Twist` command to publish.
    pub fn tick(&mut self, sensors: &SensorState) -> Twist {
        // ---- Safety watchdog ----------------------------------------
        if !sensors.check_tilt() && self.state != BehaviorState::Idle {
            if !matches!(self.state, BehaviorState::EmergencyStop { .. }) {
                warn!("[BehaviorFSM] Excessive tilt detected — E-Stop!");
                self.state = BehaviorState::EmergencyStop {
                    reason: format!(
                        "tilt (roll={:.2} pitch={:.2})",
                        sensors.roll, sensors.pitch
                    ),
                };
                self.state_entered_at = Instant::now();
            }
        }

        let elapsed = self.state_entered_at.elapsed();

        match &self.state.clone() {
            // ---- Idle: zero velocity --------------------------------
            BehaviorState::Idle => zero_twist(),

            // ---- StandUp: ramp joints to stand pose, then go Trot --
            BehaviorState::StandUp => {
                // After 2 s we assume stand is achieved (gait node handles
                // the actual joint motion). Auto-transition to Trot.
                if elapsed > Duration::from_secs(2) && sensors.is_stable {
                    info!("[BehaviorFSM] Stand complete → Trot");
                    self.state = BehaviorState::Trot;
                    self.state_entered_at = Instant::now();
                }
                zero_twist()
            }

            // ---- Trot: publish constant forward velocity -----------
            BehaviorState::Trot => make_twist(self.target_vel_x, self.target_ang_z),

            // ---- Patrol: cycle through waypoints -------------------
            BehaviorState::Patrol { waypoint_index } => {
                let wp_idx = *waypoint_index;
                let (target_yaw, duration) = self.patrol_waypoints[wp_idx];
                if elapsed.as_secs_f64() > duration {
                    let next_idx = (wp_idx + 1) % self.patrol_waypoints.len();
                    info!(
                        "[BehaviorFSM] Patrol waypoint {} → {}",
                        wp_idx, next_idx
                    );
                    self.state = BehaviorState::Patrol {
                        waypoint_index: next_idx,
                    };
                    self.state_entered_at = Instant::now();
                }
                // Simple: just rotate toward waypoint yaw
                make_twist(0.2, target_yaw * 0.3)
            }

            // ---- EmergencyStop: broadcast zero velocity ------------
            BehaviorState::EmergencyStop { reason } => {
                if elapsed.as_millis() % 1000 < 50 {
                    warn!("[BehaviorFSM] E-STOP active: {}", reason);
                }
                zero_twist()
            }

            // ---- Manual: pass through teleop velocity --------------
            BehaviorState::Manual => self.manual_vel.clone(),
        }
    }
}

// ============================================================
// Twist helpers
// ============================================================

fn zero_twist() -> Twist {
    Twist::default()
}

fn make_twist(lin_x: f64, ang_z: f64) -> Twist {
    let mut t = Twist::default();
    t.linear.x = lin_x;
    t.angular.z = ang_z;
    t
}

// ============================================================
// ROS 2 Node
// ============================================================

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let ctx = Context::default_from_env()?;
    let mut executor = ctx.create_basic_executor();
    let node: rclrs::Node = executor.create_node("go2_behavior")?;

    // ------------------------------------------------------------------ //
    // Shared state (sensor readings + FSM)
    // ------------------------------------------------------------------ //
    let sensors = Arc::new(Mutex::new(SensorState::default()));
    let fsm = Arc::new(Mutex::new(BehaviorFsm::new()));

    // ------------------------------------------------------------------ //
    // Publishers
    // ------------------------------------------------------------------ //
    let cmd_vel_pub = node.create_publisher::<Twist>("/go2/cmd_vel")?;
    let state_pub = node.create_publisher::<RosString>("/go2/behavior_state")?;

    // ------------------------------------------------------------------ //
    // Subscribers
    // ------------------------------------------------------------------ //

    // IMU → update roll / pitch
    let sensors_imu = Arc::clone(&sensors);
    let _imu_sub = node.create_subscription::<Imu, _>(
        "/imu/data",
        move |msg: Imu| {
            // Quaternion → roll/pitch (simplified; replace with proper math)
            let qx = msg.orientation.x;
            let qy = msg.orientation.y;
            let qz = msg.orientation.z;
            let qw = msg.orientation.w;
            let sinr = 2.0 * (qw * qx + qy * qz);
            let cosr = 1.0 - 2.0 * (qx * qx + qy * qy);
            let roll = sinr.atan2(cosr);
            let sinp = 2.0 * (qw * qy - qz * qx);
            let pitch = sinp.clamp(-1.0, 1.0).asin();

            let mut s = sensors_imu.lock().unwrap();
            s.roll = roll;
            s.pitch = pitch;
            s.is_stable = s.check_tilt();
        },
    )?;

    // Odometry → linear velocity
    let sensors_odom = Arc::clone(&sensors);
    let _odom_sub = node.create_subscription::<Odometry, _>(
        "/odom",
        move |msg: Odometry| {
            let mut s = sensors_odom.lock().unwrap();
            s.lin_vel_x = msg.twist.twist.linear.x;
            s.lin_vel_y = msg.twist.twist.linear.y;
        },
    )?;

    // Operator commands (simple string: "stand" | "trot" | "patrol" | "idle" | "estop" | "clear")
    let fsm_cmd = Arc::clone(&fsm);
    let _cmd_sub = node.create_subscription::<RosString, _>(
        "/go2/cmd_behavior",
        move |msg: RosString| {
            fsm_cmd.lock().unwrap().transition(&msg.data);
        },
    )?;

    // Manual teleop velocity → pass-through
    let fsm_manual = Arc::clone(&fsm);
    let _teleop_sub = node.create_subscription::<Twist, _>(
        "/cmd_vel",
        move |msg: Twist| {
            fsm_manual.lock().unwrap().manual_vel = msg;
        },
    )?;

    // JointState subscription (stubbed — extend to monitor joint positions)
    let _js_sub = node.create_subscription::<JointState, _>(
        "/joint_states",
        |_msg: JointState| {
            // TODO: parse joint positions to verify stand/sit poses
        },
    )?;

    // ------------------------------------------------------------------ //
    // Control timer — 50 Hz
    // ------------------------------------------------------------------ //
    let fsm_tick = Arc::clone(&fsm);
    let sensors_tick = Arc::clone(&sensors);

    let mut last_log = Instant::now();
    let _timer = node.create_timer_repeating(Duration::from_millis(20), move || {
        let sensor_snapshot = sensors_tick.lock().unwrap().clone();
        let mut f = fsm_tick.lock().unwrap();
        let twist = f.tick(&sensor_snapshot);
        let lin_x = twist.linear.x;

        // Publish velocity command
        let _ = cmd_vel_pub.publish(twist);

        // Publish FSM state
        let state_label = f.state.label();
        let _ = state_pub.publish(RosString { data: state_label.into() });

        if last_log.elapsed() > Duration::from_secs(1) {
            println!("[go2_behavior] HEARTBEAT: State = {}, VelX = {:.2}", state_label, lin_x);
            last_log = Instant::now();
        }
    })?;

    info!("[go2_behavior] Node started — waiting for commands on /go2/cmd_behavior");
    info!("  Commands: stand | trot | patrol | idle | estop | clear");

    let _ = executor.spin(rclrs::SpinOptions::default());
    Ok(())
}
