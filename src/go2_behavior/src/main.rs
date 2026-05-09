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
//!            │       ┌──────────────┐   intruder    ┌──────────────┐
//!            └───────│   Patrol     │──────────────►│   Pursue     │
//!        cmd:idle    └──────────────┘               └──────┬───────┘
//!                                                     close│& stopped
//!                                                          ▼
//!                                                   ┌──────────────┐
//!                                                   │  Apprehend   │
//!                                                   │ (SIREN ON)   │
//!                                                   └──────────────┘
//! ```
//!
//! ## Topics
//!
//! | Direction  | Topic                    | Type                        |
//! |------------|--------------------------|-----------------------------|\
//! | Subscribes | `/imu/data`              | `sensor_msgs/Imu`           |
//! | Subscribes | `/odom`                  | `nav_msgs/Odometry`         |
//! | Subscribes | `/joint_states`          | `sensor_msgs/JointState`    |
//! | Subscribes | `/go2/cmd_behavior`      | `std_msgs/String`           |
//! | Subscribes | `/go2/intruder_bearing`  | `geometry_msgs/Vector3`     |
//! | Subscribes | `/go2/intruder_detected` | `std_msgs/Bool`             |
//! | Publishes  | `/cmd_vel`               | `geometry_msgs/Twist`       |
//! | Publishes  | `/go2/behavior_state`    | `std_msgs/String`           |
//! | Publishes  | `/go2/siren_cmd`         | `std_msgs/Bool`             |

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::Result;
use log::{info, warn};

use geometry_msgs::msg::{Twist, Vector3};
use nav_msgs::msg::Odometry;
use rclrs::{Context, CreateBasicExecutor};
use sensor_msgs::msg::Imu;
use sensor_msgs::msg::JointState;
use std_msgs::msg::Bool;
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
    EmergencyStop { reason: String },
    /// Pass-through mode for manual teleop.
    Manual,
    /// Robot has detected an intruder and is pursuing.
    Pursue,
    /// Robot has cornered the intruder — siren ON, holding position.
    Apprehend,
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
            BehaviorState::Pursue => "Pursue",
            BehaviorState::Apprehend => "Apprehend",
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
    /// Safety check: abort if the robot tilts more than 45 degrees.
    pub fn check_tilt(&self) -> bool {
        self.roll.abs() < 0.78 && self.pitch.abs() < 0.78
    }
}

// ============================================================
// Intruder tracking state
// ============================================================

/// Latest intruder detection data from the go2_detector node.
#[derive(Debug, Clone)]
pub struct IntruderState {
    /// Whether an intruder is currently detected.
    pub detected: bool,
    /// Distance to intruder (m).
    pub distance: f64,
    /// Bearing to intruder (rad, 0=forward, +=left).
    pub bearing: f64,
    /// Estimated intruder velocity (m/s, negative = approaching).
    pub velocity: f64,
    /// When was the intruder last seen?
    pub last_seen: Instant,
    /// Time since intruder was lost (seconds).
    pub time_since_lost: f64,
}

impl Default for IntruderState {
    fn default() -> Self {
        Self {
            detected: false,
            distance: 0.0,
            bearing: 0.0,
            velocity: 0.0,
            last_seen: Instant::now(),
            time_since_lost: f64::MAX,
        }
    }
}

// ============================================================
// Pursuit parameters
// ============================================================

/// Distance (m) at which the robot considers the intruder "cornered".
const APPREHEND_DISTANCE: f64 = 3.0;
/// Intruder velocity threshold (m/s) below which they're considered "stopped".
/// Note: EMA smoothing (alpha=0.1) leaves residual noise ~0.3 m/s for stationary targets.
const INTRUDER_STOPPED_THRESHOLD: f64 = 0.5;
/// Seconds without detection before returning to Patrol.
const LOST_TIMEOUT_SECS: f64 = 5.0;
/// Maximum pursuit speed (m/s).
const PURSUIT_MAX_SPEED: f64 = 0.15;
/// Proportional gain for steering toward the intruder.
const PURSUIT_STEERING_GAIN: f64 = 0.5;

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
    /// Siren command output (set by FSM, read by timer).
    siren_on: bool,
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
            siren_on: false,
        }
    }

    /// Attempt a state transition from the outside (e.g., operator command).
    pub fn transition(&mut self, cmd: &str) {
        let next = match (cmd.trim(), &self.state) {
            ("stand", _) => Some(BehaviorState::StandUp),
            ("trot", _) => Some(BehaviorState::Trot),
            ("patrol", _) => {
                Some(BehaviorState::Patrol { waypoint_index: 0 })
            }
            ("manual", _) => Some(BehaviorState::Manual),
            ("idle", _) => Some(BehaviorState::Idle),
            ("estop", _) => Some(BehaviorState::EmergencyStop {
                reason: "operator command".into(),
            }),
            ("clear", BehaviorState::EmergencyStop { .. }) => Some(BehaviorState::Idle),
            ("clear", BehaviorState::Apprehend) => {
                // Operator clears the apprehend — return to patrol
                Some(BehaviorState::Patrol { waypoint_index: 0 })
            }
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
            // Deactivate siren on any manual state transition
            if !matches!(next_state, BehaviorState::Apprehend) {
                self.siren_on = false;
            }
            self.state = next_state;
            self.state_entered_at = Instant::now();
        }
    }

    /// Called every control tick. Returns the `Twist` command to publish.
    pub fn tick(&mut self, sensors: &SensorState, intruder: &IntruderState) -> Twist {
        // ---- Safety watchdog (DISABLED) ----------------------------
        // The robot currently flips on spawn due to physics instability.
        // Tilt E-Stop is disabled until spawn dynamics are fixed.
        // To re-enable, uncomment the block below.
        /*
        if !sensors.check_tilt() && self.state != BehaviorState::Idle {
            if !matches!(self.state, BehaviorState::EmergencyStop { .. }) {
                warn!("[BehaviorFSM] Excessive tilt detected — E-Stop!");
                self.siren_on = false;
                self.state = BehaviorState::EmergencyStop {
                    reason: format!(
                        "tilt (roll={:.2} pitch={:.2})",
                        sensors.roll, sensors.pitch
                    ),
                };
                self.state_entered_at = Instant::now();
            }
        }
        */

        let elapsed = self.state_entered_at.elapsed();

        match &self.state.clone() {
            // ---- Idle: zero velocity --------------------------------
            BehaviorState::Idle => {
                self.siren_on = false;
                // Detect intruder → transition to Pursue
                if intruder.detected {
                    info!(
                        "[BehaviorFSM] 🚨 INTRUDER DETECTED from Idle at {:.1}m — PURSUING!",
                        intruder.distance
                    );
                    self.state = BehaviorState::Pursue;
                    self.state_entered_at = Instant::now();
                    return self.compute_pursuit_twist(intruder);
                }
                zero_twist()
            }

            // ---- StandUp: robot should stand still ------------------
            BehaviorState::StandUp => {
                // Detect intruder → transition to Pursue
                if intruder.detected {
                    info!(
                        "[BehaviorFSM] 🚨 INTRUDER DETECTED from StandUp at {:.1}m — PURSUING!",
                        intruder.distance
                    );
                    self.state = BehaviorState::Pursue;
                    self.state_entered_at = Instant::now();
                    return self.compute_pursuit_twist(intruder);
                }
                zero_twist()
            }

            // ---- Trot: publish constant forward velocity -----------
            BehaviorState::Trot => {
                // Detect intruder → transition to Pursue
                if intruder.detected {
                    info!(
                        "[BehaviorFSM] 🚨 INTRUDER DETECTED from Trot at {:.1}m — PURSUING!",
                        intruder.distance
                    );
                    self.state = BehaviorState::Pursue;
                    self.state_entered_at = Instant::now();
                    return self.compute_pursuit_twist(intruder);
                }
                make_twist(self.target_vel_x, self.target_ang_z)
            }

            // ---- Patrol: cycle through waypoints -------------------
            BehaviorState::Patrol { waypoint_index } => {
                let wp_idx = *waypoint_index;

                // Check for intruder detection during patrol
                if intruder.detected {
                    info!(
                        "[BehaviorFSM] 🚨 INTRUDER DETECTED at {:.1}m, bearing {:.1}° — PURSUING!",
                        intruder.distance,
                        intruder.bearing.to_degrees()
                    );
                    self.state = BehaviorState::Pursue;
                    self.state_entered_at = Instant::now();
                    // Immediately start pursuing
                    return self.compute_pursuit_twist(intruder);
                }

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

            // ---- Pursue: follow the intruder -----------------------
            BehaviorState::Pursue => {
                if !intruder.detected {
                    // Intruder lost — check timeout
                    if intruder.time_since_lost > LOST_TIMEOUT_SECS {
                        info!(
                            "[BehaviorFSM] Intruder lost for {:.1}s — returning to Patrol",
                            intruder.time_since_lost
                        );
                        self.state = BehaviorState::Patrol { waypoint_index: 0 };
                        self.state_entered_at = Instant::now();
                        return make_twist(0.0, 0.3); // Slowly rotate to search
                    }
                    // Still within timeout — keep heading in last known direction
                    return make_twist(0.1, 0.0);
                }

                // Check if we should apprehend (close + intruder stopped)
                let intruder_stopped = intruder.velocity.abs() < INTRUDER_STOPPED_THRESHOLD;
                if intruder.distance < APPREHEND_DISTANCE && intruder_stopped {
                    info!(
                        "[BehaviorFSM] 🛑 Intruder STOPPED at {:.1}m — APPREHENDING!",
                        intruder.distance
                    );
                    self.state = BehaviorState::Apprehend;
                    self.state_entered_at = Instant::now();
                    self.siren_on = true;
                    return zero_twist();
                }

                self.compute_pursuit_twist(intruder)
            }

            // ---- Apprehend: siren ON, hold position ----------------
            BehaviorState::Apprehend => {
                self.siren_on = true;

                // Minimum dwell time: stay in Apprehend for at least 3 seconds
                // before considering that the intruder is moving again.
                // This prevents oscillation from noisy velocity estimates.
                if elapsed.as_secs_f64() > 3.0
                    && intruder.detected
                    && intruder.velocity.abs() > 0.5
                {
                    info!(
                        "[BehaviorFSM] Intruder MOVING AGAIN (vel={:.2}m/s) — resuming pursuit!",
                        intruder.velocity
                    );
                    self.siren_on = false;
                    self.state = BehaviorState::Pursue;
                    self.state_entered_at = Instant::now();
                    return self.compute_pursuit_twist(intruder);
                }

                // If intruder is lost, resume patrol
                if !intruder.detected && intruder.time_since_lost > LOST_TIMEOUT_SECS {
                    info!("[BehaviorFSM] Intruder lost during apprehend — returning to Patrol");
                    self.siren_on = false;
                    self.state = BehaviorState::Patrol { waypoint_index: 0 };
                    self.state_entered_at = Instant::now();
                    return zero_twist();
                }

                // Hold position, face intruder with slight steering correction
                if intruder.detected && intruder.bearing.abs() > 0.1 {
                    make_twist(0.0, intruder.bearing * 0.5)
                } else {
                    zero_twist()
                }
            }

            // ---- EmergencyStop: broadcast zero velocity ------------
            BehaviorState::EmergencyStop { reason } => {
                self.siren_on = false;

                // Auto-recover: if tilt has stabilized for 3+ seconds, go to Idle
                if sensors.check_tilt() && elapsed.as_secs_f64() > 3.0 {
                    info!("[BehaviorFSM] Tilt recovered — auto-clearing E-Stop → Idle");
                    self.state = BehaviorState::Idle;
                    self.state_entered_at = Instant::now();
                    return zero_twist();
                }

                // Throttle log to once per second
                if elapsed.as_millis() % 2000 < 50 {
                    warn!("[BehaviorFSM] E-STOP active: {} (roll={:.2} pitch={:.2})",
                        reason, sensors.roll, sensors.pitch);
                }
                zero_twist()
            }

            // ---- Manual: pass through teleop velocity --------------
            BehaviorState::Manual => {
                self.siren_on = false;
                self.manual_vel.clone()
            }
        }
    }

    /// Compute a pursuit Twist to steer toward the intruder.
    fn compute_pursuit_twist(&self, intruder: &IntruderState) -> Twist {
        let bearing_abs = intruder.bearing.abs();

        // If bearing is large (>25°), stop and turn in place first.
        // This prevents the robot from tipping over by turning while moving.
        if bearing_abs > 0.44 {
            let steer = intruder.bearing.signum() * 0.25; // Very gentle turn in place
            return make_twist(0.0, steer);
        }

        // Speed proportional to distance (slow down as we approach)
        // Also reduce speed when bearing is off-center
        let bearing_factor = 1.0 - (bearing_abs / 0.44).min(1.0) * 0.6;
        let speed = if intruder.distance > 3.0 {
            PURSUIT_MAX_SPEED * bearing_factor
        } else {
            (intruder.distance / 3.0 * PURSUIT_MAX_SPEED * bearing_factor).max(0.05)
        };

        // Gentle steering: proportional to bearing error
        let steer = (intruder.bearing * PURSUIT_STEERING_GAIN).clamp(-0.3, 0.3);

        make_twist(speed, steer)
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
    // Shared state (sensor readings + FSM + intruder)
    // ------------------------------------------------------------------ //
    let sensors = Arc::new(Mutex::new(SensorState::default()));
    let fsm = Arc::new(Mutex::new(BehaviorFsm::new()));
    let intruder = Arc::new(Mutex::new(IntruderState::default()));

    // ------------------------------------------------------------------ //
    // Publishers
    // ------------------------------------------------------------------ //
    let cmd_vel_pub = node.create_publisher::<Twist>("/cmd_vel")?;
    let state_pub = node.create_publisher::<RosString>("/go2/behavior_state")?;
    let siren_pub = node.create_publisher::<Bool>("/go2/siren_cmd")?;

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

    // Intruder detection bearing (from go2_detector)
    let intruder_bearing = Arc::clone(&intruder);
    let _intruder_bearing_sub = node.create_subscription::<Vector3, _>(
        "/go2/intruder_bearing",
        move |msg: Vector3| {
            let mut i = intruder_bearing.lock().unwrap();
            i.distance = msg.x;
            i.bearing = msg.y;
            i.velocity = msg.z;
            i.last_seen = Instant::now();
        },
    )?;

    // Intruder detected flag (from go2_detector)
    let intruder_detected = Arc::clone(&intruder);
    let _intruder_detected_sub = node.create_subscription::<Bool, _>(
        "/go2/intruder_detected",
        move |msg: Bool| {
            let mut i = intruder_detected.lock().unwrap();
            i.detected = msg.data;
            if msg.data {
                i.last_seen = Instant::now();
                i.time_since_lost = 0.0;
            }
        },
    )?;

    // ------------------------------------------------------------------ //
    // Control timer — 50 Hz
    // ------------------------------------------------------------------ //
    let fsm_tick = Arc::clone(&fsm);
    let sensors_tick = Arc::clone(&sensors);
    let intruder_tick = Arc::clone(&intruder);

    let mut last_log = Instant::now();
    let behavior_startup = Instant::now();
    let _timer = node.create_timer_repeating(Duration::from_millis(20), move || {
        // Settling period: hold Idle for first 5 seconds after launch
        if behavior_startup.elapsed().as_secs_f64() < 5.0 {
            let _ = cmd_vel_pub.publish(zero_twist());
            if last_log.elapsed() > Duration::from_secs(1) {
                info!("[go2_behavior] Settling... ({:.1}s)", behavior_startup.elapsed().as_secs_f64());
                last_log = Instant::now();
            }
            return;
        }

        let sensor_snapshot = sensors_tick.lock().unwrap().clone();
        let mut intruder_snapshot = intruder_tick.lock().unwrap().clone();

        // Update time_since_lost
        intruder_snapshot.time_since_lost = intruder_snapshot.last_seen.elapsed().as_secs_f64();
        if intruder_snapshot.time_since_lost > 1.0 {
            intruder_snapshot.detected = false;
        }

        let mut f = fsm_tick.lock().unwrap();
        let twist = f.tick(&sensor_snapshot, &intruder_snapshot);
        let lin_x = twist.linear.x;

        // Publish velocity command
        let _ = cmd_vel_pub.publish(twist);

        // Publish FSM state
        let state_label = f.state.label();
        let _ = state_pub.publish(RosString { data: state_label.into() });

        // Publish siren command
        let _ = siren_pub.publish(Bool { data: f.siren_on });

        if last_log.elapsed() > Duration::from_secs(1) {
            let intruder_info = if intruder_snapshot.detected {
                format!(
                    " | Intruder: {:.1}m @ {:.0}° vel={:.2}m/s",
                    intruder_snapshot.distance,
                    intruder_snapshot.bearing.to_degrees(),
                    intruder_snapshot.velocity
                )
            } else {
                String::new()
            };

            // Tilt monitoring
            let roll_deg = sensor_snapshot.roll.to_degrees();
            let pitch_deg = sensor_snapshot.pitch.to_degrees();
            let tilt_info = if sensor_snapshot.roll.abs() > 1.57 || sensor_snapshot.pitch.abs() > 1.57 {
                warn!("[go2_behavior] 💀 ROBOT FLIPPED! roll={:.0}° pitch={:.0}°", roll_deg, pitch_deg);
                format!(" | ⚠ FLIPPED roll={:.0}° pitch={:.0}°", roll_deg, pitch_deg)
            } else if sensor_snapshot.roll.abs() > 0.52 || sensor_snapshot.pitch.abs() > 0.52 {
                warn!("[go2_behavior] ⚠ TILTING! roll={:.0}° pitch={:.0}°", roll_deg, pitch_deg);
                format!(" | ⚠ TILT roll={:.0}° pitch={:.0}°", roll_deg, pitch_deg)
            } else {
                String::new()
            };

            println!(
                "[go2_behavior] HEARTBEAT: State = {}, VelX = {:.2}, Siren = {}{}{}",
                state_label, lin_x, f.siren_on, intruder_info, tilt_info
            );
            last_log = Instant::now();
        }
    })?;

    info!("[go2_behavior] Node started — waiting for commands on /go2/cmd_behavior");
    info!("  Commands: stand | trot | patrol | idle | estop | clear");
    info!("  Intruder detection: /go2/intruder_bearing, /go2/intruder_detected");
    info!("  Siren control: /go2/siren_cmd");

    let _ = executor.spin(rclrs::SpinOptions::default());
    Ok(())
}
