//! # go2_teleop
//!
//! Keyboard teleoperation node for the Unitree Go2.
//!
//! Reads raw keypresses from stdin (using `termios` for raw-mode input)
//! and publishes on two topics:
//!
//! | Topic                 | Type                       | When              |
//! |-----------------------|----------------------------|-------------------|
//! | `/cmd_vel`            | `geometry_msgs/Twist`      | every key event   |
//! | `/go2/cmd_behavior`   | `std_msgs/String`          | on mode keys      |
//!
//! ## Key bindings
//! ```text
//!  w / s     forward / backward   (+/- linear.x)
//!  a / d     turn left / right    (+/- angular.z)
//!  SPACE     full stop
//!  1         behavior: stand
//!  2         behavior: trot
//!  3         behavior: patrol
//!  0         behavior: idle
//!  e         emergency stop
//!  c         clear e-stop
//!  q / ESC   quit
//! ```

use std::io::{self, Read};
use std::os::unix::io::AsRawFd;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use anyhow::Result;
use log::info;

use geometry_msgs::msg::Twist;
use rclrs::{Context, CreateBasicExecutor};
use std_msgs::msg::String as RosString;
use termios::{tcsetattr, Termios, ECHO, ICANON, TCSANOW};

// ============================================================
// Velocity step sizes
// ============================================================
const VEL_STEP: f64 = 0.05; // m/s per keypress
const ANG_STEP: f64 = 0.1;  // rad/s per keypress
const VEL_MAX: f64 = 0.6;
const ANG_MAX: f64 = 1.5;

// ============================================================
// Terminal raw-mode guard
// ============================================================

/// RAII guard that restores the terminal on drop.
struct RawTerminal {
    fd: i32,
    original: Termios,
}

impl RawTerminal {
    fn enable() -> io::Result<Self> {
        let fd = io::stdin().as_raw_fd();
        let original = Termios::from_fd(fd)?;
        let mut raw = original.clone();
        raw.c_lflag &= !(ICANON | ECHO);
        tcsetattr(fd, TCSANOW, &raw)?;
        Ok(Self { fd, original })
    }
}

impl Drop for RawTerminal {
    fn drop(&mut self) {
        let _ = tcsetattr(self.fd, TCSANOW, &self.original);
    }
}

// ============================================================
// Teleop state
// ============================================================

#[derive(Debug, Default)]
struct TeleopState {
    lin_x: f64,
    ang_z: f64,
}

impl TeleopState {
    fn apply_key(&mut self, key: u8) -> Option<&'static str> {
        match key {
            b'w' => { self.lin_x = (self.lin_x + VEL_STEP).min(VEL_MAX); None }
            b's' => { self.lin_x = (self.lin_x - VEL_STEP).max(-VEL_MAX); None }
            b'a' => { self.ang_z = (self.ang_z + ANG_STEP).min(ANG_MAX); None }
            b'd' => { self.ang_z = (self.ang_z - ANG_STEP).max(-ANG_MAX); None }
            b' ' => { self.lin_x = 0.0; self.ang_z = 0.0; None }
            b'1' => Some("stand"),
            b'2' => Some("trot"),
            b'3' => Some("patrol"),
            b'0' => Some("idle"),
            b'm' => Some("manual"),
            b'e' => Some("estop"),
            b'c' => Some("clear"),
            _    => None,
        }
    }

    fn as_twist(&self) -> Twist {
        let mut t = Twist::default();
        t.linear.x  = self.lin_x;
        t.angular.z = self.ang_z;
        t
    }
}

// ============================================================
// Main
// ============================================================

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let ctx = Context::default_from_env()?;
    let mut executor = ctx.create_basic_executor();
    let node = executor.create_node("go2_teleop")?;

    let cmd_vel_pub = node.create_publisher::<Twist>("/cmd_vel")?;
    let behavior_pub = node.create_publisher::<RosString>("/go2/cmd_behavior")?;

    // ---- Print help banner ----------------------------------------
    println!(
        "\n╔══════════════════════════════════════╗\
       \n║     CrazyDog — Go2 Teleoperation     ║\
       \n╠══════════════════════════════════════╣\
       \n║  w/s   forward / backward            ║\
       \n║  a/d   turn left / right             ║\
       \n║  SPACE full stop                     ║\
       \n║  1     behavior: stand               ║\
       \n║  2     behavior: trot                ║\
       \n║  3     behavior: patrol              ║\
       \n║  0     behavior: idle                ║\
       \n║  e     emergency stop                ║\
       \n║  c     clear e-stop                  ║\
       \n║  q/ESC quit                          ║\
       \n╚══════════════════════════════════════╝\n"
    );

    let running = Arc::new(AtomicBool::new(true));
    let running_kb = Arc::clone(&running);

    // ---- Keyboard thread (blocking reads on stdin) ----------------
    let kb_thread = thread::spawn(move || -> Result<()> {
        let _raw = RawTerminal::enable()?;
        let mut stdin = io::stdin();
        let mut buf = [0u8; 1];
        let mut state = TeleopState::default();

        while running_kb.load(Ordering::Relaxed) {
            if stdin.read(&mut buf)? == 0 {
                break;
            }
            let key = buf[0];

            // Quit keys
            if key == b'q' || key == 27 /* ESC */ {
                running_kb.store(false, Ordering::Relaxed);
                break;
            }

            if let Some(cmd) = state.apply_key(key) {
                // Behavior command
                info!("[teleop] behavior cmd: {}", cmd);
                let _ = behavior_pub.publish(RosString { data: cmd.into() });
            }

            // Always publish cmd_vel
            let twist = state.as_twist();
            info!(
                "[teleop] vel_x={:.2}  ang_z={:.2}",
                twist.linear.x, twist.angular.z
            );
            let _ = cmd_vel_pub.publish(twist);
        }

        println!("\n[go2_teleop] Exiting — sending zero velocity.");
        let _ = cmd_vel_pub.publish(Twist::default());
        Ok(())
    });

    // ---- Spin ROS executor (in main thread) -----------------------
    while running.load(Ordering::Relaxed) {
        let _ = executor.spin(rclrs::SpinOptions::default().timeout(Duration::from_millis(50)));
    }

    kb_thread.join().ok();
    Ok(())
}
