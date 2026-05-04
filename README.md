# 🐕 CrazyDog — Unitree Go2 Custom Behavior Stack

A **Rust + ROS 2** skeleton project for customizing the behavior of the
**Unitree Go2** quadruped robot, with full **Gazebo Harmonic** simulation
support.

```
crazydog/
├── src/                          # ROS 2 packages (colcon workspace)
│   ├── go2_description/          # URDF/XACRO robot model + meshes
│   ├── go2_bringup/              # Launch files (sim + real)
│   ├── go2_behavior/             # Rust — high-level behavior FSM node
│   ├── go2_gait/                 # Rust — gait controller / foot planner
│   └── go2_teleop/               # Rust — keyboard / joystick teleop node
├── config/                       # Shared YAML parameter files
├── scripts/                      # Helper shell scripts
├── Cargo.toml                    # Workspace Cargo.toml (Rust packages)
└── README.md
```

---

## Prerequisites

| Tool | Version |
|------|---------|
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic (gz-sim8) |
| Rust | stable ≥ 1.80 (`rustup update stable`) |
| colcon-cargo | `pip install colcon-cargo colcon-ros-cargo` |

> **⚠️ Ubuntu version matters.**
> ROS 2 Jazzy apt packages (`ros-jazzy-*`) are only published for
> **Ubuntu 24.04 LTS (Noble)**. On Ubuntu 25.10 or other releases you
> must use one of the alternative install paths below.

### Option A — Ubuntu 24.04 LTS (apt, easiest)
```bash
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  libclang-dev
pip install colcon-cargo colcon-ros-cargo
```

### Option B — Ubuntu 25.10 / non-LTS: build ROS 2 from source

> **Ubuntu 25.10 — hard blocker with Gazebo Harmonic**
>
> ROS 2 Jazzy core packages can be built from source on Ubuntu 25.10, but
> **Gazebo Harmonic cannot be installed**: the OSRF apt packages are built
> for Ubuntu 24.04 (Noble) and depend on `libtinyxml2-10` at a specific Noble
> ABI. Ubuntu 25.10 ships a newer, binary-incompatible version. Building
> Gazebo from source would take several hours and hit similar GCC 15 issues.
>
> ✅ **Use Option C (Docker) instead** — it runs Ubuntu 24.04 inside the
> container, giving you full Gazebo + ROS 2 Jazzy support with zero conflicts.
> The Docker image mounts your source so edits are reflected instantly.

<details><summary>Option B — source build (ROS 2 core only, no Gazebo)</summary>

This is useful if you only need `rclcpp`/`rclpy` and don't need simulation.

```bash
# Ubuntu 25.10 ships GCC 15 which is stricter about implicit <cstdint>/<cstdlib>
# includes, and several vendor packages need system libs that rosdep cannot
# resolve for 'questing'. Install them all up front to avoid one-by-one failures.
sudo apt install -y \
  `# build toolchain` \
  python3-pip python3-vcstool python3-colcon-common-extensions \
  python3-rosdep cmake ninja-build build-essential \
  `# rclrs / bindgen` \
  libclang-dev \
  `# iceoryx  (sys/acl.h)` \
  libacl1-dev \
  `# fastrtps / Fast-DDS` \
  libasio-dev libssl-dev \
  `# tinyxml2_vendor  (uses system lib instead of broken download)` \
  libtinyxml2-dev \
  `# yaml_cpp_vendor  (GCC 15 cstdint fix)` \
  libyaml-cpp-dev \
  `# image_tools / OpenCV (skipped, but headers needed by other pkgs)` \
  libopencv-dev \
  `# misc runtime deps` \
  libbullet-dev libboost-all-dev libcurl4-openssl-dev \
  python3-lark \
  libglu1-mesa-dev libgl1-mesa-dev \
  libassimp-dev
  # LTTng tracing is fully disabled via -DTRACETOOLS_DISABLED=ON below;
  # install liblttng-ust-dev + liblttng-ctl-dev only if you need ros2 tracing.
  # liblttng-ust-dev liblttng-ctl-dev lttng-tools

# Set this to wherever you cloned crazydog
CRAZYDOG_DIR=~/Projects/crazydog

mkdir -p ~/ros2_jazzy/src && cd ~/ros2_jazzy

# 1. Import core ROS 2 Jazzy
vcs import src < <(curl -fsSL https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos)

# 2. Import extra packages (ros2_control, gz_ros2_control, xacro, etc.)
#    These are NOT in the base ros2.repos — they live in separate repos.
vcs import src < "${CRAZYDOG_DIR}/deps.repos"

sudo rosdep init 2>/dev/null || true
rosdep update

# 3. Install dependencies
#    --os ubuntu:noble  — use Noble's package mappings (25.10 has none)
#    --skip-keys        — packages with no apt equivalent / not needed
rosdep install --from-paths src --ignore-src --rosdistro jazzy \
  --os ubuntu:noble \
  --skip-keys "
    fastcdr
    rti-connext-dds-6.0.1
    urdfdom_headers
    libopencv-imgproc
    libopencv-dev
    libopencv-imgcodecs
    intra_process_demo
    image_tools
  " \
  -y

# 4. Build — skip OpenCV demos and the entire rviz2/OGRE/Assimp stack.
#    We use Gazebo for visualisation; these packages take 30+ min to build
#    from source and fail on GCC 15 with -Werror=array-bounds / missing GL headers.
colcon build --symlink-install \
  --packages-skip \
    intra_process_demo image_tools \
    rviz_ogre_vendor rviz_assimp_vendor rviz_rendering rviz_common \
    rviz_default_plugins rviz2 rviz_visual_testing_framework rviz_rendering_tests \
    lttngpy \
    qt_gui_cpp python_qt_binding qt_gui_py_common \
  --packages-skip-by-dep qt_gui_cpp python_qt_binding \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DTRACETOOLS_DISABLED=ON \
    -DTRACETOOLS_LTTNG_UST_ENABLED=OFF

source ~/ros2_jazzy/install/setup.bash
pip install colcon-cargo colcon-ros-cargo
```

### Option C — Docker (works on any OS today)
```bash
docker run -it --rm --network host \
  -v /home/koke/Projects/crazydog:/ws/src/crazydog \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  osrf/ros:jazzy-desktop bash -c "
    apt-get update -qq && apt-get install -y \
      ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
      ros-jazzy-gz-ros2-control ros-jazzy-ros-gz-bridge \
      ros-jazzy-ros-gz-sim ros-jazzy-robot-state-publisher \
      ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro \
      libclang-dev && \
    pip install -q colcon-cargo colcon-ros-cargo && \
    source /opt/ros/jazzy/setup.bash && \
    cd /ws && colcon build --symlink-install && \
    source install/setup.bash && \
    ros2 launch go2_bringup sim.launch.py"
```

---

## Quick Start

```bash
# 1. Source ROS 2
source /opt/ros/jazzy/setup.bash

# 2. Build the whole workspace (C++ description + Rust nodes)
cd ~/Projects/crazydog
colcon build --symlink-install

# 3. Source the overlay
source install/setup.bash

# 4. Launch Gazebo simulation
ros2 launch go2_bringup sim.launch.py

# 5. In a second terminal — run teleop
ros2 run go2_teleop go2_teleop

# 6. In a third terminal — run the behavior FSM
ros2 run go2_behavior go2_behavior
```

---

## Architecture

```
┌──────────────────────────────────────────────────────┐
│                   go2_behavior  (Rust)               │
│   BehaviorFSM:  Idle → Stand → Walk → Patrol → ...  │
│   Subscribes:   /joint_states, /odom, /imu           │
│   Publishes:    /cmd_vel (→ go2_gait)                │
└────────────────────┬─────────────────────────────────┘
                     │  /cmd_vel
┌────────────────────▼─────────────────────────────────┐
│                   go2_gait  (Rust)                   │
│   Trot gait generator + inverse-kinematics stub      │
│   Subscribes:   /cmd_vel                             │
│   Publishes:    /joint_commands (Float64MultiArray)  │
└────────────────────┬─────────────────────────────────┘
                     │  /joint_commands
┌────────────────────▼─────────────────────────────────┐
│          Gazebo + ros2_control + go2_description     │
│          (joint_trajectory_controller / effort ctrl) │
└──────────────────────────────────────────────────────┘
```

---

## Rust Packages

Each Rust package lives in `src/<name>/` and is also a standard ROS 2
`ament_cargo` package (has `package.xml` + `Cargo.toml`).

| Package | Binary | Purpose |
|---------|--------|---------|
| `go2_behavior` | `go2_behavior` | Finite-state machine — decides _what_ the robot does |
| `go2_gait` | `go2_gait` | Converts velocity commands into joint references |
| `go2_teleop` | `go2_teleop` | Keyboard teleop (publishes `/cmd_vel`) |

---

## Topics / Services

| Topic | Type | Direction |
|-------|------|-----------|
| `/cmd_vel` | `geometry_msgs/Twist` | behavior → gait |
| `/joint_states` | `sensor_msgs/JointState` | sim → behavior, gait |
| `/odom` | `nav_msgs/Odometry` | sim → behavior |
| `/imu/data` | `sensor_msgs/Imu` | sim → behavior |
| `/joint_commands` | `std_msgs/Float64MultiArray` | gait → sim |
| `/go2/behavior_state` | `std_msgs/String` | behavior → external |

---

## Extending the Behavior FSM

Open `src/go2_behavior/src/main.rs` and add a new variant to
`BehaviorState`.  Connect it with `transition()` and implement the
corresponding `tick()` logic.  The FSM deliberately avoids `async` so it
remains deterministic and easy to reason about.
