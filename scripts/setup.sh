#!/usr/bin/env bash
# setup.sh — CrazyDog developer environment bootstrap.
# Detects your Ubuntu version and takes the appropriate install path.
#
# Usage:
#   bash scripts/setup.sh             # tracing disabled (faster build)
#   bash scripts/setup.sh --tracing   # enable LTTng ros2 tracing
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ------------------------------------------------------------------ #
# Parse flags
# ------------------------------------------------------------------ #
ENABLE_TRACING=false
for arg in "$@"; do
    [[ "$arg" == "--tracing" ]] && ENABLE_TRACING=true
done
echo "==> Tracing: ${ENABLE_TRACING}"

# ------------------------------------------------------------------ #
# Detect Ubuntu codename
# ------------------------------------------------------------------ #
UBUNTU_CODENAME=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME:-unknown}}")
echo "==> Detected Ubuntu codename: ${UBUNTU_CODENAME}"

# ------------------------------------------------------------------ #
# Rust (always needed, works on any distro)
# ------------------------------------------------------------------ #
echo "==> Ensuring Rust stable toolchain..."
if ! command -v rustup &>/dev/null; then
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain stable
    # shellcheck disable=SC1090
    source "$HOME/.cargo/env"
else
    rustup update stable
    rustup default stable
fi

# ------------------------------------------------------------------ #
# Docker Compose plugin (needed for Option C — works on any Ubuntu)
# ------------------------------------------------------------------ #
if ! docker compose version &>/dev/null 2>&1; then
    echo "==> Installing docker-compose-v2..."
    sudo apt install -y docker-compose-v2
fi

# ------------------------------------------------------------------ #
# ROS 2 + Gazebo — path depends on Ubuntu version
# ------------------------------------------------------------------ #
if [[ "${UBUNTU_CODENAME}" == "noble" ]]; then
    # ── Ubuntu 24.04 LTS: install from apt ──────────────────────── #
    echo "==> Ubuntu 24.04 (Noble) — installing ROS 2 Jazzy from apt..."

    # Add ROS 2 apt repo if not present
    if ! apt-cache show ros-jazzy-rclcpp &>/dev/null 2>&1; then
        sudo apt install -y software-properties-common curl
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu noble main" \
            | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        sudo apt update -qq
    fi

    sudo apt install -y \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-xacro \
        libclang-dev \
        python3-venv python3-pip

    # ---- Python venv for colcon extensions (Noble) ----------------- #
    VENV_DIR="$HOME/.ros2_venv"
    echo "==> Creating Python venv at ${VENV_DIR}..."
    python3 -m venv --system-site-packages "${VENV_DIR}"
    # shellcheck disable=SC1090
    source "${VENV_DIR}/bin/activate"
    pip install --quiet --upgrade pip
    pip install --quiet --upgrade colcon-cargo colcon-ros-cargo

    echo ""
    echo "✅ ROS 2 Jazzy installed via apt."
    echo "   Source with:"
    echo "     source /opt/ros/jazzy/setup.bash"
    echo "     source ${VENV_DIR}/bin/activate"

elif [[ "${UBUNTU_CODENAME}" == "questing" ]] || [[ "${UBUNTU_CODENAME}" == "oracular" ]]; then
    # ── Ubuntu 25.10 / 24.10: build from source ─────────────────── #
    echo "==> Ubuntu ${UBUNTU_CODENAME} detected."
    echo "    ROS 2 Jazzy apt packages are NOT available for this release."
    echo "    Building ROS 2 Jazzy from source (~30-45 min first run)..."
    echo ""

    ROS_WS="$HOME/ros2_jazzy"

    # ---- Gazebo Harmonic ABI check ---------------------------------- #
    # The OSRF Gazebo Harmonic apt packages require libtinyxml2-10 at the
    # Ubuntu 24.04 (Noble) ABI. Ubuntu 25.10 ships an incompatible newer
    # version — the packages cannot be installed natively.
    echo ""
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║  ⚠️  GAZEBO HARMONIC CANNOT BE INSTALLED ON UBUNTU 25.10    ║"
    echo "║                                                              ║"
    echo "║  The OSRF apt packages fail due to a libtinyxml2 ABI        ║"
    echo "║  mismatch between Noble (24.04) and Questing (25.10).       ║"
    echo "║                                                              ║"
    echo "║  ✅  USE DOCKER INSTEAD (Option C):                         ║"
    echo "║      xhost +local:docker                                     ║"
    echo "║      docker compose run --rm crazydog                        ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo ""
    read -rp "Continue anyway with ROS 2 core only (no Gazebo)? [y/N] " ans
    if [[ "${ans,,}" != "y" ]]; then
        echo "Exiting. Run: docker compose run --rm crazydog"
        exit 0
    fi
    echo "==> Proceeding with ROS 2 core source build (Gazebo disabled)..."
    echo ""

    # Ubuntu 25.10 / GCC 15: install all known-missing system libs up front
    # to avoid one-by-one build failures from vendor packages.
    # NOTE: python3-colcon-* and python3-lark are intentionally NOT listed here;
    #       they are installed into the venv below to keep pip packages isolated.
    sudo apt install -y \
        python3-venv python3-pip \
        python3-vcstool \
        python3-rosdep \
        cmake ninja-build build-essential \
        libclang-dev \
        libacl1-dev \
        libasio-dev libssl-dev \
        libtinyxml2-dev \
        libyaml-cpp-dev \
        libopencv-dev \
        libbullet-dev libboost-all-dev libcurl4-openssl-dev \
        python3-pytest \
        libglu1-mesa-dev libgl1-mesa-dev \
        libassimp-dev

    # Optionally install LTTng for ros2 tracing support
    if [[ "${ENABLE_TRACING}" == "true" ]]; then
        echo "==> Installing LTTng tracing libraries..."
        sudo apt install -y liblttng-ust-dev liblttng-ctl-dev lttng-tools
    fi

    # ---- Python venv ----------------------------------------------- #
    # --system-site-packages lets the venv see apt-installed tools
    # (rosdep, vcstool, pytest) while pip packages live cleanly inside.
    VENV_DIR="${ROS_WS}/venv"
    echo "==> Creating Python venv at ${VENV_DIR}..."
    mkdir -p "${ROS_WS}"
    python3 -m venv --system-site-packages "${VENV_DIR}"
    # shellcheck disable=SC1090
    source "${VENV_DIR}/bin/activate"

    pip install --quiet --upgrade pip
    pip install --quiet --upgrade \
        colcon-common-extensions \
        colcon-cargo \
        colcon-ros-cargo \
        lark \
        empy==3.3.4 \
        catkin-pkg \
        vcstool

    echo "==> Python venv ready: ${VENV_DIR}"

    # ---- Import source repos --------------------------------------- #
    mkdir -p "${ROS_WS}/src"
    cd "${ROS_WS}"

    echo "==> Importing ROS 2 Jazzy repos..."
    vcs import src < <(curl -fsSL https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos)

    echo "==> Importing extra deps (ros2_control, gz, xacro, ...)"
    # These packages are NOT in the base ros2.repos — import them separately.
    vcs import src < "${SCRIPT_DIR}/../deps.repos"

    echo "==> Initialising rosdep..."
    sudo rosdep init 2>/dev/null || true
    rosdep update

    echo "==> Installing rosdep dependencies..."
    # --os ubuntu:noble: use Noble package mappings on Questing (25.10 has no
    # rosdep mappings yet). --skip-keys drops packages with no apt mapping.
    rosdep install --from-paths src --ignore-src --rosdistro jazzy \
        --os ubuntu:noble \
        --skip-keys "
            fastcdr
            rti-connext-dds-6.0.1
            urdfdom_headers
            python3-catkin-pkg-modules
            libopencv-imgproc
            libopencv-dev
            libopencv-imgcodecs
            intra_process_demo
            image_tools
        " \
        -y || true

    # ---- Configure tracing ----------------------------------------- #
    SKIP_TRACING_PKGS=()
    TRACING_CMAKE_ARGS=()
    if [[ "${ENABLE_TRACING}" == "false" ]]; then
        SKIP_TRACING_PKGS=(lttngpy)
        TRACING_CMAKE_ARGS=(-DTRACETOOLS_DISABLED=ON -DTRACETOOLS_LTTNG_UST_ENABLED=OFF)
        echo "==> Building ROS 2 (tracing disabled — pass --tracing to enable)"
    else
        echo "==> Building ROS 2 (LTTng tracing ENABLED)"
        rm -rf \
            "${ROS_WS}/build/tracetools" \
            "${ROS_WS}/build/lttngpy" \
            "${ROS_WS}/install/tracetools" \
            "${ROS_WS}/install/lttngpy" 2>/dev/null || true
    fi

    # ---- Build ----------------------------------------------------- #
    colcon build \
        --symlink-install \
        --packages-skip \
            intra_process_demo image_tools \
            rviz_ogre_vendor rviz_assimp_vendor rviz_rendering rviz_common \
            rviz_default_plugins rviz2 rviz_visual_testing_framework rviz_rendering_tests \
            "${SKIP_TRACING_PKGS[@]}" \
            qt_gui_cpp python_qt_binding qt_gui_py_common \
        --packages-skip-by-dep qt_gui_cpp python_qt_binding \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=RelWithDebInfo \
            "${TRACING_CMAKE_ARGS[@]}"

    echo ""
    echo "✅ ROS 2 Jazzy built from source at: ${ROS_WS}"
    echo "   Venv location: ${VENV_DIR}"

    # ---- Offer to add both to .bashrc ------------------------------ #
    BASHRC_SNIPPET="
# CrazyDog / ROS 2 Jazzy (source build)
source ${ROS_WS}/install/setup.bash
source ${VENV_DIR}/bin/activate
"
    if ! grep -q "ros2_jazzy" "$HOME/.bashrc"; then
        read -rp "Add ROS 2 + venv activation to ~/.bashrc? [y/N] " ans
        if [[ "${ans,,}" == "y" ]]; then
            echo "${BASHRC_SNIPPET}" >> "$HOME/.bashrc"
            echo "Added to ~/.bashrc"
        fi
    fi

else
    echo "⚠️  Unknown Ubuntu codename: ${UBUNTU_CODENAME}"
    echo "   See README.md for manual ROS 2 installation instructions."
    echo "   Or use the Docker option (Option C in README.md) to get started immediately."
    exit 1
fi

echo ""
echo "==> libclang-dev check (required by rclrs bindgen)..."
dpkg -l libclang-dev &>/dev/null || sudo apt install -y libclang-dev

echo ""
echo "════════════════════════════════════════════"
echo "  ✅  Setup complete!"
echo ""
echo "  Next steps:"
echo "    source \${ROS_WS}/install/setup.bash   # or /opt/ros/jazzy/setup.bash on Noble"
echo "    source \${VENV_DIR}/bin/activate        # activate the Python venv"
echo "    cd ${SCRIPT_DIR}/.."
echo "    colcon build --symlink-install"
echo "    source install/setup.bash"
echo "    ros2 launch go2_bringup sim.launch.py"
echo "════════════════════════════════════════════"
