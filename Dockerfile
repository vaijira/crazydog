# ============================================================
# CrazyDog — Docker image for Go2 ROS 2 development
#
# Base: official ROS 2 Jazzy desktop (Ubuntu 24.04 LTS)
# Adds: gz-harmonic, ros2_control, Rust, colcon-cargo
#
# Build:  docker compose build
# Run:    xhost +local:docker && docker compose run --rm crazydog
# ============================================================

FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]

# ---- System packages + Gazebo Harmonic ------------------------
# Add OSRF Gazebo repo (Harmonic = gz-sim8 / Jazzy era)
RUN apt-get update -qq \
    && apt-get install -y --no-install-recommends curl ca-certificates \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
        http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update -qq \
    && apt-get install -y --no-install-recommends \
        wget \
        gz-harmonic \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-xacro \
        ros-jazzy-test-msgs \
        # Rust / bindgen requirements
        libclang-dev \
        python3-pip \
        python3-venv \
        python3-colcon-common-extensions \
        python3-colcon-ros \
    && rm -rf /var/lib/apt/lists/*

# ---- Rust stable toolchain ------------------------------------
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
    | sh -s -- -y --default-toolchain stable --profile minimal
ENV PATH="/root/.cargo/bin:${PATH}"

# ---- colcon-cargo extensions (via pip3) -----------------------
# --break-system-packages is safe inside Docker (no real system to break)
# and required on Ubuntu 24.04 due to PEP 668 externally-managed enforcement.
RUN python3 -m pip install --no-cache-dir --break-system-packages \
        colcon-cargo \
        colcon-ros-cargo \
        lark \
        "empy==3.3.4"

# ---- Workspace ------------------------------------------------
WORKDIR /ws

# Source ROS 2 in every interactive shell
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /root/.bashrc

# ---- Entrypoint -----------------------------------------------
COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
