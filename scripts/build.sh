#!/usr/bin/env bash
# build.sh — Convenience wrapper around colcon build.
#
# Usage:
#   bash scripts/build.sh                        # full incremental build
#   bash scripts/build.sh --clean                # wipe build/ install/ log/ first
#   bash scripts/build.sh go2_behavior           # build a single package
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

CLEAN=false
PKG=""

for arg in "$@"; do
    case "$arg" in
        --clean) CLEAN=true ;;
        --*)     echo "Unknown flag: $arg"; exit 1 ;;
        *)       PKG="$arg" ;;
    esac
done

if [[ "${CLEAN}" == "true" ]]; then
    echo "==> Cleaning build artifacts..."
    rm -rf "$WS_ROOT/build" "$WS_ROOT/install" "$WS_ROOT/log"
fi

# Source ROS 2 if not already sourced
if [ -z "${ROS_DISTRO:-}" ]; then
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        # Docker / Noble: apt-installed ROS 2
        # shellcheck disable=SC1091
        source /opt/ros/jazzy/setup.bash
    elif [ -f "$HOME/ros2_jazzy/install/setup.bash" ]; then
        # Questing: source-built ROS 2
        # shellcheck disable=SC1091
        source "$HOME/ros2_jazzy/install/setup.bash"
    else
        echo "ERROR: ROS 2 not found. Source setup.bash first."
        exit 1
    fi
fi

cd "$WS_ROOT"

if [[ -n "$PKG" ]]; then
    echo "==> Building package: ${PKG}"
    colcon build \
        --base-paths src \
        --symlink-install \
        --packages-select "$PKG" \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        --cargo-args --release
else
    echo "==> Building CrazyDog workspace..."
    colcon build \
        --base-paths src \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        --cargo-args --release
fi

echo ""
echo "✅  Build complete. Source the overlay with:"
echo "    source $WS_ROOT/install/setup.bash"
