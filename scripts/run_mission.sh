#!/bin/bash
# run_mission.sh — Build and launch the full NBV mission ROS 2 stack.
#
# What this does
# --------------
# 1. Activates the `nerfly` conda environment (provides Python packages).
# 2. Sources ROS 2 Jazzy.
# 3. Builds the nbv_demo package with colcon.
# 4. Sources the install overlay (setup.zsh — correct for zsh/bash).
# 5. Launches the full mission:
#      - mavros_node     (MAVLink ↔ ROS 2 bridge)
#      - parameter_bridge (Gazebo camera → ROS 2)
#      - mission_node    (autonomous circular orbit state machine)
#      - image_saver_node (saves PNG frames on /nbv/capture trigger)
#
# Prerequisites
# -------------
#   - Terminal A must already be running launch_gz_world.sh (PX4 SITL).
#   - The `nerfly` conda environment must exist.
#   - ROS 2 Jazzy must be installed at /opt/ros/jazzy.
#
# Usage
# -----
#   bash scripts/run_mission.sh
#   # Images are saved to ~/nbv_images/ as frame_000.png, frame_001.png, …

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# # ── Step 1: Activate conda environment ────────────────────────────────────────
# # conda's shell function is only available after sourcing conda's init script.
# CONDA_BASE="$(conda info --base 2>/dev/null || echo "${HOME}/miniforge3")"
# # shellcheck source=/dev/null
# source "${CONDA_BASE}/etc/profile.d/conda.sh"
# conda activate nerfly
# echo "[run_mission] conda env: $(conda info --envs | grep '*' | awk '{print $1}')"

# ── Step 2: Source ROS 2 Jazzy ────────────────────────────────────────────────
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
echo "[run_mission] ROS_DISTRO: ${ROS_DISTRO}"

# ── Step 3: Build the package ─────────────────────────────────────────────────
echo "[run_mission] Building nbv_demo …"
cd "${REPO_ROOT}"
colcon build --packages-select nbv_demo

# ── Step 4: Source the install overlay ────────────────────────────────────────
# Use setup.sh (POSIX sh, works in both bash and zsh) rather than setup.bash
# which relies on ${BASH_SOURCE[0]} and breaks in zsh.
# shellcheck source=/dev/null
source "${REPO_ROOT}/install/setup.sh"
echo "[run_mission] Workspace overlay sourced."

# ── Step 5: Launch ────────────────────────────────────────────────────────────
echo "[run_mission] Launching mission …"
echo "[run_mission] Images will be saved to: ~/nbv_images/"
exec ros2 launch nbv_demo mission.launch.py
