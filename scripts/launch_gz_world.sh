#!/bin/bash
# launch_gz_world.sh — Start PX4 SITL with the NeRFly custom world.
#
# What this does
# --------------
# 1. Symlinks worlds/nbv_scene.sdf from this repo into PX4's worlds directory
#    so PX4 can find it by name.  The symlink is always (re)created so that
#    any edits to the repo's SDF are immediately picked up without a manual
#    copy step.
# 2. Sets PX4_GZ_WORLD=nbv_scene and runs the PX4 SITL + Gazebo simulation.
#
# Prerequisites
# -------------
#   - PX4-Autopilot built at least once: make px4_sitl gz_x500_mono_cam
#   - No conda environment active (PX4 build system manages its own env).
#
# Usage
# -----
#   bash scripts/launch_gz_world.sh
#   # or from the repo root:
#   ./scripts/launch_gz_world.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORLD_SRC="${REPO_ROOT}/worlds/nbv_scene.sdf"
PX4_ROOT="${HOME}/PhD/nerf/PX4-Autopilot"
WORLD_DST="${PX4_ROOT}/Tools/simulation/gz/worlds/nbv_scene.sdf"

# Always force-create the symlink so edits to worlds/nbv_scene.sdf are
# picked up immediately.  -f removes any existing file or broken symlink first.
ln -sf "$(realpath "${WORLD_SRC}")" "${WORLD_DST}"
echo "[launch_gz_world] Symlinked: ${WORLD_SRC} → ${WORLD_DST}"

export PX4_GZ_WORLD=nbv_scene
echo "[launch_gz_world] Launching PX4 SITL with world '${PX4_GZ_WORLD}' …"

exec make -C "${PX4_ROOT}" px4_sitl gz_x500_mono_cam
