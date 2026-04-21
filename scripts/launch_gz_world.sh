#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD_SRC="${SCRIPT_DIR}/../worlds/nbv_scene.sdf"
WORLD_DST="${HOME}/PhD/nerf/PX4-Autopilot/Tools/simulation/gz/worlds/nbv_scene.sdf"

# 1. Symlink our custom world into PX4's worlds directory so PX4 can find it
if [ ! -e "${WORLD_DST}" ]; then
    ln -s "$(realpath "${WORLD_SRC}")" "${WORLD_DST}"
    echo "[launch_gz_world] Symlinked nbv_scene.sdf into PX4 worlds directory."
fi

# 2. Tell PX4 which world to load
export PX4_GZ_WORLD=nbv_scene

# 3. Execute the PX4 simulation
make -C "${HOME}/PhD/nerf/PX4-Autopilot" px4_sitl gz_x500_mono_cam