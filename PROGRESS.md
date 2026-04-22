# NeRFly — Project Progress

## Phase 1: Environment Setup

**Goal:** Establish a reproducible Python environment for the full stack.

- Created conda environment `nerfly` with **Python 3.10** (matches ROS 2 Jazzy's Python ABI).
- Installed all required Python packages via pip into the environment:

  | Package | Version | Role |
  |---|---|---|
  | `numpy` | 2.2.6 | Array math, image data |
  | `opencv-python` | 4.13.0 | Image read / write / processing |
  | `scipy` | 1.15.3 | Spatial math |
  | `matplotlib` | 3.10.8 | Visualization and debugging |
  | `transforms3d` | 0.4.2 | 3D rotation representations |
  | `pyquaternion` | 0.9.9 | Quaternion math for flight control |
  | `pycolmap` | 4.0.3 | Python bindings for COLMAP |
  | `empy` | 3.3.4 | Required by ROS 2 build tools (pinned version) |
  | `catkin-pkg` | 1.1.0 | ROS 2 package tooling |
  | `lark` | 1.3.1 | Parser required by ROS 2 |
  | `colcon-common-extensions` | 0.3.0 | ROS 2 colcon build system |

- Verified all packages import correctly.

> `rclpy` and MAVROS2 Python bindings are ROS 2 system packages installed
> via apt and are not pip-installable. They are available after sourcing
> `/opt/ros/jazzy/setup.bash`.

---

## Exploratory Development

### ROS 2 Package: `nbv_demo`

Built a complete ROS 2 Python package at `src/nbv_demo/` targeting the
following stack: **Ubuntu 24.04 · ROS 2 Jazzy · PX4 SITL · Gazebo Harmonic ·
MAVROS2 · ros_gz_bridge**.

**Package structure:**

```
src/nbv_demo/
├── package.xml                  # ament_python package, ROS 2 Jazzy
├── setup.py / setup.cfg
├── resource/nbv_demo            # ament resource index marker
├── config/
│   └── mission_params.yaml      # all tunable parameters
├── launch/
│   └── mission.launch.py        # launches MAVROS + gz bridge + both nodes
└── nbv_demo/
    ├── mission_node.py          # autonomous circular orbit state machine
    └── image_saver_node.py      # saves PNG frames on /nbv/capture trigger
```

**`mission_node.py`** — 9-step non-blocking state machine:
`WAIT_FCU → PREARM_STREAM → WAIT_OFFBOARD → WAIT_ARM → TAKEOFF → ORBIT → RETURN_HOME → WAIT_LAND → DONE`
- Streams setpoints at 20 Hz (required by PX4 before OFFBOARD mode is accepted).
- All MAVROS service calls (`SetMode`, `CommandBool`) use `call_async` +
  `add_done_callback`; transitions are confirmed by polling `current_state`.
- At each orbit waypoint, holds position for a configurable dwell time before
  publishing `std_msgs/Empty` to `/nbv/capture`.
- Yaw faces inward toward the orbit centre throughout the flight.

**`image_saver_node.py`** — Trigger-based image saver:
- Caches the latest `sensor_msgs/Image` from the Gazebo camera topic (bridged
  by `ros_gz_bridge`) without writing to disk on every frame.
- On each `/nbv/capture` trigger, converts via `cv_bridge` and writes
  `frame_000.png`, `frame_001.png`, … to `~/nbv_images/`.

**`mission.launch.py`** — Launches four nodes:
1. `mavros_node` — MAVLink ↔ ROS 2 bridge (`udp://:14540@localhost:14557`)
2. `parameter_bridge` — Gazebo camera image + camera_info → ROS 2
3. `mission_node`
4. `image_saver_node`

---

## Phase 2: Custom World + Camera Verification

**Goal:** Build a Gazebo world containing a target object and confirm the drone
camera can see it — without flying the drone.

**Completion criteria:** Launch PX4 SITL with the custom world, bridge the
camera to ROS 2, run `snapshot_node`, and save a PNG that shows the target
object.

### What was built

**`nbv_scene.sdf`** — Custom Gazebo world at
`PX4-Autopilot/Tools/simulation/gz/worlds/nbv_scene.sdf`:
- Identical physics / ground plane / sun to `default.sdf` so PX4 SITL works
  unchanged.
- Adds a SUV

**`snapshot_node.py`** — New standalone ROS 2 node in `nbv_demo`:
- Subscribes to the Gazebo camera topic (via `ros_gz_bridge`).
- Waits for the first frame, saves it as `snapshot.png` to `~/nbv_images/`,
  then shuts down cleanly.
- No MAVROS, no flight — purely for camera pipeline verification.

### Launch sequence (Phase 2)

```bash
# Terminal A — PX4 SITL with the custom world (no conda env needed)
./scripts/launch_gz_world.sh

# Terminal B — Bridge camera topics to ROS 2
ros2 run ros_gz_bridge parameter_bridge "/world/nbv_scene/model/x500_mono_cam_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image" "/world/nbv_scene/model/x500_mono_cam_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"


# Terminal C — Rebuild and grab one snapshot
colcon build --packages-select nbv_demo && source install/setup.zsh
ros2 run nbv_demo snapshot_node --ros-args --params-file src/nbv_demo/config/mission_params.yaml
# → ~/nbv_images/snapshot.png should show the SUV
```

---

### Bug Fix: MAVROS crash on launch

**Symptom:** `mavros_node` died immediately with `invalid allocator` /
`rclcpp::exceptions::RCLError`.

**Root cause:** In `ros-jazzy-mavros`, the `companion_process_status` plugin
attempts to create a subscription to a topic already registered by
`sys_status` with an incompatible message type. RCL mishandles the resulting
error, corrupts its allocator state, and aborts the process.

**Fix:** Added `plugin_blocklist` to the MAVROS node parameters in the launch
file to skip the offending plugin and other unused plugins:

```python
'plugin_blocklist': [
    'companion_process_status',   # root cause of crash
    'adsb',
    'cellular_status',
    'cam_imu_sync',
]
```

## Phase 3: Autonomous Orbit Mission

**Goal:** Fly the drone autonomously around the target object, capturing images
at N equidistant waypoints to produce a complete photogrammetric dataset.

**Completion criteria:** The drone arms, climbs, orbits the SUV at the
configured radius and height, fires `/nbv/capture` at each waypoint, lands,
and `data/captures/` contains N PNG frames.

**Status: Complete.**

### What was built / wired up

The flight logic was already implemented in `mission_node.py` during exploratory
development.  Phase 3 wires it together with the custom world:

**`mission_params.yaml` — orbit centre updated:**
- `target_x: 5.0` — matches the SUV position in `nbv_scene.sdf`.
- Orbit at radius 5 m, height 3 m, 36 images (every 10°).

**`scripts/launch_gz_world.sh` — symlink fixed:**
- Now uses `ln -sf` (force) so any edit to `worlds/nbv_scene.sdf` is
  immediately reflected in PX4's worlds directory without a manual copy.

**`scripts/run_mission.sh` — new:**
- Activates the `nerfly` conda env, sources ROS 2 Jazzy, builds `nbv_demo`,
  sources the install overlay, and launches `mission.launch.py`.

### Launch sequence (Phase 3)

```bash
# Terminal A — PX4 SITL with the custom world
bash scripts/launch_gz_world.sh

# Terminal B — Full mission (MAVROS + bridge + orbit + image saver)
bash scripts/run_mission.sh
# → watch data/captures/ fill up with frame_000.png … frame_099.png
```

### Mission flow recap

```
WAIT_FCU → PREARM_STREAM → WAIT_OFFBOARD → WAIT_ARM →
TAKEOFF  → ORBIT (36 WPs, capture at each) → RETURN_HOME → WAIT_LAND → DONE
```

- Setpoints streamed at 20 Hz throughout (PX4 OFFBOARD requirement).
- At each waypoint: hold for `waypoint_dwell` (0.5 s) then fire `/nbv/capture`.
- `image_saver_node` catches each trigger and writes the latest camera frame.

**Image output directory:** `data/captures/` (workspace root, gitignored except
for the `.gitkeep` placeholder).  Configured via `image_save_dir` in
`mission_params.yaml`.