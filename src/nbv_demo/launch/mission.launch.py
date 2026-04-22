#!/usr/bin/env python3
"""
mission.launch.py — Launch the full NBV demo pipeline.

What this file starts
----------------------
1. mavros_node        — bridges ROS 2 ↔ PX4 SITL over MAVLink (UDP).
2. ros_gz_bridge      — one-way bridge: Gazebo camera image/info → ROS 2.
3. mission_node       — autonomous circular orbit state machine.
4. image_saver_node   — saves triggered frames to ~/nbv_images/.

Prerequisites (must be running BEFORE this launch file)
-------------------------------------------------------
  Terminal A:  PX4 SITL + Gazebo Harmonic
      bash scripts/launch_gz_world.sh

  (PX4 SITL opens a MAVLink UDP port at localhost:14540 for MAVROS.)

Usage
-----
  # From the workspace root (after colcon build):
  bash scripts/run_mission.sh

Optional overrides
------------------
  ros2 launch nbv_demo mission.launch.py orbit_radius:=8.0 n_images:=72
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share        = get_package_share_directory('nbv_demo')
    params_file      = os.path.join(pkg_share, 'config', 'mission_params.yaml')
    # MAVROS plugin denylist loaded as a YAML file — the reliable method.
    # Passing plugin lists as inline Python dict parameters does not work
    # because MAVROS reads the denylist before the ROS 2 parameter server is
    # fully initialised.  A YAML file loaded at launch time is applied first.
    pluginlists_file = os.path.join(pkg_share, 'config', 'mavros_pluginlists.yaml')

    # ── 1. MAVROS2 ────────────────────────────────────────────────────────────
    # Connects to PX4 SITL via UDP.  Default PX4 SITL MAVLink port is 14540.
    # gcs_url='' disables the GCS forwarding port (not needed for a demo).
    #
    # The companion_process_status plugin causes a known crash in ros-jazzy-mavros:
    # it subscribes to a topic already registered by sys_status with an
    # incompatible type, which corrupts RCL's allocator state and aborts the node.
    # mavros_pluginlists.yaml denies that plugin (and others unused by this mission).
    mavros_node = Node(
        package    = 'mavros',
        executable = 'mavros_node',
        # name       = 'mavros',
        output     = 'screen',
        parameters=[{'fcu_url': 'udp://:14540@', 'gcs_url': '',}],
        # parameters = [
        #     pluginlists_file,   # plugin_denylist — must come before the dict
        #     {
        #         'fcu_url':                'udp://:14540@localhost:14557',
        #         'gcs_url':                '',
        #         'target_system_id':       1,
        #         'target_component_id':    1,
        #         'local_position/tf/send': True,
        #     },
        # ],
    )

    # ── 2. ros_gz_bridge: Gazebo camera topics → ROS 2 ───────────────────────
    # Bridge string format for one-way gz→ros: gz_topic@ros_type[gz_type
    # The '[' indicates the message flows FROM Gazebo INTO ROS 2 only.
    #
    # The world name must match PX4_GZ_WORLD and the <world name="..."> in the
    # SDF.  Gazebo namespaces all topics under /world/<gz_world>/...
    gz_world = 'nbv_scene'
    gz_base  = (
        f'/world/{gz_world}/model/x500_mono_cam_0'
        f'/link/camera_link/sensor/camera'
    )
    bridge_node = Node(
        package    = 'ros_gz_bridge',
        executable = 'parameter_bridge',
        # name       = 'gz_ros_bridge',
        output     = 'screen',
        arguments  = [
            # Camera raw image: gz → ros
            f'{gz_base}/image'
            '@sensor_msgs/msg/Image'
            '[gz.msgs.Image',

            # Camera calibration info: gz → ros
            f'{gz_base}/camera_info'
            '@sensor_msgs/msg/CameraInfo'
            '[gz.msgs.CameraInfo',
        ],
    )

    # ── 3. Mission node ───────────────────────────────────────────────────────
    mission_node = Node(
        package    = 'nbv_demo',
        executable = 'mission_node',
        name       = 'mission_node',
        output     = 'screen',
        parameters = [params_file],
    )

    # ── 4. Image saver node ───────────────────────────────────────────────────
    image_saver_node = Node(
        package    = 'nbv_demo',
        executable = 'image_saver_node',
        name       = 'image_saver_node',
        output     = 'screen',
        parameters = [params_file],
    )

    return LaunchDescription([
        mavros_node,
        bridge_node,
        mission_node,
        image_saver_node,
    ])
