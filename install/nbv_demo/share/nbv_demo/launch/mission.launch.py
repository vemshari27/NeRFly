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
      cd ~/PX4-Autopilot
      make px4_sitl gz_x500_mono_cam

  (PX4 SITL opens a MAVLink UDP port at localhost:14540 for MAVROS.)

Usage
-----
  # From the workspace root:
  source install/setup.bash
  ros2 launch nbv_demo mission.launch.py

Optional overrides
------------------
  ros2 launch nbv_demo mission.launch.py orbit_radius:=8.0 n_images:=72
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Path to the installed config directory (populated by setup.py data_files).
    pkg_share  = get_package_share_directory('nbv_demo')
    params_file = os.path.join(pkg_share, 'config', 'mission_params.yaml')

    # ── 1. MAVROS2 ────────────────────────────────────────────────────────────
    # Connects to PX4 SITL via UDP.  Default PX4 SITL MAVLink port is 14540.
    # gcs_url='' disables the GCS forwarding port (not needed for a demo).
    #
    # plugin_blocklist: There is a known crash in ros-jazzy-mavros where the
    # companion_process_status plugin attempts to create a subscription to a
    # topic that already exists with an incompatible type, which cascades into
    # an 'invalid allocator' error and kills the node.  Blocking that plugin
    # avoids the crash.  None of the plugins below are needed for this mission.
    mavros_node = Node(
        package    = 'mavros',
        executable = 'mavros_node',
        name       = 'mavros',
        output     = 'screen',
        parameters = [
            {
                'fcu_url':             'udp://:14540@localhost:14557',
                'gcs_url':             '',
                'target_system_id':    1,
                'target_component_id': 1,
                # Publish TF so rviz2 can visualise the vehicle frame.
                'local_position/tf/send': True,
                # Block plugins that conflict or are unused by this mission.
                'plugin_blocklist': [
                    'companion_process_status',  # causes topic-type conflict crash
                    'adsb',
                    'cellular_status',
                    'cam_imu_sync',
                ],
            }
        ],
    )

    # ── 2. ros_gz_bridge: Gazebo camera topics → ROS 2 ───────────────────────
    # Bridge string format for one-way gz→ros: gz_topic@ros_type[gz_type
    # The '[' indicates the message flows FROM Gazebo INTO ROS 2 only.
    #
    # The world name must match PX4_GZ_WORLD and the <world name="..."> in the
    # SDF.  Gazebo namespaces all topics under /world/<gz_world>/...
    gz_world = 'nbv_scene'
    gz_base = (
        f'/world/{gz_world}/model/x500_mono_cam_0'
        f'/link/camera_link/sensor/camera'
    )
    bridge_node = Node(
        package    = 'ros_gz_bridge',
        executable = 'parameter_bridge',
        name       = 'gz_ros_bridge',
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
