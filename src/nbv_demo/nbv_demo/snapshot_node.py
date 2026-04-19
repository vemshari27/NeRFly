#!/usr/bin/env python3
"""
snapshot_node.py — Save one camera frame to disk and exit.

Purpose
-------
Phase 2 verification tool.  Run this node while PX4 SITL and the
ros_gz_bridge are active.  It subscribes to the Gazebo camera topic,
waits for the first frame to arrive, writes it to disk as
`snapshot.png`, and then shuts down cleanly.

This lets you confirm that:
  1. ros_gz_bridge is publishing camera frames.
  2. The drone camera can see the target object in the world.

Usage
-----
  # Terminal A: PX4 SITL with the custom world
  PX4_GZ_WORLD=nbv_scene make px4_sitl gz_x500_mono_cam

  # Terminal B: Bridge the camera topics (world name must match gz_world param)
  source /opt/ros/jazzy/setup.bash
  ros2 run ros_gz_bridge parameter_bridge \\
    "/world/nbv_scene/model/x500_mono_cam_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image" \\
    "/world/nbv_scene/model/x500_mono_cam_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"

  # Terminal C: Grab one frame
  source install/setup.bash
  ros2 run nbv_demo snapshot_node

Parameters
----------
image_save_dir  str  Directory to write snapshot.png into  (default ~/nbv_images)

Output
------
  <image_save_dir>/snapshot.png
"""

import os
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class SnapshotNode(Node):

    def __init__(self):
        super().__init__('snapshot_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('image_save_dir', '~/nbv_images')
        self.declare_parameter('gz_world',       'nbv_scene_2')

        raw_dir       = self.get_parameter('image_save_dir').value
        gz_world      = self.get_parameter('gz_world').value
        self.save_dir = os.path.expanduser(raw_dir)

        # Camera topic is namespaced under the Gazebo world name.
        # If PX4_GZ_WORLD=nbv_scene then Gazebo publishes on /world/nbv_scene/...
        camera_topic = (
            f'/world/{gz_world}/model/x500_mono_cam_0'
            f'/link/camera_link/sensor/camera/image'
        )

        os.makedirs(self.save_dir, exist_ok=True)

        # ── Internal state ────────────────────────────────────────────────────
        self.bridge      = CvBridge()
        self._saved      = False   # becomes True once the snapshot is written

        # ── QoS: Gazebo bridge publishes with BEST_EFFORT ─────────────────────
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscriber ────────────────────────────────────────────────────────
        self.create_subscription(
            Image,
            camera_topic,
            self._image_cb,
            qos_be,
        )

        self.get_logger().info(
            f'SnapshotNode waiting for first camera frame on:\n'
            f'  {camera_topic}'
        )

    def _image_cb(self, msg: Image):
        """On the first frame received, save it and request shutdown."""

        # Guard: only save once even if multiple frames arrive before shutdown.
        if self._saved:
            return
        self._saved = True

        # Convert ROS Image → OpenCV BGR array.
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge conversion failed: {exc}')
            return

        filepath = os.path.join(self.save_dir, 'snapshot.png')
        success  = cv2.imwrite(filepath, cv_image)

        if success:
            h, w = cv_image.shape[:2]
            self.get_logger().info(
                f'Snapshot saved ({w}×{h} px) → {filepath}'
            )
        else:
            self.get_logger().error(f'cv2.imwrite failed for path: {filepath}')
            return

        # Ask the executor to stop so the process exits cleanly.
        self.get_logger().info('Done — shutting down.')
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = SnapshotNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
