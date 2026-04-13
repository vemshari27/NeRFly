#!/usr/bin/env python3
"""
image_saver_node.py — Save camera frames to disk on demand.

How it works
------------
- Subscribes to the Gazebo camera image topic and caches the most recent frame
  in memory (no disk I/O on every frame).
- Subscribes to /nbv/capture (std_msgs/Empty) for trigger messages.
- On each trigger, converts the cached sensor_msgs/Image to an OpenCV BGR
  image via cv_bridge and writes it as a PNG to `image_save_dir`.
- Files are named frame_000.png, frame_001.png, … (zero-padded to 3 digits).

Parameters (loaded from mission_params.yaml)
--------------------------------------------
image_save_dir  str  Directory to write images into  (default ~/nbv_images)

Topics
------
Subscribed:
  /world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image
      sensor_msgs/Image   — raw camera frames from Gazebo (via ros_gz_bridge)
  /nbv/capture
      std_msgs/Empty      — capture trigger from mission_node

Notes
-----
- The camera topic is published by ros_gz_bridge with BEST_EFFORT reliability,
  so the subscriber QoS is set to match.
- If a trigger arrives before the first camera frame has been received, the
  trigger is silently skipped with a warning (no crash).
"""

import os
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from cv_bridge import CvBridge


# Full Gazebo camera topic as published by ros_gz_bridge.
CAMERA_TOPIC = (
    '/world/default/model/x500_mono_cam_0'
    '/link/camera_link/sensor/camera/image'
)


class ImageSaverNode(Node):

    def __init__(self):
        super().__init__('image_saver_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('image_save_dir', '~/nbv_images')
        raw_dir        = self.get_parameter('image_save_dir').value
        self.save_dir  = os.path.expanduser(raw_dir)

        os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info(f'Images will be saved to: {self.save_dir}')

        # ── Internal state ────────────────────────────────────────────────────
        self.bridge      = CvBridge()
        self.latest_img  = None   # most recent sensor_msgs/Image (not yet saved)
        self.frame_count = 0      # increments with each successful save

        # ── QoS: Gazebo bridge uses BEST_EFFORT ───────────────────────────────
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,   # only the newest frame matters
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self._image_cb,
            qos_be,
        )
        self.create_subscription(
            Empty,
            '/nbv/capture',
            self._trigger_cb,
            10,
        )

        self.get_logger().info(
            f'ImageSaverNode ready.\n'
            f'  Listening on camera topic : {CAMERA_TOPIC}\n'
            f'  Listening for triggers on : /nbv/capture'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        """Cache the latest frame.  We do not save here — wait for a trigger."""
        self.latest_img = msg

    def _trigger_cb(self, _msg: Empty):
        """Save the most recently received frame when a trigger arrives."""

        if self.latest_img is None:
            self.get_logger().warn(
                'Trigger received but no camera frame available yet — skipping.'
            )
            return

        # Convert ROS Image → OpenCV BGR array.
        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                self.latest_img, desired_encoding='bgr8'
            )
        except Exception as exc:
            self.get_logger().error(f'cv_bridge conversion failed: {exc}')
            return

        # Build output path: frame_000.png, frame_001.png, …
        filename = f'frame_{self.frame_count:03d}.png'
        filepath = os.path.join(self.save_dir, filename)

        success = cv2.imwrite(filepath, cv_image)
        if not success:
            self.get_logger().error(f'cv2.imwrite failed for path: {filepath}')
            return

        self.get_logger().info(
            f'Saved frame {self.frame_count:03d} → {filepath}'
        )
        self.frame_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
