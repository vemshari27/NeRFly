#!/usr/bin/env python3
"""
mission_node.py — Autonomous circular orbit mission using MAVROS2.

Mission flow
------------
1.  WAIT_FCU       — spin until /mavros/state reports connected=True.
2.  PREARM_STREAM  — publish setpoints at 20 Hz for ≥2 s.  PX4 SITL will
                     reject OFFBOARD mode if setpoints haven't been streaming
                     before the mode request.
3.  WAIT_OFFBOARD  — send SetMode(OFFBOARD); poll current_state.mode to confirm.
4.  WAIT_ARM       — send CommandBool(arm=True); poll current_state.armed.
5.  TAKEOFF        — stream setpoint above orbit centre until altitude reached.
6.  ORBIT          — visit N equally-spaced waypoints on a horizontal circle.
                     At each waypoint:
                       (a) hold position for `waypoint_dwell` seconds, then
                       (b) publish std_msgs/Empty to /nbv/capture.
7.  RETURN_HOME    — fly back directly above orbit centre at orbit altitude.
8.  WAIT_LAND      — switch to AUTO.LAND; wait for disarm.
9.  DONE           — mission complete.

All service calls (SetMode, CommandBool) are non-blocking (call_async + done
callback).  The state machine polls current_state to confirm each transition
instead of blocking on the future.  This is the correct ROS 2 pattern — it
avoids deadlocks when spinning a single-threaded executor.

Parameters (loaded from mission_params.yaml)
--------------------------------------------
orbit_radius       float  Orbit circle radius in metres          (default 5.0)
orbit_height       float  Flight altitude in metres AGL          (default 3.0)
n_images           int    Number of capture waypoints            (default 36)
target_x           float  Orbit centre X in local ENU frame      (default 0.0)
target_y           float  Orbit centre Y in local ENU frame      (default 0.0)
waypoint_tolerance float  Distance threshold to "reach" a WP (m) (default 0.3)
waypoint_dwell     float  Hold time at each WP before capture (s) (default 0.5)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Empty


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        # ── Declare and read parameters ───────────────────────────────────────
        self.declare_parameter('orbit_radius',       5.0)
        self.declare_parameter('orbit_height',       3.0)
        self.declare_parameter('n_images',           36)
        self.declare_parameter('target_x',           0.0)
        self.declare_parameter('target_y',           0.0)
        self.declare_parameter('waypoint_tolerance', 0.3)
        self.declare_parameter('waypoint_dwell',     0.5)

        self.orbit_radius  = self.get_parameter('orbit_radius').value
        self.orbit_height  = self.get_parameter('orbit_height').value
        self.n_images      = self.get_parameter('n_images').value
        self.target_x      = self.get_parameter('target_x').value
        self.target_y      = self.get_parameter('target_y').value
        self.wp_tolerance  = self.get_parameter('waypoint_tolerance').value
        self.wp_dwell      = self.get_parameter('waypoint_dwell').value

        # ── Internal state ────────────────────────────────────────────────────
        self.current_state  = State()        # latest /mavros/state message
        self.current_pose   = PoseStamped()  # latest /mavros/local_position/pose
        self.setpoint       = PoseStamped()  # the position we continuously stream

        self._mission_step  = 'WAIT_FCU'
        self._step_start    = self.get_clock().now()

        self._waypoints     = []    # list of (x, y, z, yaw_rad) tuples
        self._wp_index      = 0     # index into _waypoints
        self._wp_arrived_at = None  # clock.Time when current WP was first reached

        # ── QoS profile ───────────────────────────────────────────────────────
        # MAVROS publishes state and pose with BEST_EFFORT reliability.
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            State,
            '/mavros/state',
            self._state_cb,
            qos_be,
        )
        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._pose_cb,
            qos_be,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10,
        )
        self.trigger_pub = self.create_publisher(
            Empty,
            '/nbv/capture',
            10,
        )

        # ── Service clients ───────────────────────────────────────────────────
        self.set_mode_client = self.create_client(SetMode,     '/mavros/set_mode')
        self.arming_client   = self.create_client(CommandBool, '/mavros/cmd/arming')

        # ── Timers ────────────────────────────────────────────────────────────
        # Stream the current setpoint at 20 Hz.  PX4 will exit OFFBOARD mode
        # if setpoints stop arriving for more than ~0.5 s.
        self.create_timer(0.05, self._stream_setpoint_cb)

        # Run the mission state machine at 10 Hz.
        self.create_timer(0.10, self._mission_tick)

        self.get_logger().info(
            f'MissionNode ready | '
            f'radius={self.orbit_radius} m  '
            f'height={self.orbit_height} m  '
            f'N={self.n_images} images'
        )

    # ── Topic callbacks ───────────────────────────────────────────────────────

    def _state_cb(self, msg: State):
        self.current_state = msg

    def _pose_cb(self, msg: PoseStamped):
        self.current_pose = msg

    # ── Setpoint stream ───────────────────────────────────────────────────────

    def _stream_setpoint_cb(self):
        """Publish the current setpoint at 20 Hz.

        This must run continuously (even before OFFBOARD mode) because PX4
        requires a stream of setpoints to already be flowing before it will
        accept a SetMode(OFFBOARD) request.
        """
        self.setpoint.header.stamp    = self.get_clock().now().to_msg()
        self.setpoint.header.frame_id = 'map'
        self.setpoint_pub.publish(self.setpoint)

    # ── Helper: update target setpoint ───────────────────────────────────────

    def _set_setpoint(self, x: float, y: float, z: float, yaw_rad: float = 0.0):
        """Update the position and yaw of the streamed setpoint.

        The quaternion encodes only a pure rotation about Z (yaw only), which
        is all we need for horizontal orbit flight.
        """
        self.setpoint.pose.position.x    = x
        self.setpoint.pose.position.y    = y
        self.setpoint.pose.position.z    = z
        # Yaw-only quaternion: q = [0, 0, sin(yaw/2), cos(yaw/2)]
        self.setpoint.pose.orientation.x = 0.0
        self.setpoint.pose.orientation.y = 0.0
        self.setpoint.pose.orientation.z = math.sin(yaw_rad / 2.0)
        self.setpoint.pose.orientation.w = math.cos(yaw_rad / 2.0)

    # ── Helper: distance to current setpoint ─────────────────────────────────

    def _dist_to_setpoint(self) -> float:
        """Euclidean distance (metres) from current pose to the active setpoint."""
        p = self.current_pose.pose.position
        s = self.setpoint.pose.position
        return math.sqrt((p.x - s.x)**2 + (p.y - s.y)**2 + (p.z - s.z)**2)

    # ── Helper: state machine bookkeeping ────────────────────────────────────

    def _seconds_in_step(self) -> float:
        """Seconds elapsed since the current step started."""
        return (self.get_clock().now() - self._step_start).nanoseconds / 1e9

    def _go_to_step(self, name: str):
        """Advance to a new state machine step and reset the step timer."""
        self.get_logger().info(f'[Mission] ── {name}')
        self._mission_step = name
        self._step_start   = self.get_clock().now()

    # ── Helper: build orbit waypoints ────────────────────────────────────────

    def _build_orbit_waypoints(self):
        """Return a list of N (x, y, z, yaw_rad) tuples.

        Waypoints are equally spaced around a horizontal circle of radius
        `orbit_radius` centred on (target_x, target_y) at height
        `orbit_height`.  Yaw is set so the drone faces inward toward the
        orbit centre at every point (good for photogrammetry).
        """
        waypoints = []
        for i in range(self.n_images):
            angle = 2.0 * math.pi * i / self.n_images
            x = self.target_x + self.orbit_radius * math.cos(angle)
            y = self.target_y + self.orbit_radius * math.sin(angle)
            z = self.orbit_height
            # atan2 gives the bearing FROM this point TOWARD the centre
            yaw = math.atan2(self.target_y - y, self.target_x - x)
            waypoints.append((x, y, z, yaw))
        return waypoints

    # ── Async service request helpers ─────────────────────────────────────────

    def _request_mode(self, mode: str):
        """Send a non-blocking SetMode request to MAVROS.

        The state machine does NOT block here.  It instead polls
        current_state.mode in the next ticks to confirm the transition.
        """
        if not self.set_mode_client.service_is_ready():
            self.get_logger().warn('/mavros/set_mode not ready — will retry.')
            return
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info(
            f'SetMode({mode}) response: '
            f'{"accepted" if f.result() and f.result().mode_sent else "rejected"}'
        ))

    def _request_arm(self, arm: bool):
        """Send a non-blocking CommandBool arming/disarming request."""
        if not self.arming_client.service_is_ready():
            self.get_logger().warn('/mavros/cmd/arming not ready — will retry.')
            return
        req = CommandBool.Request()
        req.value = arm
        future = self.arming_client.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info(
            f'Arm({arm}) response: '
            f'{"accepted" if f.result() and f.result().success else "rejected"}'
        ))

    # ── Mission state machine (10 Hz) ─────────────────────────────────────────

    def _mission_tick(self):  # noqa: C901
        """Drive the mission state machine.  Called at 10 Hz by a timer."""

        step = self._mission_step

        # ── Step 1: Wait for MAVROS to connect to PX4 ─────────────────────────
        if step == 'WAIT_FCU':
            if self.current_state.connected:
                self.get_logger().info('FCU connected.')
                # Seed the setpoint at the origin so the stream starts cleanly.
                self._set_setpoint(0.0, 0.0, 0.0)
                self._go_to_step('PREARM_STREAM')

        # ── Step 2: Stream setpoints for ≥2 s before requesting OFFBOARD ──────
        elif step == 'PREARM_STREAM':
            # PX4 will reject OFFBOARD mode unless setpoints have been arriving
            # continuously.  Two seconds of streaming is a safe margin.
            if self._seconds_in_step() >= 2.0:
                self._request_mode('OFFBOARD')
                self._go_to_step('WAIT_OFFBOARD')

        # ── Step 3: Wait for OFFBOARD mode confirmation ────────────────────────
        elif step == 'WAIT_OFFBOARD':
            if self.current_state.mode == 'OFFBOARD':
                self.get_logger().info('OFFBOARD mode confirmed.')
                self._request_arm(True)
                self._go_to_step('WAIT_ARM')
            elif self._seconds_in_step() > 5.0:
                # PX4 may reject OFFBOARD if the vehicle is not ready.  Retry.
                self.get_logger().warn('OFFBOARD timeout — retrying SetMode request.')
                self._request_mode('OFFBOARD')
                self._step_start = self.get_clock().now()  # reset timeout

        # ── Step 4: Wait for arming confirmation ───────────────────────────────
        elif step == 'WAIT_ARM':
            if self.current_state.armed:
                self.get_logger().info('Armed. Climbing to orbit altitude …')
                self._set_setpoint(self.target_x, self.target_y, self.orbit_height)
                self._go_to_step('TAKEOFF')
            elif self._seconds_in_step() > 5.0:
                self.get_logger().warn('ARM timeout — retrying arming request.')
                self._request_arm(True)
                self._step_start = self.get_clock().now()

        # ── Step 5: Climb to orbit altitude ────────────────────────────────────
        elif step == 'TAKEOFF':
            # Setpoint was set when we entered this step; just wait to arrive.
            if self._dist_to_setpoint() < self.wp_tolerance:
                self.get_logger().info(
                    f'Reached altitude {self.orbit_height:.1f} m. '
                    f'Computing {self.n_images} orbit waypoints …'
                )
                self._waypoints     = self._build_orbit_waypoints()
                self._wp_index      = 0
                self._wp_arrived_at = None
                self._go_to_step('ORBIT')

        # ── Step 6: Circular orbit with image capture ──────────────────────────
        elif step == 'ORBIT':
            # All waypoints visited — move on to landing.
            if self._wp_index >= len(self._waypoints):
                self.get_logger().info('Orbit complete. Returning to landing point.')
                self._set_setpoint(self.target_x, self.target_y, self.orbit_height)
                self._go_to_step('RETURN_HOME')
                return

            x, y, z, yaw = self._waypoints[self._wp_index]
            self._set_setpoint(x, y, z, yaw)

            if self._dist_to_setpoint() < self.wp_tolerance:
                # First time we've arrived at this waypoint: start the dwell timer.
                if self._wp_arrived_at is None:
                    self._wp_arrived_at = self.get_clock().now()

                # Check whether we've been stationary long enough.
                dwell_s = (self.get_clock().now() - self._wp_arrived_at).nanoseconds / 1e9
                if dwell_s >= self.wp_dwell:
                    self.get_logger().info(
                        f'  WP {self._wp_index + 1}/{self.n_images} '
                        f'at ({x:.2f}, {y:.2f}, {z:.2f}) — CAPTURE'
                    )
                    self.trigger_pub.publish(Empty())
                    self._wp_index     += 1
                    self._wp_arrived_at = None  # reset for next waypoint
            else:
                # Drifted outside tolerance — reset the dwell timer.
                self._wp_arrived_at = None

        # ── Step 7: Return to hover above orbit centre before landing ──────────
        elif step == 'RETURN_HOME':
            if self._dist_to_setpoint() < self.wp_tolerance:
                self.get_logger().info('Over landing point. Engaging AUTO.LAND …')
                self._request_mode('AUTO.LAND')
                self._go_to_step('WAIT_LAND')

        # ── Step 8: Wait for AUTO.LAND to disarm the drone ────────────────────
        elif step == 'WAIT_LAND':
            # PX4 disarms automatically after touchdown.
            if not self.current_state.armed:
                self.get_logger().info('Drone disarmed — mission complete!')
                self._go_to_step('DONE')

        elif step == 'DONE':
            pass  # Nothing left to do; node keeps spinning so ROS 2 stays alive.


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
