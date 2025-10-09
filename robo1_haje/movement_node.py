#!/usr/bin/env python3
# HAJE Movement Subsystem (ROS 2 Humble + Ignition/Gazebo)
#
# PASS scope:
# - /movement/go starts the mission; /movement/stop stops.
# - Executes ≥3 waypoints in order with arrival tolerances and per-WP hold.
# - Publishes geometry_msgs/Twist to drive the drone.
# - Publishes nav_msgs/Path on /movement/path (with proper header stamps).
#
# Assumptions:
# - Use simulator odometry: /model/<name>/odometry (defaults to drone), or /odometry via bridge.
#
# This version adds "face_forward" yaw control so the drone turns toward the
# direction of travel (publishes Twist.angular.z).
# HAJE Movement Subsystem (ROS 2 Humble + Gazebo/GZ)
# CREDIT tier: GUI goal subscription + queuing + defined path visualisation.
# HAJE Movement Subsystem (ROS 2 Humble + Gazebo/GZ)
# CREDIT tier: GUI goal subscription + queuing + defined path + markers.
# Distinction-grade Movement Subsystem (ROS 2 Humble + Ignition/Gazebo)
# Features:
# - GO/STOP/PAUSE/CONTINUE/SKIP/CLEAR + dynamic waypoint add
# - ≥3 WPs execution with hold; XY+optional Z control
# - Yaw control to face direction of travel
# - Acceleration limiting (x/y/z/yaw)
# - Odom watchdog -> safe hold on dropout
# - RViz Path + MarkerArray (TRANSIENT_LOCAL)
# - Optional geofence & lookahead follower
# - Works with /odometry or /odometry/filtered
# Movement Subsystem (ROS 2 Humble + Ignition/Gazebo)
# Adds /movement/launch and /movement/land on top of the distinction build.
#!/usr/bin/env python3
"""
Movement controller for the drone sim (ROS 2 Humble).

Core features
-------------
- Mission control via topics:
  /movement/go (Empty)           : start mission (optionally uses takeoff phase)
  /movement/stop (Empty)         : abort mission and idle (zero velocity)
  /movement/pause (Empty)        : pause (zero velocity, keep state)
  /movement/continue (Empty)     : resume after pause
  /movement/launch (Empty)       : climb to 'launch_altitude' and hover
  /movement/land (Empty)         : descend to 'land_touchdown_z' and idle
  /movement/waypoint (Point)     : append a waypoint {x,y,z}
  /movement/skip (Empty)         : skip to next waypoint
  /movement/clear (Empty)        : clear all waypoints

- Control:
  * PD-lite (P only) velocity in world XY (+ optional Z).
  * Optional yaw control to align with path heading.
  * Optional lookahead carrot (meters) for smoother arcs.
  * Optional geofence [xmin,xmax,ymin,ymax,zmin,zmax] clamps targets.
  * Per-waypoint hold time on arrival.

- Visualization & status:
  * nav_msgs/Path on /movement/path.
  * visualization_msgs/MarkerArray on /movement/markers.
  * std_msgs/String status lines on /movement/status.

Assumptions
-----------
- Odometry is available on 'odom_topic' (default: /odometry) in the world frame.
- /cmd_vel is bridged to the Gazebo model's Twist input (your bringup already does this).
"""

"""
HAJE Movement Subsystem (ROS 2 Humble + Gazebo/ros_gz)

Features
- GO / STOP / PAUSE / CONTINUE    (std_msgs/Empty)
- LAUNCH / LAND                   (std_msgs/Empty)
- Add waypoint, Skip, Clear       (geometry_msgs/Point + Empty)
- Publishes /movement/status      (std_msgs/String)
- Publishes /movement/path        (nav_msgs/Path)
- Publishes /movement/markers     (visualization_msgs/MarkerArray)
- Yaw-to-path (on/off), optional lookahead follower
- Optional geofence [xmin,xmax,ymin,ymax,zmin,zmax]
- Works with /odometry (Gazebo bridge) and /cmd_vel -> /model/drone/cmd_vel

Notes
- X/Y are commanded in world frame. Z control is optional.
- Yaw is controlled only if yaw_to_path==True (angular.z).
"""


"""
HAJE Movement Subsystem (ROS 2 Humble + Gazebo/ros_gz)

Features
- GO / STOP / PAUSE / CONTINUE    (std_msgs/Empty)
- LAUNCH / LAND                   (std_msgs/Empty)
- Add waypoint, Skip, Clear       (geometry_msgs/Point + Empty)
- Publishes /movement/status      (std_msgs/String)
- Publishes /movement/path        (nav_msgs/Path)
- Publishes /movement/markers     (visualization_msgs/MarkerArray)
- Yaw-to-path (on/off), optional lookahead follower
- Optional geofence [xmin,xmax,ymin,ymax,zmin,zmax]
"""

"""
HAJE Movement Subsystem (ROS 2 Humble)

State machine:
  GROUNDED -> (LAUNCH) -> TAKEOFF -> AWAIT_GO --(GO)--> MISSION
                                 \--(GO while taking off)--> remember & auto-enter MISSION
  From MISSION: PAUSE, LAND, STOP all allowed
  PAUSE -> CONTINUE returns to previous active state
  LAND always lands at current XY to land_touchdown_z, ends GROUNDED (wp index preserved)
  STOP lands and resets waypoint index to 0, ends GROUNDED

Topics (I/O):
  Sub:  /odometry (nav_msgs/Odometry)
        /movement/go         (std_msgs/Empty)
        /movement/stop       (std_msgs/Empty)
        /movement/pause      (std_msgs/Empty)
        /movement/continue   (std_msgs/Empty)
        /movement/launch     (std_msgs/Empty)
        /movement/land       (std_msgs/Empty)
        /movement/waypoint   (geometry_msgs/Point)
        /movement/skip       (std_msgs/Empty)
        /movement/clear      (std_msgs/Empty)

  Pub:  cmd_vel_topic (geometry_msgs/Twist)
        movement/status  (std_msgs/String)
        movement/path    (nav_msgs/Path)
        movement/markers (visualization_msgs/MarkerArray)
"""

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray


def quat_to_yaw(qx, qy, qz, qw) -> float:
    s = 2.0 * (qw * qz + qx * qy)
    c = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(s, c)

def clamp(v: float, vmin: float, vmax: float) -> float:
    return max(min(v, vmax), vmin)

def angle_wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class MovementNode(Node):
    def __init__(self):
        super().__init__("movement_controller")

        # ---- Parameters -----------------------------------------------------
        # I/O topics
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odometry")
        self.declare_parameter("world_frame", "map")

        # Behaviour toggles
        self.declare_parameter("skip_takeoff", False)   # set True to allow GO from ground without LAUNCH
        self.declare_parameter("use_z_control", False)  # follow Z in waypoints during MISSION
        self.declare_parameter("yaw_to_path", True)     # rotate to face velocity direction
        self.declare_parameter("lookahead_m", 0.0)      # >0 for smooth arcs

        # Altitudes
        self.declare_parameter("launch_altitude", 2.0)
        self.declare_parameter("land_touchdown_z", 0.1)

        # Gains & limits
        self.declare_parameter("kp_xy", 0.6)
        self.declare_parameter("kp_z", 0.8)
        self.declare_parameter("kp_yaw", 1.2)
        self.declare_parameter("max_xy", 0.5)
        self.declare_parameter("max_z", 0.5)
        self.declare_parameter("max_yaw_rate", 0.8)

        # Tolerances / timing
        self.declare_parameter("xy_tol", 0.35)
        self.declare_parameter("z_tol", 0.25)
        self.declare_parameter("wp_hold_sec", 2.0)
        self.declare_parameter("tick_hz", 20.0)

        # Waypoints (typed)
        self.declare_parameter(
            "waypoints",
            [0.0, 0.0, 2.0, 2.0, 0.0, 2.0, 2.0, 2.0, 2.0, 0.0, 2.0, 2.0],
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY),
        )

        # IMPORTANT: non-empty default so ROS types this as DOUBLE_ARRAY
        self.declare_parameter(
            "geofence",
            [0.0],  # length!=6 => geofence disabled
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY),
        )

        # STOP semantics: land & reset wp index (but keep list)
        self.declare_parameter("stop_resets_index", True)

        # ---- Resolve parameters --------------------------------------------
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value

        self.skip_takeoff = bool(self.get_parameter("skip_takeoff").value)
        self.use_z = bool(self.get_parameter("use_z_control").value)
        self.yaw_to_path = bool(self.get_parameter("yaw_to_path").value)
        self.lookahead_m = float(self.get_parameter("lookahead_m").value)
        self.launch_alt = float(self.get_parameter("launch_altitude").value)
        self.land_touchdown_z = float(self.get_parameter("land_touchdown_z").value)

        self.kp_xy = float(self.get_parameter("kp_xy").value)
        self.kp_z = float(self.get_parameter("kp_z").value)
        self.kp_yaw = float(self.get_parameter("kp_yaw").value)
        self.vmax_xy = float(self.get_parameter("max_xy").value)
        self.vmax_z = float(self.get_parameter("max_z").value)
        self.vmax_yaw = float(self.get_parameter("max_yaw_rate").value)

        self.xy_tol = float(self.get_parameter("xy_tol").value)
        self.z_tol = float(self.get_parameter("z_tol").value)
        self.wp_hold_sec = float(self.get_parameter("wp_hold_sec").value)
        tick_hz = float(self.get_parameter("tick_hz").value)
        self.dt = 1.0 / max(1e-6, tick_hz)

        wpa = list(self.get_parameter("waypoints").get_parameter_value().double_array_value)
        self.waypoints: List[Tuple[float, float, float]] = []
        for i in range(0, len(wpa), 3):
            self.waypoints.append((float(wpa[i]), float(wpa[i + 1]), float(wpa[i + 2])))

        gf = list(self.get_parameter("geofence").get_parameter_value().double_array_value)
        self.geofence: Optional[Tuple[float, float, float, float, float, float]] = (
            (gf[0], gf[1], gf[2], gf[3], gf[4], gf[5]) if len(gf) == 6 else None
        )

        self.stop_resets_index = bool(self.get_parameter("stop_resets_index").value)

        # ---- ROS I/O --------------------------------------------------------
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.path_pub = self.create_publisher(Path, "movement/path", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "movement/markers", 10)
        self.status_pub = self.create_publisher(String, "movement/status", 10)

        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 20)

        # Commands
        self.create_subscription(Empty, "movement/go", self._go_cb, 10)
        self.create_subscription(Empty, "movement/stop", self._stop_cb, 10)
        self.create_subscription(Empty, "movement/pause", self._pause_cb, 10)
        self.create_subscription(Empty, "movement/continue", self._continue_cb, 10)
        self.create_subscription(Empty, "movement/launch", self._launch_cb, 10)
        self.create_subscription(Empty, "movement/land", self._land_cb, 10)
        self.create_subscription(Point, "movement/waypoint", self._add_wp_cb, 10)
        self.create_subscription(Empty, "movement/skip", self._skip_cb, 10)
        self.create_subscription(Empty, "movement/clear", self._clear_cb, 10)

        # ---- Internal state -------------------------------------------------
        self.pos_xyz: Optional[Tuple[float, float, float]] = None
        self.yaw: float = 0.0

        self.state: str = "grounded"     # grounded | takeoff | await_go | mission | paused | landing
        self.prev_active_state: str = "mission"
        self.wp_i: int = 0
        self.hold_t: float = 0.0
        self.go_pending_after_takeoff: bool = False

        # Timers
        self.timer = self.create_timer(self.dt, self._tick)
        self.status_timer = self.create_timer(0.5, self._publish_status)

        self._publish_path_and_markers()

        self.get_logger().info(
            f"MovementNode up. cmd_vel={self.cmd_vel_topic}, odom={self.odom_topic}, "
            f"skip_takeoff={self.skip_takeoff}, z_ctrl={self.use_z}, yaw_to_path={self.yaw_to_path}, "
            f"lookahead={self.lookahead_m}m, geofence={'on' if self.geofence else 'off'}."
        )
        self.get_logger().info(
            "Waiting for TAKEOFF. When ready:  ros2 topic pub -1 /movement/launch std_msgs/msg/Empty \"{}\""
        )


    # --------------------------- Callbacks ----------------------------------
    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pos_xyz = (p.x, p.y, p.z)
        self.yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

    def _go_cb(self, _):
        if self.state == "paused":
            self._continue_cb(None); return
        if self.state == "await_go":
            self.state = "mission"
            self.get_logger().info("GO -> MISSION")
            return
        if self.state == "takeoff":
            self.go_pending_after_takeoff = True
            self.get_logger().info("GO noted; will enter MISSION at altitude.")
            return
        if self.state == "grounded":
            if self.skip_takeoff:
                if self.pos_xyz is None:
                    self.get_logger().warn("GO ignored: no odometry yet."); return
                self.get_logger().info("GO from ground (skip_takeoff=True) -> MISSION")
                self.state = "mission"
            else:
                self.get_logger().warn("GO ignored: LAUNCH first."); 
            return
        self.get_logger().warn("GO ignored: already active.")

    def _stop_cb(self, _):
        # emergency stop: land now and reset progress
        self.state = "landing"
        self.go_pending_after_takeoff = False
        if self.stop_resets_index:
            self.wp_i = 0
        self.hold_t = 0.0
        self._publish_vel(0.0, 0.0, 0.0, 0.0)
        self.get_logger().info("STOP -> LANDING (will end GROUNDED)")

    def _pause_cb(self, _):
        if self.state in ("mission", "takeoff", "await_go"):
            self.prev_active_state = self.state
            self.state = "paused"
            self._publish_vel(0.0, 0.0, 0.0, 0.0)
            self.get_logger().info("PAUSE")
        else:
            self.get_logger().warn("PAUSE ignored (not active).")

    def _continue_cb(self, _):
        if self.state == "paused":
            self.state = self.prev_active_state
            self.get_logger().info(f"CONTINUE -> {self.state.upper()}")
        elif self.state == "grounded":
            # convenience: continue from ground behaves like launch
            self._launch_cb(None)
        else:
            self.get_logger().warn("CONTINUE ignored.")

    def _launch_cb(self, _):
        if self.pos_xyz is None:
            self.get_logger().warn("LAUNCH ignored: no odometry yet."); return
        self.state = "takeoff"
        self.hold_t = 0.0
        self.get_logger().info("LAUNCH -> TAKEOFF")

    def _land_cb(self, _):
        if self.pos_xyz is None:
            self.get_logger().warn("LAND ignored: no odometry yet."); return
        self.state = "landing"
        self.hold_t = 0.0
        self.get_logger().info("LAND -> LANDING")

    def _add_wp_cb(self, pt: Point):
        self.waypoints.append((float(pt.x), float(pt.y), float(pt.z)))
        self._publish_path_and_markers()
        self.get_logger().info(f"Added waypoint ({pt.x:.2f},{pt.y:.2f},{pt.z:.2f}); total={len(self.waypoints)}")

    def _skip_cb(self, _):
        if self.wp_i < len(self.waypoints):
            self.wp_i += 1; self.hold_t = 0.0
            self.get_logger().info(f"Skipped to waypoint index {self.wp_i}")
        else:
            self.get_logger().info("Skip ignored: already past last waypoint.")

    def _clear_cb(self, _):
        self.waypoints.clear(); self.wp_i = 0; self.hold_t = 0.0
        self._publish_path_and_markers()
        self.get_logger().info("Cleared all waypoints.")

    # --------------------------- Control loop --------------------------------
    def _tick(self):
        if self.pos_xyz is None:
            return
        x, y, z = self.pos_xyz

        if self.state == "grounded":
            return

        if self.state == "takeoff":
            # Vertical only
            target_z = self.launch_alt
            vz = clamp(self.kp_z * (target_z - z), -self.vmax_z, self.vmax_z)
            if abs(target_z - z) < self.z_tol:
                if self.hold_t <= 0.0:
                    self.hold_t = self.wp_hold_sec
                    self.get_logger().info("Reached launch altitude; holding...")
                else:
                    self.hold_t -= self.dt
                    if self.hold_t <= 0.0:
                        if self.go_pending_after_takeoff:
                            self.state = "mission"
                            self.get_logger().info("Takeoff complete -> MISSION (GO was pending).")
                            self.go_pending_after_takeoff = False
                        else:
                            self.state = "await_go"
                            self.get_logger().info("Takeoff complete -> AWAIT_GO.")
                        vz = 0.0
            self._publish_vel(0.0, 0.0, vz, 0.0)
            return

        if self.state == "await_go":
            # Hover until GO or LAND/STOP
            self._publish_vel(0.0, 0.0, 0.0, 0.0)
            return

        if self.state == "mission":
            if self.wp_i >= len(self.waypoints):
                self.get_logger().info("All waypoints done -> LANDING")
                self.state = "landing"
                return

            # Follow current waypoint
            tx, ty, tz = self._target_xyz(x, y, z)
            vz_cmd = clamp(self.kp_z * (tz - z), -self.vmax_z, self.vmax_z) if self.use_z else 0.0

            dx, dy = (tx - x), (ty - y)
            dist_xy = math.hypot(dx, dy)

            if dist_xy > self.xy_tol:
                # optional lookahead
                if self.lookahead_m > 0.0 and dist_xy > 1e-6:
                    L = min(self.lookahead_m, dist_xy)
                    ux, uy = dx / dist_xy, dy / dist_xy
                    gx, gy = x + ux * L, y + uy * L
                    ex, ey = (gx - x), (gy - y)
                else:
                    ex, ey = dx, dy

                vx_cmd = clamp(self.kp_xy * ex, -self.vmax_xy, self.vmax_xy)
                vy_cmd = clamp(self.kp_xy * ey, -self.vmax_xy, self.vmax_xy)

                if self.yaw_to_path and (abs(vx_cmd) + abs(vy_cmd)) > 1e-6:
                    hdg = math.atan2(vy_cmd, vx_cmd)
                    yaw_err = angle_wrap(hdg - self.yaw)
                    wz_cmd = clamp(self.kp_yaw * yaw_err, -self.vmax_yaw, self.vmax_yaw)
                else:
                    wz_cmd = 0.0

                self._publish_vel(vx_cmd, vy_cmd, vz_cmd, wz_cmd)
            else:
                # Hold briefly, then advance
                self._publish_vel(0.0, 0.0, 0.0, 0.0)
                if self.hold_t <= 0.0:
                    self.hold_t = self.wp_hold_sec
                    self.get_logger().info(
                        f"Reached waypoint {self.wp_i + 1}/{len(self.waypoints)}; holding..."
                    )
                else:
                    self.hold_t -= self.dt
                    if self.hold_t <= 0.0:
                        self.wp_i += 1
                        self.get_logger().info("Proceeding to next waypoint...")
            return

        if self.state == "paused":
            self._publish_vel(0.0, 0.0, 0.0, 0.0)
            return

        if self.state == "landing":
            target_z = self.land_touchdown_z
            dz = target_z - z
            if abs(dz) < self.z_tol:
                self._publish_vel(0.0, 0.0, 0.0, 0.0)
                self.state = "grounded"
                self.get_logger().info("Touchdown -> GROUNDED.")
                return
            vz = clamp(self.kp_z * dz, -self.vmax_z, self.vmax_z)
            self._publish_vel(0.0, 0.0, vz, 0.0)

    # ---- helpers ------------------------------------------------------------
    def _target_xyz(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        tx, ty, tz = self.waypoints[self.wp_i]
        if self.geofence:
            xmin, xmax, ymin, ymax, zmin, zmax = self.geofence
            tx = clamp(tx, xmin, xmax)
            ty = clamp(ty, ymin, ymax)
            tz = clamp(tz, zmin, zmax)
        return tx, ty, tz

    def _publish_vel(self, vx, vy, vz, wz):
        msg = Twist()
        msg.linear.x = float(vx); msg.linear.y = float(vy); msg.linear.z = float(vz)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def _publish_status(self):
        if self.pos_xyz is None:
            return
        s = String()
        cur = f"{self.pos_xyz[0]:.2f},{self.pos_xyz[1]:.2f},{self.pos_xyz[2]:.2f}"
        if self.state == "mission" and self.wp_i < len(self.waypoints):
            tx, ty, tz = self.waypoints[self.wp_i]
            tgt = f"{tx:.2f},{ty:.2f},{tz:.2f}"
        else:
            tgt = "-"
        s.data = f"state={self.state} wp={self.wp_i}/{len(self.waypoints)} pos=({cur}) tgt=({tgt}) yaw={self.yaw:.2f}"
        self.status_pub.publish(s)

    def _publish_path_and_markers(self):
        now = self.get_clock().now().to_msg()
        path = Path(); path.header.frame_id = self.world_frame; path.header.stamp = now
        for (x, y, z) in self.waypoints:
            ps = PoseStamped(); ps.header.frame_id = self.world_frame; ps.header.stamp = now
            ps.pose.position.x = float(x); ps.pose.position.y = float(y); ps.pose.position.z = float(z)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

        marr = MarkerArray()
        ls = Marker()
        ls.header.frame_id = self.world_frame; ls.header.stamp = now
        ls.ns = "path"; ls.id = 0; ls.type = Marker.LINE_STRIP; ls.action = Marker.ADD
        ls.scale.x = 0.03; ls.color.r = 1.0; ls.color.g = 0.2; ls.color.b = 0.2; ls.color.a = 1.0
        for (x, y, z) in self.waypoints:
            pt = Point(); pt.x, pt.y, pt.z = x, y, z
            ls.points.append(pt)
        marr.markers.append(ls)

        for i, (x, y, z) in enumerate(self.waypoints, start=1):
            m = Marker()
            m.header.frame_id = self.world_frame; m.header.stamp = now
            m.ns = "wps"; m.id = i; m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = z
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.12
            m.color.r = 0.2; m.color.g = 0.8; m.color.b = 0.2; m.color.a = 1.0
            marr.markers.append(m)
        self.marker_pub.publish(marr)


def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted.")
    finally:
        # Try to stop motion and shut down cleanly; guard double-shutdown.
        try:
            node._publish_vel(0.0, 0.0, 0.0, 0.0)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()


