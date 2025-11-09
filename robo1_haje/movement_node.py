#!/usr/bin/env python3
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




# -----------Alternate unintegrated version--------- #
# import math
# import random
# import heapq
# from typing import List, Optional, Tuple

# import rclpy
# from rclpy.node import Node
# from rcl_interfaces.msg import ParameterDescriptor, ParameterType
# from rclpy.qos import (
#     QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
# )

# from std_msgs.msg import Empty, String, Int32
# from geometry_msgs.msg import Twist, PoseStamped, Point
# from nav_msgs.msg import Odometry, Path
# from visualization_msgs.msg import Marker, MarkerArray


# # ----------------------------- helpers ---------------------------------

# def quat_to_yaw(qx, qy, qz, qw) -> float:
#     s = 2.0 * (qw * qz + qx * qy)
#     c = 1.0 - 2.0 * (qy * qy + qz * qz)
#     return math.atan2(s, c)

# def clamp(v: float, vmin: float, vmax: float) -> float:
#     return max(min(v, vmax), vmin)

# def angle_wrap(a: float) -> float:
#     while a > math.pi:
#         a -= 2.0 * math.pi
#     while a < -math.pi:
#         a += 2.0 * math.pi
#     return a

# def dist_xy(a: Tuple[float, float], b: Tuple[float, float]) -> float:
#     return math.hypot(a[0]-b[0], a[1]-b[1])


# class MovementNode(Node):
#     # --------------------------- init -----------------------------------
#     def __init__(self):
#         super().__init__("movement_controller")

#         # I/O topics
#         self.declare_parameter("cmd_vel_topic", "/cmd_vel")
#         self.declare_parameter("odom_topic", "/odometry")
#         self.declare_parameter("world_frame", "map")

#         # Behaviour toggles
#         self.declare_parameter("skip_takeoff", False)     
#         self.declare_parameter("use_z_control", False)    
#         self.declare_parameter("yaw_to_path", True)       
#         self.declare_parameter("lookahead_m", 0.0)        

#         # Altitudes
#         self.declare_parameter("launch_altitude", 2.0)
#         self.declare_parameter("land_touchdown_z", 0.1)

#         # Gains & limits
#         self.declare_parameter("kp_xy", 0.6)
#         self.declare_parameter("kp_z", 0.8)
#         self.declare_parameter("kp_yaw", 1.2)
#         self.declare_parameter("max_xy", 0.5)
#         self.declare_parameter("max_z", 0.5)
#         self.declare_parameter("max_yaw_rate", 0.8)

#         # Tolerances / timing
#         self.declare_parameter("xy_tol", 0.35)
#         self.declare_parameter("z_tol", 0.25)
#         self.declare_parameter("wp_hold_sec", 2.0)
#         self.declare_parameter("tick_hz", 20.0)

#         # Waypoints (typed)
#         self.declare_parameter(
#             "waypoints",
#             [0.0, 0.0, 2.0,  2.0, 0.0, 2.0,  2.0, 2.0, 2.0,  0.0, 2.0, 2.0],
#             ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY),
#         )

#         # Geofence (typed) — length != 6 disables
#         self.declare_parameter(
#             "geofence", [0.0],
#             ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY),
#         )

#         # STOP semantics
#         self.declare_parameter("stop_resets_index", True)

#         # Obstacle perception
#         self.declare_parameter("obstacles_enabled", False)    # off by default
#         self.declare_parameter("obstacles_topic", "/tree_pose")
#         self.declare_parameter("obstacle_radius_m", 0.6)      # fallback 
#         self.declare_parameter("clearance_m", 0.8)            # min clearance around obstacles for planning
#         self.declare_parameter("grid_res_m", 0.5)             # grid cell size for A*
#         self.declare_parameter("replan_ahead_radius", 1.5)    # trigger replan if obstacle within R ahead

#         # TSP + planning
#         self.declare_parameter("tsp_mode", "none")            # "none" | "nn_2opt"
#         self.declare_parameter("rth_on_breach", True)         # enforce RTH on geofence breach

#         # ----- resolve params
#         self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
#         self.odom_topic    = self.get_parameter("odom_topic").value
#         self.world_frame   = self.get_parameter("world_frame").value

#         self.skip_takeoff  = bool(self.get_parameter("skip_takeoff").value)
#         self.use_z         = bool(self.get_parameter("use_z_control").value)
#         self.yaw_to_path   = bool(self.get_parameter("yaw_to_path").value)
#         self.lookahead_m   = float(self.get_parameter("lookahead_m").value)

#         self.launch_alt    = float(self.get_parameter("launch_altitude").value)
#         self.land_touchdown_z = float(self.get_parameter("land_touchdown_z").value)

#         self.kp_xy = float(self.get_parameter("kp_xy").value)
#         self.kp_z  = float(self.get_parameter("kp_z").value)
#         self.kp_yaw= float(self.get_parameter("kp_yaw").value)
#         self.vmax_xy = float(self.get_parameter("max_xy").value)
#         self.vmax_z  = float(self.get_parameter("max_z").value)
#         self.vmax_yaw = float(self.get_parameter("max_yaw_rate").value)

#         self.xy_tol = float(self.get_parameter("xy_tol").value)
#         self.z_tol  = float(self.get_parameter("z_tol").value)
#         self.wp_hold_sec = float(self.get_parameter("wp_hold_sec").value)
#         tick_hz = float(self.get_parameter("tick_hz").value)
#         self.dt = 1.0 / max(1e-6, tick_hz)

#         wpa = list(self.get_parameter("waypoints").get_parameter_value().double_array_value)
#         self.waypoints: List[Tuple[float, float, float]] = [
#             (float(wpa[i]), float(wpa[i+1]), float(wpa[i+2]))
#             for i in range(0, len(wpa), 3)
#         ]

#         gf = list(self.get_parameter("geofence").get_parameter_value().double_array_value)
#         self.geofence = (gf[0], gf[1], gf[2], gf[3], gf[4], gf[5]) if len(gf) == 6 else None

#         self.stop_resets_index = bool(self.get_parameter("stop_resets_index").value)

#         self.obstacles_enabled   = bool(self.get_parameter("obstacles_enabled").value)
#         self.obstacles_topic     = self.get_parameter("obstacles_topic").value
#         self.obstacle_radius_m   = float(self.get_parameter("obstacle_radius_m").value)
#         self.clearance_m         = float(self.get_parameter("clearance_m").value)
#         self.grid_res            = float(self.get_parameter("grid_res_m").value)
#         self.replan_ahead_radius = float(self.get_parameter("replan_ahead_radius").value)
#         self.tsp_mode            = self.get_parameter("tsp_mode").value
#         self.rth_on_breach       = bool(self.get_parameter("rth_on_breach").value)

#         # ----- publishers (latched for GUI)
#         qos_tl = QoSProfile(depth=1)
#         qos_tl.reliability = QoSReliabilityPolicy.RELIABLE
#         qos_tl.history = QoSHistoryPolicy.KEEP_LAST
#         qos_tl.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

#         self.cmd_pub    = self.create_publisher(Twist, self.cmd_vel_topic, 10)
#         self.path_pub   = self.create_publisher(Path, "movement/path", qos_tl)
#         self.marker_pub = self.create_publisher(MarkerArray, "movement/markers", qos_tl)
#         self.status_pub = self.create_publisher(String, "movement/status", 10)

#         # ----- subscriptions
#         self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 20)

#         self.create_subscription(Empty, "movement/go",       self._go_cb,       10)
#         self.create_subscription(Empty, "movement/stop",     self._stop_cb,     10)
#         self.create_subscription(Empty, "movement/pause",    self._pause_cb,    10)
#         self.create_subscription(Empty, "movement/continue", self._continue_cb, 10)
#         self.create_subscription(Empty, "movement/launch",   self._launch_cb,   10)
#         self.create_subscription(Empty, "movement/land",     self._land_cb,     10)
#         self.create_subscription(Point, "movement/waypoint", self._add_wp_cb,   10)
#         self.create_subscription(Empty, "movement/skip",     self._skip_cb,     10)
#         self.create_subscription(Empty, "movement/clear",    self._clear_cb,    10)
#         self.create_subscription(Int32, "movement/seed_random", self._seed_random_cb, 10)

#         if self.obstacles_enabled and self.obstacles_topic:
#             self.create_subscription(MarkerArray, self.obstacles_topic, self._obstacles_cb, 10)

#         # ----- state
#         self.pos_xyz: Optional[Tuple[float, float, float]] = None
#         self.yaw: float = 0.0
#         self.home_xy: Optional[Tuple[float, float]] = None

#         self.state: str = "grounded"     # grounded | takeoff | await_go | mission | paused | landing | rth
#         self.prev_active_state: str = "mission"
#         self.wp_i: int = 0
#         self.hold_t: float = 0.0
#         self.go_pending_after_takeoff: bool = False

#         # current global plan (sequence of XY points + indices that correspond to true WPs)
#         self.plan_xy: List[Tuple[float, float]] = []
#         self.plan_wp_index_breaks: List[int] = []  # indices in plan_xy where a true WP lies (for marker indices)
#         self.plan_i: int = 0
#         self.last_plan_stamp: float = 0.0

#         # obstacle list [(x,y,radius)] and stamp
#         self.obstacles: List[Tuple[float, float, float]] = []
#         self.obstacles_stamp: float = 0.0

#         # timers
#         self.timer = self.create_timer(self.dt, self._tick)
#         self.status_timer = self.create_timer(0.5, self._publish_status)

#         # initial visualization
#         self._publish_path_markers()

#         self.get_logger().info(
#             f"MovementNode up. cmd_vel={self.cmd_vel_topic}, odom={self.odom_topic}, "
#             f"skip_takeoff={self.skip_takeoff}, z_ctrl={self.use_z}, yaw_to_path={self.yaw_to_path}, "
#             f"lookahead={self.lookahead_m}m, geofence={'on' if self.geofence else 'off'}, "
#             f"tsp={self.tsp_mode}, obstacles={'on' if self.obstacles_enabled else 'off'}."
#         )
#         self.get_logger().info("Waiting for TAKEOFF. "
#                                "Use:  ros2 topic pub -1 /movement/launch std_msgs/msg/Empty \"{}\"")

#     # --------------------------- callbacks -------------------------------#

#     def _odom_cb(self, msg: Odometry):
#         p = msg.pose.pose.position
#         q = msg.pose.pose.orientation
#         self.pos_xyz = (p.x, p.y, p.z)
#         self.yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
#         # set home on first odom after launch command
#         if self.state == "takeoff" and self.home_xy is None:
#             self.home_xy = (p.x, p.y)

#     def _go_cb(self, _):
#         if self.state == "paused":
#             self._continue_cb(None); return
#         if self.state == "await_go":
#             self._plan_if_needed()
#             self.state = "mission"
#             self.get_logger().info("GO -> MISSION")
#             return
#         if self.state == "takeoff":
#             self.go_pending_after_takeoff = True
#             self.get_logger().info("GO noted; will enter MISSION at altitude.")
#             return
#         if self.state == "grounded":
#             if self.skip_takeoff:
#                 if self.pos_xyz is None:
#                     self.get_logger().warn("GO ignored: no odometry yet."); return
#                 self._plan_if_needed()
#                 self.get_logger().info("GO from ground (skip_takeoff=True) -> MISSION")
#                 self.state = "mission"
#             else:
#                 self.get_logger().warn("GO ignored: LAUNCH first.")
#             return
#         self.get_logger().warn("GO ignored: already active.")

#     def _stop_cb(self, _):
#         # emergency stop: either RTH or straight land depending on config
#         if self.rth_on_breach and self.home_xy is not None:
#             self.state = "rth"
#             self._plan_rth()
#             self.get_logger().info("STOP -> RTH (pre-emption).")
#         else:
#             self.state = "landing"
#             self.get_logger().info("STOP -> LANDING.")
#         self.go_pending_after_takeoff = False
#         if self.stop_resets_index:
#             self.wp_i = 0
#         self.hold_t = 0.0
#         self._publish_vel(0.0, 0.0, 0.0, 0.0)

#     def _pause_cb(self, _):
#         if self.state in ("mission", "takeoff", "await_go"):
#             self.prev_active_state = self.state
#             self.state = "paused"
#             self._publish_vel(0.0, 0.0, 0.0, 0.0)
#             self.get_logger().info("PAUSE")
#         else:
#             self.get_logger().warn("PAUSE ignored (not active).")

#     def _continue_cb(self, _):
#         if self.state == "paused":
#             self.state = self.prev_active_state
#             self.get_logger().info(f"CONTINUE -> {self.state.upper()}")
#         elif self.state == "grounded":
#             self._launch_cb(None)
#         else:
#             self.get_logger().warn("CONTINUE ignored.")

#     def _launch_cb(self, _):
#         if self.pos_xyz is None:
#             self.get_logger().warn("LAUNCH ignored: no odometry yet."); return
#         self.state = "takeoff"
#         self.hold_t = 0.0
#         # record home on first odometry tick in takeoff
#         self.get_logger().info("LAUNCH -> TAKEOFF")

#     def _land_cb(self, _):
#         if self.pos_xyz is None:
#             self.get_logger().warn("LAND ignored: no odometry yet."); return
#         self.state = "landing"
#         self.hold_t = 0.0
#         self.get_logger().info("LAND -> LANDING")

#     def _add_wp_cb(self, pt: Point):
#         self.waypoints.append((float(pt.x), float(pt.y), float(pt.z)))
#         self.get_logger().info(f"Added waypoint ({pt.x:.2f},{pt.y:.2f},{pt.z:.2f}); total={len(self.waypoints)}")
#         self._publish_path_markers()
#         if self.state in ("await_go", "mission"):
#             self._plan_if_needed()

#     def _skip_cb(self, _):
#         if self.wp_i < len(self.waypoints):
#             self.wp_i += 1
#             self.hold_t = 0.0
#             self.get_logger().info(f"Skipped to waypoint index {self.wp_i}")
#             self._plan_if_needed()
#         else:
#             self.get_logger().info("Skip ignored: already past last waypoint.")

#     def _clear_cb(self, _):
#         self.waypoints.clear()
#         self.wp_i = 0
#         self.hold_t = 0.0
#         self.plan_xy.clear()
#         self.plan_wp_index_breaks.clear()
#         self.plan_i = 0
#         self._publish_path_markers()
#         self.get_logger().info("Cleared all waypoints.")

#     def _seed_random_cb(self, msg: Int32):
#         """Seed N random waypoints inside geofence for TSP demos."""
#         n = int(msg.data)
#         if not self.geofence:
#             self.get_logger().warn("seed_random ignored: requires geofence."); return
#         xmin, xmax, ymin, ymax, zmin, zmax = self.geofence
#         for _ in range(max(0, n)):
#             x = random.uniform(xmin, xmax)
#             y = random.uniform(ymin, ymax)
#             z = min(max(self.launch_alt, zmin), zmax) if self.use_z else self.launch_alt
#             self.waypoints.append((x, y, z))
#         self.get_logger().info(f"Seeded {n} random WPs; total={len(self.waypoints)}")
#         self._publish_path_markers()
#         if self.state in ("await_go", "mission"):
#             self._plan_if_needed()

#     def _obstacles_cb(self, msg: MarkerArray):
#         """Ingest obstacles from perception as MarkerArray (/tree_pose)."""
#         obs: List[Tuple[float, float, float]] = []
#         frame_id = ""
#         stamp_ns = 0
#         for i, m in enumerate(msg.markers):
#             frame_id = m.header.frame_id or frame_id
#             stamp_ns = max(stamp_ns, int(m.header.stamp.sec)*10**9 + int(m.header.stamp.nanosec))
#             x = m.pose.position.x
#             y = m.pose.position.y
#             # Use sphere/scale.x as radius if available, else fallback param
#             r = float(m.scale.x) * 0.5 if m.scale.x > 0.0 else self.obstacle_radius_m
#             obs.append((x, y, max(0.1, r)))
#         self.obstacles = obs
#         self.obstacles_stamp = float(stamp_ns) * 1e-9
#         self.get_logger().info(f"Perception: {len(obs)} obstacles (frame='{frame_id}', t={self.obstacles_stamp:.2f}s)")
#         if self.state == "mission" and self._segment_blocked_ahead():
#             self.get_logger().info("Obstacle ahead -> REPLAN")
#             self._plan_if_needed(force=True)

#     # --------------------------- control loop ----------------------------

#     def _tick(self):
#         if self.pos_xyz is None:
#             return

#         x, y, z = self.pos_xyz

#         # geofence enforcement
#         if self.geofence and self.state in ("mission", "await_go", "takeoff") and self.rth_on_breach:
#             if not self._inside_geofence(x, y, z):
#                 self.get_logger().warn("Geofence breach -> RTH")
#                 self.state = "rth"
#                 self._plan_rth()

#         if self.state == "grounded":
#             return

#         if self.state == "takeoff":
#             target_z = self.launch_alt
#             dz = target_z - z
#             if abs(dz) < self.z_tol:
#                 if self.hold_t <= 0.0:
#                     self.hold_t = self.wp_hold_sec
#                     self.get_logger().info("Reached launch altitude; holding...")
#                 else:
#                     self.hold_t -= self.dt
#                     if self.hold_t <= 0.0:
#                         if self.go_pending_after_takeoff:
#                             self._plan_if_needed()
#                             self.state = "mission"
#                             self.go_pending_after_takeoff = False
#                             self.get_logger().info("Takeoff complete -> MISSION (GO was pending).")
#                         else:
#                             self.state = "await_go"
#                             self.get_logger().info("Takeoff complete -> AWAIT_GO.")
#                 self._publish_vel(0.0, 0.0, 0.0, 0.0)
#             else:
#                 vz = clamp(self.kp_z * dz, -self.vmax_z, self.vmax_z)
#                 self._publish_vel(0.0, 0.0, vz, 0.0)
#             return

#         if self.state == "await_go":
#             self._publish_vel(0.0, 0.0, 0.0, 0.0)
#             return

#         if self.state == "rth":
#             # Follow plan to home, then LANDING
#             if not self.plan_xy:
#                 self._publish_vel(0.0, 0.0, 0.0, 0.0)
#                 self.state = "landing"
#                 return
#             self._track_plan_step(x, y, z, follow_z=False)
#             return

#         if self.state == "mission":
#             if self.wp_i >= len(self.waypoints):
#                 self.get_logger().info("All waypoints done -> LANDING")
#                 self.state = "landing"
#                 return
#             # Ensure we have a plan
#             if not self.plan_xy:
#                 self._plan_if_needed(force=True)
#                 if not self.plan_xy:
#                     # no plan, hover
#                     self._publish_vel(0.0, 0.0, 0.0, 0.0)
#                     return
#             self._track_plan_step(x, y, z, follow_z=self.use_z)
#             return

#         if self.state == "paused":
#             self._publish_vel(0.0, 0.0, 0.0, 0.0)
#             return

#         if self.state == "landing":
#             target_z = self.land_touchdown_z
#             dz = target_z - z
#             if abs(dz) < self.z_tol:
#                 self._publish_vel(0.0, 0.0, 0.0, 0.0)
#                 self.state = "grounded"
#                 self.get_logger().info("Touchdown -> GROUNDED.")
#                 return
#             vz = clamp(self.kp_z * dz, -self.vmax_z, self.vmax_z)
#             self._publish_vel(0.0, 0.0, vz, 0.0)

#     # tracking & planning 

#     def _track_plan_step(self, x: float, y: float, z: float, follow_z: bool):
#         """Drive toward current plan point, optionally controlling Z."""
#         if self.plan_i >= len(self.plan_xy):
#             self._publish_vel(0.0, 0.0, 0.0, 0.0); return

#         tx, ty = self.plan_xy[self.plan_i]
#         # waypoint-level hold when we hit a true waypoint index
#         at_wp = self.plan_i in self.plan_wp_index_breaks

#         # Z command
#         if follow_z and self.wp_i < len(self.waypoints):
#             tz = self.waypoints[self.wp_i][2]
#             vz = clamp(self.kp_z * (tz - z), -self.vmax_z, self.vmax_z)
#         else:
#             vz = 0.0

#         # XY command
#         dx, dy = (tx - x), (ty - y)
#         dist = math.hypot(dx, dy)

#         if dist > self.xy_tol:
#             # optional lookahead
#             ex, ey = dx, dy
#             if self.lookahead_m > 0.0 and dist > 1e-6:
#                 L = min(self.lookahead_m, dist)
#                 ux, uy = dx / dist, dy / dist
#                 gx, gy = x + ux * L, y + uy * L
#                 ex, ey = (gx - x), (gy - y)

#             vx = clamp(self.kp_xy * ex, -self.vmax_xy, self.vmax_xy)
#             vy = clamp(self.kp_xy * ey, -self.vmax_xy, self.vmax_xy)

#             if self.yaw_to_path and (abs(vx) + abs(vy)) > 1e-6:
#                 hdg = math.atan2(vy, vx)
#                 yaw_err = angle_wrap(hdg - self.yaw)
#                 wz = clamp(self.kp_yaw * yaw_err, -self.vmax_yaw, self.vmax_yaw)
#             else:
#                 wz = 0.0

#             self._publish_vel(vx, vy, vz, wz)
#         else:
#             # Arrived at this plan step; advance or hold at true waypoint
#             self.plan_i += 1
#             if at_wp:
#                 self._publish_vel(0.0, 0.0, 0.0, 0.0)
#                 if self.hold_t <= 0.0:
#                     self.hold_t = self.wp_hold_sec
#                     # Determine which WP index we just reached:
#                     reached_idx = self.wp_i + 1
#                     self.get_logger().info(f"Reached waypoint {reached_idx}/{len(self.waypoints)}; holding...")
#                 else:
#                     self.hold_t -= self.dt
#                     if self.hold_t <= 0.0:
#                         self.wp_i += 1
#                         self.get_logger().info("Proceeding to next waypoint...")
#                         # If plan exhausted for remaining WPs, replan from here
#                         if self.plan_i >= len(self.plan_xy):
#                             self._plan_if_needed(force=True)

#     def _inside_geofence(self, x: float, y: float, z: float) -> bool:
#         if not self.geofence:
#             return True
#         xmin, xmax, ymin, ymax, zmin, zmax = self.geofence
#         return (xmin <= x <= xmax) and (ymin <= y <= ymax) and (zmin <= z <= zmax)

#     def _segment_blocked_ahead(self) -> bool:
#         """Is there an obstacle within replan_ahead_radius of current plan segment point?"""
#         if not self.plan_xy or not self.obstacles:
#             return False
#         i = max(0, self.plan_i)
#         j = min(len(self.plan_xy)-1, i+5)  # short lookahead along polyline
#         if i >= j: return False
#         a = self.plan_xy[i]; b = self.plan_xy[j]
#         ax, ay = a; bx, by = b
#         abx, aby = (bx-ax, by-ay)
#         ab2 = abx*abx + aby*aby + 1e-9
#         for (ox, oy, r) in self.obstacles:
#             # distance from obstacle to segment
#             proj = ((ox-ax)*abx + (oy-ay)*aby) / ab2
#             proj = clamp(proj, 0.0, 1.0)
#             cx, cy = ax + proj*abx, ay + proj*aby
#             d = math.hypot(ox-cx, oy-cy)
#             if d <= (r + self.clearance_m + self.replan_ahead_radius):
#                 return True
#         return False

#     def _plan_rth(self):
#         """Plan from current XY to home XY; keep altitude, then LAND."""
#         if self.pos_xyz is None or self.home_xy is None:
#             self.plan_xy = []; self.plan_wp_index_breaks = []; self.plan_i = 0; return
#         sx, sy, _ = self.pos_xyz
#         gx, gy = self.home_xy
#         self.plan_xy = self._astar_xy((sx, sy), (gx, gy))
#         self.plan_wp_index_breaks = [len(self.plan_xy)-1] if self.plan_xy else []
#         self.plan_i = 0
#         self._publish_path_markers()

#     def _plan_if_needed(self, force: bool = False):
#         """Create a global plan: optional TSP order + A* between consecutive WPs."""
#         if self.pos_xyz is None or self.wp_i >= len(self.waypoints):
#             self.plan_xy = []; self.plan_wp_index_breaks = []; self.plan_i = 0; return
#         if not force and (self.get_clock().now().nanoseconds * 1e-9 - self.last_plan_stamp) < 0.25:
#             return  # debounce replans

#         # seed start from current pos
#         sx, sy, _ = self.pos_xyz

#         # target set = remaining waypoints
#         targets = self.waypoints[self.wp_i:]

#         # order by TSP if enabled
#         order = list(range(len(targets)))
#         if self.tsp_mode == "nn_2opt" and len(targets) >= 3:
#             order = self._tsp_nn_2opt((sx, sy), [(x, y) for (x, y, _) in targets])

#         # stitch with A* between each pair
#         plan: List[Tuple[float, float]] = []
#         wp_breaks: List[int] = []
#         cur = (sx, sy)
#         for k, idx in enumerate(order):
#             tx, ty, _tz = targets[idx]
#             seg = self._astar_xy(cur, (tx, ty))
#             if not seg:
#                 # fallback straight line if planner fails (very rare)
#                 seg = [cur, (tx, ty)]
#             if plan and seg and seg[0] == plan[-1]:
#                 plan.extend(seg[1:])
#             else:
#                 plan.extend(seg)
#             wp_breaks.append(len(plan)-1)  # index of this true WP in the polyline
#             cur = (tx, ty)

#         self.plan_xy = plan
#         self.plan_wp_index_breaks = wp_breaks
#         self.plan_i = 0
#         self.last_plan_stamp = self.get_clock().now().nanoseconds * 1e-9
#         self._publish_path_markers()

#     # --------------------------- TSP (NN + 2-opt) -----------------------

#     def _tsp_nn_2opt(self, start_xy: Tuple[float, float], pts: List[Tuple[float, float]]) -> List[int]:
#         n = len(pts)
#         if n <= 1:
#             return list(range(n))
#         # nearest neighbor
#         unvisited = set(range(n))
#         cur = start_xy
#         route: List[int] = []
#         while unvisited:
#             nxt = min(unvisited, key=lambda i: dist_xy(cur, pts[i]))
#             route.append(nxt)
#             unvisited.remove(nxt)
#             cur = pts[nxt]
#         # 2-opt improvement
#         improved = True
#         while improved:
#             improved = False
#             for i in range(n-2):
#                 for j in range(i+2, n):
#                     a, b = route[i], route[i+1]
#                     c, d = route[j], route[(j+1) % n] if j+1 < n else None
#                     if d is None:  # don't close tour; it's a path, not cycle
#                         before = dist_xy(pts[a], pts[b]) + (0 if j+1 >= n else dist_xy(pts[c], pts[d]))
#                         after  = dist_xy(pts[a], pts[c]) + (0 if j+1 >= n else dist_xy(pts[b], pts[d]))
#                     else:
#                         before = dist_xy(pts[a], pts[b]) + dist_xy(pts[c], pts[d])
#                         after  = dist_xy(pts[a], pts[c]) + dist_xy(pts[b], pts[d])
#                     if after + 1e-6 < before:
#                         route[i+1:j+1] = reversed(route[i+1:j+1])
#                         improved = True
#         return route

#     # A* grid routing 

#     def _astar_xy(self, start: Tuple[float, float], goal: Tuple[float, float]) -> List[Tuple[float, float]]:
#         # build grid from geofence
#         if not self.geofence:
#             return [start, goal]
#         xmin, xmax, ymin, ymax, _zmin, _zmax = self.geofence
#         res = max(0.1, self.grid_res)

#         def to_cell(p):
#             return (int((p[0]-xmin)/res), int((p[1]-ymin)/res))
#         def to_xy(c):
#             return (xmin + (c[0]+0.5)*res, ymin + (c[1]+0.5)*res)

#         sx, sy = to_cell(start)
#         gx, gy = to_cell(goal)
#         width  = max(2, int((xmax-xmin)/res))
#         height = max(2, int((ymax-ymin)/res))
#         occ = [[False]*height for _ in range(width)]
#         inflate = self.clearance_m
#         for (ox, oy, r) in self.obstacles:
#             rr = r + inflate
#             # bounding box in cells
#             cx, cy = to_cell((ox, oy))
#             rad_c = int(rr / res) + 1
#             for ix in range(max(0, cx-rad_c), min(width, cx+rad_c+1)):
#                 for iy in range(max(0, cy-rad_c), min(height, cy+rad_c+1)):
#                     px, py = to_xy((ix, iy))
#                     if math.hypot(px-ox, py-oy) <= rr:
#                         occ[ix][iy] = True

#         # keep borders free unless outside geofence
#         def in_bounds(c):
#             return 0 <= c[0] < width and 0 <= c[1] < height
#         def passable(c):
#             return not occ[c[0]][c[1]]

#         if not in_bounds((sx, sy)) or not in_bounds((gx, gy)):
#             return [start, goal]

#         # A* with 8-neighborhood
#         g_score = { (sx, sy): 0.0 }
#         came = {}
#         h = lambda a: math.hypot(a[0]-gx, a[1]-gy)
#         openq = [(h((sx,sy)), (sx,sy))]
#         visited = set()
#         nbrs = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
#         w = { (dx,dy): (math.sqrt(2.0) if dx!=0 and dy!=0 else 1.0) for (dx,dy) in nbrs }

#         while openq:
#             _, cur = heapq.heappop(openq)
#             if cur in visited: continue
#             visited.add(cur)
#             if cur == (gx, gy):
#                 # reconstruct
#                 path_cells = [cur]
#                 while cur in came:
#                     cur = came[cur]
#                     path_cells.append(cur)
#                 path_cells.reverse()
#                 # densify to XY
#                 return [to_xy(c) for c in path_cells]
#             for d in nbrs:
#                 nxt = (cur[0]+d[0], cur[1]+d[1])
#                 if not in_bounds(nxt) or not passable(nxt):
#                     continue
#                 tentative = g_score[cur] + w[d]
#                 if tentative < g_score.get(nxt, 1e18):
#                     g_score[nxt] = tentative
#                     came[nxt] = cur
#                     f = tentative + h(nxt)
#                     heapq.heappush(openq, (f, nxt))

#         # no path
#         return [start, goal]

#     # --------------------------- visualization & status -----------------

#     def _publish_status(self):
#         if self.pos_xyz is None:
#             return
#         s = String()
#         cur = f"{self.pos_xyz[0]:.2f},{self.pos_xyz[1]:.2f},{self.pos_xyz[2]:.2f}"
#         tgt = "-"
#         if self.state == "mission" and self.wp_i < len(self.waypoints):
#             tx, ty, tz = self.waypoints[self.wp_i]
#             tgt = f"{tx:.2f},{ty:.2f},{tz:.2f}"
#         s.data = f"state={self.state} wp={self.wp_i}/{len(self.waypoints)} pos=({cur}) tgt=({tgt}) yaw={self.yaw:.2f}"
#         self.status_pub.publish(s)

#     def _publish_path_markers(self):
#         """Publish current plan as Path + numbered markers."""
#         now = self.get_clock().now().to_msg()

#         # nav_msgs/Path
#         path = Path()
#         path.header.frame_id = self.world_frame
#         path.header.stamp = now
#         for (x, y) in self.plan_xy:
#             ps = PoseStamped()
#             ps.header.frame_id = self.world_frame
#             ps.header.stamp = now
#             ps.pose.position.x = float(x)
#             ps.pose.position.y = float(y)
#             ps.pose.position.z = float(self.launch_alt)
#             ps.pose.orientation.w = 1.0
#             path.poses.append(ps)
#         self.path_pub.publish(path)

#         # MarkerArray: line + numbered spheres for true WPs
#         marr = MarkerArray()

#         ls = Marker()
#         ls.header.frame_id = self.world_frame; ls.header.stamp = now
#         ls.ns = "plan"; ls.id = 0; ls.type = Marker.LINE_STRIP; ls.action = Marker.ADD
#         ls.scale.x = 0.03; ls.color.r = 1.0; ls.color.g = 0.2; ls.color.b = 0.2; ls.color.a = 1.0
#         for (x, y) in self.plan_xy:
#             p = Point(); p.x = x; p.y = y; p.z = self.launch_alt
#             ls.points.append(p)
#         marr.markers.append(ls)

#         # spheres + numbers at the “true” waypoints along the polyline
#         for k, idx in enumerate(self.plan_wp_index_breaks, start=self.wp_i+1):
#             if 0 <= idx < len(self.plan_xy):
#                 x, y = self.plan_xy[idx]
#                 m = Marker()
#                 m.header.frame_id = self.world_frame; m.header.stamp = now
#                 m.ns = "wps"; m.id = 1000 + k; m.type = Marker.SPHERE; m.action = Marker.ADD
#                 m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = self.launch_alt
#                 m.pose.orientation.w = 1.0
#                 m.scale.x = m.scale.y = m.scale.z = 0.12
#                 m.color.r = 0.2; m.color.g = 0.8; m.color.b = 0.2; m.color.a = 1.0
#                 marr.markers.append(m)

#                 t = Marker()
#                 t.header.frame_id = self.world_frame; t.header.stamp = now
#                 t.ns = "wp_idx"; t.id = 2000 + k; t.type = Marker.TEXT_VIEW_FACING; t.action = Marker.ADD
#                 t.pose.position.x = x; t.pose.position.y = y; t.pose.position.z = self.launch_alt + 0.2
#                 t.scale.z = 0.2
#                 t.color.r = t.color.g = t.color.b = 1.0; t.color.a = 1.0
#                 t.text = str(k)
#                 marr.markers.append(t)

#         self.marker_pub.publish(marr)

#     def _publish_vel(self, vx, vy, vz, wz):
#         msg = Twist()
#         msg.linear.x = float(vx); msg.linear.y = float(vy); msg.linear.z = float(vz)
#         msg.angular.z = float(wz)
#         self.cmd_pub.publish(msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = MovementNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Interrupted.")
#     finally:
#         try: node._publish_vel(0.0, 0.0, 0.0, 0.0)
#         except Exception: pass
#         try: node.destroy_node()
#         except Exception: pass
#         try:
#             if rclpy.ok():
#                 rclpy.shutdown()
#         except Exception:
#             pass


# if __name__ == "__main__":
#     main()

