
#!/usr/bin/env python3
# HAJE Movement Subsystem (ROS 2 Humble + Ignition/Gazebo)
#
# PASS:
# - /movement/go starts mission; /movement/stop stops.
# - Executes ≥3 waypoints with tolerances + per-WP hold.
# - Publishes geometry_msgs/Twist to drive the drone.
# - Publishes nav_msgs/Path (/movement/path).
#
# CREDIT hooks:
# - goal_source: 'param' (default) or 'gui' (/gui/waypoints: nav_msgs/Path)
# - optimise_path: placeholder reordering hook
#
# Robustness:
# - frame_mode: 'world' or 'body' for /cmd_vel interpretation
# - odom staleness check; optional geofence stop
# - low-rate debug logs of pose/target/dist

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path

def yaw_from_quat(q):
    # 2D yaw from quaternion (x,y,z,w)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)

class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_controller')

        # ---------- Parameters (override with --ros-args -p ...) ----------
        self.declare_parameter('drone_name', 'parrot')
        name = self.get_parameter('drone_name').get_parameter_value().string_value

        # Topics/frames (defaults match your sim)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('base_frame', f'{name}::base_link')

        # Modes / features
        self.declare_parameter('skip_takeoff', True)       # start in mission
        self.declare_parameter('use_z_control', False)     # ignore Z by default
        self.declare_parameter('goal_source', 'param')     # 'param' or 'gui'
        self.declare_parameter('optimise_path', False)
        self.declare_parameter('avoid_obstacles', False)   # placeholder
        self.declare_parameter('frame_mode', 'world')      # 'world' or 'body'
        self.declare_parameter('debug', True)              # low-rate logs
        self.declare_parameter('debug_hz', 1.0)            # Hz for debug
        self.declare_parameter('odom_timeout_s', 1.0)      # staleness guard
        self.declare_parameter('geofence_radius', 0.0)     # 0 = disabled

        # Controller & mission
        self.declare_parameter('kp_xy', 0.6)
        self.declare_parameter('kp_z', 0.8)
        self.declare_parameter('max_xy', 1.0)   # m/s
        self.declare_parameter('max_z', 0.7)    # m/s
        self.declare_parameter('xy_tol', 0.35)  # m
        self.declare_parameter('z_tol', 0.25)   # m
        self.declare_parameter('hold_sec', 2.0)
        self.declare_parameter('tick_hz', 20.0)

        # Default ≥3-waypoint loop
        self.declare_parameter('waypoints', [
            0.0, 0.0, 2.0,
            2.0, 0.0, 2.0,
            2.0, 2.0, 2.0,
            0.0, 2.0, 2.0
        ])

        # ---------- Resolve ----------
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.odom_topic    = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.world_frame   = self.get_parameter('world_frame').get_parameter_value().string_value
        self.base_frame    = self.get_parameter('base_frame').get_parameter_value().string_value

        self.skip_takeoff  = bool(self.get_parameter('skip_takeoff').value)
        self.use_z         = bool(self.get_parameter('use_z_control').value)
        self.goal_source   = str(self.get_parameter('goal_source').value)
        self.optimise_path = bool(self.get_parameter('optimise_path').value)
        self.avoid_obst    = bool(self.get_parameter('avoid_obstacles').value)
        self.frame_mode    = str(self.get_parameter('frame_mode').value).lower()
        self.debug         = bool(self.get_parameter('debug').value)
        self.debug_dt      = 1.0 / float(self.get_parameter('debug_hz').value)
        self.odom_timeout  = float(self.get_parameter('odom_timeout_s').value)
        self.geofence_r    = float(self.get_parameter('geofence_radius').value)

        self.kp_xy   = float(self.get_parameter('kp_xy').value)
        self.kp_z    = float(self.get_parameter('kp_z').value)
        self.vmax_xy = float(self.get_parameter('max_xy').value)
        self.vmax_z  = float(self.get_parameter('max_z').value)
        self.xy_tol  = float(self.get_parameter('xy_tol').value)
        self.z_tol   = float(self.get_parameter('z_tol').value)
        self.hold_s  = float(self.get_parameter('hold_sec').value)
        tick_hz      = float(self.get_parameter('tick_hz').value)
        self.dt      = 1.0 / tick_hz

        wp_list = self.get_parameter('waypoints').get_parameter_value().double_array_value
        if len(wp_list) % 3 != 0 or len(wp_list) < 9:
            self.get_logger().warn("Waypoints param invalid; using defaults.")
            self.waypoints = [(0.0, 0.0, 2.0), (2.0, 0.0, 2.0), (2.0, 2.0, 2.0), (0.0, 2.0, 2.0)]
        else:
            self.waypoints = [(wp_list[i], wp_list[i+1], wp_list[i+2]) for i in range(0, len(wp_list), 3)]

        # ---------- IO ----------
        self.cmd_pub  = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)

        self.create_subscription(Empty, 'movement/go',   self._go_cb,   10)
        self.create_subscription(Empty, 'movement/stop', self._stop_cb, 10)

        if self.goal_source.lower() == 'gui':
            self.gui_path_sub = self.create_subscription(Path, '/gui/waypoints', self._gui_path_cb, 10)

        self.path_pub = self.create_publisher(Path, 'movement/path', 10)
        self._publish_path()

        # ---------- Mission state ----------
        self.pos = None
        self.yaw = 0.0
        self.last_odom_time = None
        self.state = 'idle'     # 'idle' | 'takeoff' | 'mission' | 'landing'
        self.wp_i = 0
        self.hold_t = 0.0
        self.home_xy = None
        self._next_debug_t = 0.0

        self.timer = self.create_timer(self.dt, self._tick)
        self.get_logger().info(f"MovementNode ready. cmd_vel={self.cmd_vel_topic}, odom={self.odom_topic}. Waiting for GO.")

    # ---------------- Callbacks ----------------
    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pos = (p.x, p.y, p.z)
        q = msg.pose.pose.orientation
        self.yaw = yaw_from_quat(q)
        self.last_odom_time = self.get_clock().now()

    def _go_cb(self, _: Empty):
        if self.state != 'idle':
            self.get_logger().warn("GO received but mission already active.")
            return
        if self.pos is None:
            self.get_logger().warn("GO received but no odometry yet.")
            return
        self.wp_i = 0
        self.hold_t = 0.0
        self.state = 'mission' if self.skip_takeoff else 'takeoff'
        self.home_xy = (self.pos[0], self.pos[1])
        phase = 'MISSION (skip_takeoff)' if self.skip_takeoff else 'TAKEOFF'
        self.get_logger().info(f"GO: entering {phase}. frame_mode={self.frame_mode}")

    def _stop_cb(self, _: Empty):
        self.get_logger().info("STOP: zeroing velocity and resetting to idle.")
        self._publish_vel(0.0, 0.0, 0.0)
        self.state = 'idle'
        self.wp_i = 0
        self.hold_t = 0.0

    # ---------------- Control loop ----------------
    def _tick(self):
        now = self.get_clock().now()
        if self.state == 'idle' or self.pos is None:
            return

        # Odom staleness guard
        if self.last_odom_time is None or (now - self.last_odom_time).nanoseconds * 1e-9 > self.odom_timeout:
            self._publish_vel(0.0, 0.0, 0.0)
            self.get_logger().warn_once("Odometry stale; holding.")
            return

        x, y, z = self.pos

        # Optional geofence around home
        if self.geofence_r > 0.0 and self.home_xy is not None:
            dxh = x - self.home_xy[0]
            dyh = y - self.home_xy[1]
            if math.hypot(dxh, dyh) > self.geofence_r:
                self.get_logger().warn("Geofence breached; STOP.")
                self._stop_cb(Empty())
                return

        # Debug log (low rate)
        if self.debug and now.nanoseconds * 1e-9 >= self._next_debug_t:
            if self.state in ('mission', 'takeoff'):
                tx, ty, tz = self.waypoints[min(self.wp_i, len(self.waypoints)-1)]
                self.get_logger().info(
                    f"state={self.state} wp={self.wp_i+1}/{len(self.waypoints)} "
                    f"pos=({x:.2f},{y:.2f},{z:.2f}) tgt=({tx:.2f},{ty:.2f},{tz:.2f}) yaw={self.yaw:.2f}rad"
                )
            self._next_debug_t = now.nanoseconds * 1e-9 + self.debug_dt

        if self.state == 'takeoff':
            target_z = self.waypoints[0][2]
            vz = self._clip(self.kp_z * (target_z - z), self.vmax_z)
            vx, vy = 0.0, 0.0
            if abs(target_z - z) < self.z_tol:
                if self.hold_t <= 0.0:
                    self.hold_t = self.hold_s
                    self.get_logger().info("Reached takeoff altitude; holding...")
                else:
                    self.hold_t -= self.dt
                    if self.hold_t <= 0.0:
                        self.state = 'mission'
                        self.get_logger().info("Starting waypoint mission...")
                        vx = vy = vz = 0.0
            self._publish_vel(vx, vy, vz)
            return

        if self.state == 'mission':
            if self.wp_i >= len(self.waypoints):
                self.get_logger().info("All waypoints completed. Initiating landing.")
                self.state = 'landing'
                return

            tx, ty, tz = self.waypoints[self.wp_i]
            dx, dy, dz = (tx - x), (ty - y), (tz - z)

            # Step 1: altitude (optional)
            if self.use_z and abs(dz) > self.z_tol:
                vx = vy = 0.0
                vz = self._clip(self.kp_z * dz, self.vmax_z)
            else:
                # Step 2: XY drive
                dist_xy = math.hypot(dx, dy)
                if dist_xy > self.xy_tol:
                    # Compute XY in correct frame
                    if self.frame_mode == 'body':
                        c, s = math.cos(self.yaw), math.sin(self.yaw)
                        # R(-yaw) * [dx; dy]
                        dx_b =  c * dx + s * dy
                        dy_b = -s * dx + c * dy
                        vx = self._clip(self.kp_xy * dx_b, self.vmax_xy)
                        vy = self._clip(self.kp_xy * dy_b, self.vmax_xy)
                    else:
                        vx = self._clip(self.kp_xy * dx, self.vmax_xy)
                        vy = self._clip(self.kp_xy * dy, self.vmax_xy)
                    vz = 0.0
                else:
                    # Arrived: hold, then advance
                    self._publish_vel(0.0, 0.0, 0.0)
                    if self.hold_t <= 0.0:
                        self.hold_t = self.hold_s
                        self.get_logger().info(f"Reached waypoint {self.wp_i+1}/{len(self.waypoints)}; holding...")
                    else:
                        self.hold_t -= self.dt
                        if self.hold_t <= 0.0:
                            self.wp_i += 1
                            self.get_logger().info("Proceeding to next waypoint...")
                    return

            self._publish_vel(vx, vy, vz)
            return

        if self.state == 'landing':
            target_z = 0.1
            dz = target_z - z
            if abs(dz) < self.z_tol:
                self._publish_vel(0.0, 0.0, 0.0)
                self.state = 'idle'
                self.get_logger().info("Mission complete. Landed (sim).")
                return
            vz = self._clip(self.kp_z * dz, self.vmax_z)
            self._publish_vel(0.0, 0.0, vz)
            return

    # ---------------- Helpers ----------------
    def _publish_vel(self, vx, vy, vz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)
        self.cmd_pub.publish(msg)

    def _publish_path(self):
        path = Path()
        path.header.frame_id = self.world_frame
        for (x, y, z) in self.waypoints:
            ps = PoseStamped()
            ps.header.frame_id = self.world_frame
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = float(z)
            path.poses.append(ps)
        self.path_pub.publish(path)

    def _path_to_waypoints(self, path_msg: Path):
        return [(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z) for ps in path_msg.poses]

    def _gui_path_cb(self, msg: Path):
        wps = self._path_to_waypoints(msg)
        if len(wps) < 3:
            self.get_logger().warn("GUI path has <3 waypoints; ignoring.")
            return
        if self.optimise_path:
            wps = self._tsp_reorder(wps)  # stub
        self.waypoints = wps
        self.wp_i = 0
        self.get_logger().info(f"Loaded {len(wps)} GUI waypoints.")
        self._publish_path()

    def _tsp_reorder(self, wps):
        # Placeholder: return unchanged. (Swap in NN/2-opt later.)
        return wps

    @staticmethod
    def _clip(v, vmax):
        return max(min(v, vmax), -vmax)

def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("MovementNode interrupted.")
    finally:
        try:
            node._publish_vel(0.0, 0.0, 0.0)
        except Exception as e:
            node.get_logger().debug(f"Ignoring shutdown publish error: {e}")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

