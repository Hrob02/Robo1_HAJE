
#!/usr/bin/env python3
# HAJE Movement Subsystem (ROS 2 Humble + Ignition/Gazebo)
#
# PASS scope:
# - /movement/go starts the mission; /movement/stop stops.
# - Executes ≥3 waypoints in order with arrival tolerances and per-WP hold.
# - Publishes geometry_msgs/Twist to drive the drone (yaw not controlled).
# - Publishes nav_msgs/Path on /movement/path (with proper header stamps).
#
# Assumptions:
# - Use simulator odometry: /model/<name>/odometry (defaults to parrot).

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path

class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_controller')

        # ---------- Parameters ----------
        self.declare_parameter('drone_name', 'parrot')
        name = self.get_parameter('drone_name').get_parameter_value().string_value

        # Topics (SAFE defaults for the sim)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic',    f'/model/{name}/odometry')

        # Frames (for RViz path)
        self.declare_parameter('world_frame', 'map')

        # Behaviour
        self.declare_parameter('skip_takeoff', True)   # start directly in mission
        self.declare_parameter('use_z_control', False) # ignore Z by default

        # Controller & mission params
        self.declare_parameter('kp_xy',    0.6)
        self.declare_parameter('kp_z',     0.8)
        self.declare_parameter('max_xy',   0.5)   # m/s (kept modest)
        self.declare_parameter('max_z',    0.5)   # m/s
        self.declare_parameter('xy_tol',   0.35)  # m
        self.declare_parameter('z_tol',    0.25)  # m
        self.declare_parameter('hold_sec', 2.0)
        self.declare_parameter('tick_hz',  20.0)

        # ≥3 waypoints
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

        self.skip_takeoff  = bool(self.get_parameter('skip_takeoff').value)
        self.use_z         = bool(self.get_parameter('use_z_control').value)

        self.kp_xy   = float(self.get_parameter('kp_xy').value)
        self.kp_z    = float(self.get_parameter('kp_z').value)
        self.vmax_xy = float(self.get_parameter('max_xy').value)
        self.vmax_z  = float(self.get_parameter('max_z').value)
        self.xy_tol  = float(self.get_parameter('xy_tol').value)
        self.z_tol   = float(self.get_parameter('z_tol').value)
        self.hold_s  = float(self.get_parameter('hold_sec').value)
        tick_hz      = float(self.get_parameter('tick_hz').value)
        self.dt      = 1.0 / tick_hz

        # Waypoints
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

        self.path_pub = self.create_publisher(Path, 'movement/path', 10)
        self._publish_path()

        # ---------- State ----------
        self.pos = None
        self.state = 'idle'     # 'idle' | 'takeoff' | 'mission' | 'landing'
        self.wp_i = 0
        self.hold_t = 0.0

        self.timer = self.create_timer(self.dt, self._tick)
        self.get_logger().info(f"MovementNode ready. cmd_vel={self.cmd_vel_topic}, odom={self.odom_topic}. Waiting for GO.")

    # ---------------- Callbacks ----------------
    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pos = (p.x, p.y, p.z)

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
        phase = 'MISSION (skip_takeoff)' if self.skip_takeoff else 'TAKEOFF'
        self.get_logger().info(f"GO: entering {phase}.")

    def _stop_cb(self, _: Empty):
        self.get_logger().info("STOP: zeroing velocity and resetting to idle.")
        self._publish_vel(0.0, 0.0, 0.0)
        self.state = 'idle'
        self.wp_i = 0
        self.hold_t = 0.0

    # ---------------- Control loop ----------------
    def _tick(self):
        if self.state == 'idle' or self.pos is None:
            return

        x, y, z = self.pos

        if self.state == 'takeoff':
            target_z = self.waypoints[0][2]
            vz = self._clip(self.kp_z * (target_z - z), self.vmax_z)
            vx = vy = 0.0
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
                # Step 2: XY drive in world frame
                dist_xy = math.hypot(dx, dy)
                if dist_xy > self.xy_tol:
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
        now = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = self.world_frame
        path.header.stamp = now
        for (x, y, z) in self.waypoints:
            ps = PoseStamped()
            ps.header.frame_id = self.world_frame
            ps.header.stamp = now
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = float(z)
            ps.pose.orientation.w = 1.0  # identity
            path.poses.append(ps)
        self.path_pub.publish(path)

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
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

