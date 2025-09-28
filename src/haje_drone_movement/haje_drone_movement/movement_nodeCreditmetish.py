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
# - Use simulator odometry: /model/<name>/odometry (defaults to parrot), or /odometry via bridge.
#
# This version adds "face_forward" yaw control so the drone turns toward the
# direction of travel (publishes Twist.angular.z).
# HAJE Movement Subsystem (ROS 2 Humble + Gazebo/GZ)
# CREDIT tier: GUI goal subscription + queuing + defined path visualisation.
# HAJE Movement Subsystem (ROS 2 Humble + Gazebo/GZ)
# CREDIT tier: GUI goal subscription + queuing + defined path + markers.

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray

def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_controller')

        # ---------- Parameters ----------
        self.declare_parameter('drone_name', 'parrot')
        name = self.get_parameter('drone_name').get_parameter_value().string_value

        # Topics
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic',    f'/model/{name}/odometry')

        # Frames (for RViz path/markers)
        self.declare_parameter('world_frame', 'map')  # use 'odom' when running without SLAM

        # Behaviour
        self.declare_parameter('goal_source', 'param')  # 'param' or 'gui'
        self.declare_parameter('skip_takeoff', True)
        self.declare_parameter('use_z_control', False)

        # Controller & mission params
        self.declare_parameter('kp_xy',    0.6)
        self.declare_parameter('kp_z',     0.8)
        self.declare_parameter('kp_yaw',   1.2)
        self.declare_parameter('max_xy',   0.5)
        self.declare_parameter('max_z',    0.5)
        self.declare_parameter('max_yaw',  1.0)
        self.declare_parameter('xy_tol',   0.35)
        self.declare_parameter('z_tol',    0.25)
        self.declare_parameter('yaw_tol',  0.20)
        self.declare_parameter('hold_sec', 1.5)
        self.declare_parameter('tick_hz',  20.0)

        # ≥3 waypoints (used when goal_source='param')
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
        self.goal_source   = self.get_parameter('goal_source').get_parameter_value().string_value

        self.skip_takeoff  = bool(self.get_parameter('skip_takeoff').value)
        self.use_z         = bool(self.get_parameter('use_z_control').value)

        self.kp_xy   = float(self.get_parameter('kp_xy').value)
        self.kp_z    = float(self.get_parameter('kp_z').value)
        self.kp_yaw  = float(self.get_parameter('kp_yaw').value)
        self.vmax_xy = float(self.get_parameter('max_xy').value)
        self.vmax_z  = float(self.get_parameter('max_z').value)
        self.vmax_yaw= float(self.get_parameter('max_yaw').value)
        self.xy_tol  = float(self.get_parameter('xy_tol').value)
        self.z_tol   = float(self.get_parameter('z_tol').value)
        self.yaw_tol = float(self.get_parameter('yaw_tol').value)
        self.hold_s  = float(self.get_parameter('hold_sec').value)
        tick_hz      = float(self.get_parameter('tick_hz').value)
        self.dt      = 1.0 / tick_hz

        # Waypoints from parameter (fallback)
        wp_list = self.get_parameter('waypoints').get_parameter_value().double_array_value
        if len(wp_list) % 3 == 0 and len(wp_list) >= 9:
            self.param_waypoints = [(wp_list[i], wp_list[i+1], wp_list[i+2]) for i in range(0, len(wp_list), 3)]
        else:
            self.param_waypoints = [(0.0, 0.0, 2.0), (2.0, 0.0, 2.0), (2.0, 2.0, 2.0), (0.0, 2.0, 2.0)]

        # ---------- IO ----------
        self.cmd_pub  = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)

        self.create_subscription(Empty, 'movement/go',   self._go_cb,   10)
        self.create_subscription(Empty, 'movement/stop', self._stop_cb, 10)

        if self.goal_source.lower() == 'gui':
            self.create_subscription(PoseStamped, 'movement/waypoint', self._waypoint_cb, 10)
            self.create_subscription(Empty,       'movement/clear',    self._clear_cb,    10)
            self.create_subscription(Empty,       'movement/commit',   self._commit_cb,   10)

        # Latched (transient local) so RViz picks them up after start
        path_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        marker_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub   = self.create_publisher(Path,        'movement/path',    path_qos)
        self.marker_pub = self.create_publisher(MarkerArray, 'movement/markers', marker_qos)

        # ---------- State ----------
        self.pos = None
        self.yaw = 0.0
        self.state = 'idle'     # 'idle' | 'takeoff' | 'mission' | 'landing'
        self.wp_i = 0
        self.hold_t = 0.0

        self.active_waypoints = list(self.param_waypoints) if self.goal_source=='param' else []

        self.timer = self.create_timer(self.dt, self._tick)
        self._publish_path()  # initial/fallback path
        self.get_logger().info(f"MovementNode (CREDIT) ready. mode={self.goal_source} cmd_vel={self.cmd_vel_topic}, odom={self.odom_topic}. Waiting for GO/commit.")

    # ---------------- Callbacks ----------------
    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pos = (p.x, p.y, p.z)
        self.yaw = quat_to_yaw(msg.pose.pose.orientation)

    def _go_cb(self, _: Empty):
        if self.goal_source == 'gui' and len(self.active_waypoints) < 3:
            self.get_logger().warn('GO ignored: GUI mode requires ≥3 queued waypoints then send movement/commit.')
            return
        self._start_mission()

    def _stop_cb(self, _: Empty):
        self.get_logger().info("STOP: zeroing velocity and resetting to idle.")
        self._publish_vel(0.0, 0.0, 0.0, 0.0)
        self.state = 'idle'
        self.wp_i = 0
        self.hold_t = 0.0

    # --- GUI-mode waypoint queue management ---
    def _waypoint_cb(self, msg: PoseStamped):
        xyz = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.active_waypoints.append(xyz)
        self._publish_path()
        self.get_logger().info(f'GUI: enqueued waypoint #{len(self.active_waypoints)}: {xyz}')

    def _clear_cb(self, _: Empty):
        self.active_waypoints.clear()
        self.wp_i = 0
        self._publish_path()
        self.get_logger().info('GUI: cleared queued waypoints.')

    def _commit_cb(self, _: Empty):
        if len(self.active_waypoints) < 3:
            self.get_logger().warn('GUI: commit rejected, need ≥3 waypoints.')
            return
        self._start_mission()

    def _start_mission(self):
        if self.state != 'idle':
            self.get_logger().warn("Start requested but mission already active.")
            return
        if self.pos is None:
            self.get_logger().warn("Start requested but no odometry yet.")
            return
        self.wp_i = 0
        self.hold_t = 0.0
        self.state = 'mission' if self.skip_takeoff else 'takeoff'
        phase = 'MISSION (skip_takeoff)' if self.skip_takeoff else 'TAKEOFF'
        self.get_logger().info(f"Start: entering {phase}.")

    # ---------------- Control loop ----------------
    def _tick(self):
        if self.state == 'idle' or self.pos is None:
            return

        x, y, z = self.pos

        if self.state == 'takeoff':
            target_z = self._wps()[0][2]
            vz = self._clip(self.kp_z * (target_z - z), self.vmax_z)
            vx = vy = 0.0
            v_yaw = 0.0
            if abs(target_z - z) < self.z_tol:
                if self.hold_t <= 0.0:
                    self.hold_t = self.hold_s
                    self.get_logger().info("Reached takeoff altitude; holding...")
                else:
                    self.hold_t -= self.dt
                    if self.hold_t <= 0.0:
                        self.state = 'mission'
                        self.get_logger().info("Starting waypoint mission...")
                        vx = vy = vz = v_yaw = 0.0
            self._publish_vel(vx, vy, vz if self.use_z else 0.0, v_yaw)
            return

        if self.state == 'mission':
            if self.wp_i >= len(self._wps()):
                self.get_logger().info("All waypoints completed. Initiating landing.")
                self.state = 'landing'
                return

            tx, ty, tz = self._wps()[self.wp_i]
            dx, dy, dz = (tx - x), (ty - y), (tz - z)

            # Yaw towards target
            target_yaw = math.atan2(dy, dx)
            yaw_err = self._wrap_pi(target_yaw - self.yaw)
            v_yaw = self._clip(self.kp_yaw * yaw_err, self.vmax_yaw)

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
                    self._publish_vel(0.0, 0.0, 0.0, 0.0)
                    if self.hold_t <= 0.0:
                        self.hold_t = self.hold_s
                        self.get_logger().info(f"Reached waypoint {self.wp_i+1}/{len(self._wps())}; holding...")
                    else:
                        self.hold_t -= self.dt
                        if self.hold_t <= 0.0:
                            self.wp_i += 1
                            self.get_logger().info("Proceeding to next waypoint...")
                    return

            self._publish_vel(vx, vy, vz if self.use_z else 0.0, v_yaw)
            return

        if self.state == 'landing':
            target_z = 0.1
            dz = target_z - z
            if abs(dz) < self.z_tol:
                self._publish_vel(0.0, 0.0, 0.0, 0.0)
                self.state = 'idle'
                self.get_logger().info("Mission complete. Landed (sim).")
                return
            vz = self._clip(self.kp_z * dz, self.vmax_z)
            self._publish_vel(0.0, 0.0, self.use_z and vz or 0.0, 0.0)
            return

    # ---------------- Helpers ----------------
    def _wps(self):
        return self.active_waypoints

    def _publish_vel(self, vx, vy, vz, wz):
        msg = Twist()
        msg.linear.x  = float(vx)
        msg.linear.y  = float(vy)
        msg.linear.z  = float(vz)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def _publish_path(self):
        now = self.get_clock().now().to_msg()

        # Path line
        path = Path()
        path.header.frame_id = self.world_frame
        path.header.stamp = now
        for (x, y, z) in self._wps():
            ps = PoseStamped()
            ps.header.frame_id = self.world_frame
            ps.header.stamp = now
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = float(z)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

        # Markers: spheres + labels
        ma = MarkerArray()

        spheres = Marker()
        spheres.header.frame_id = self.world_frame
        spheres.header.stamp = now
        spheres.ns = "wps"
        spheres.id = 0
        spheres.type = Marker.SPHERE_LIST
        spheres.action = Marker.ADD
        spheres.scale.x = spheres.scale.y = spheres.scale.z = 0.2
        spheres.color.r = 0.1; spheres.color.g = 0.8; spheres.color.b = 0.3; spheres.color.a = 0.9
        spheres.points = [Point(x=float(x), y=float(y), z=float(z)) for (x, y, z) in self._wps()]
        ma.markers.append(spheres)

        for i, (x, y, z) in enumerate(self._wps(), start=1):
            t = Marker()
            t.header.frame_id = self.world_frame
            t.header.stamp = now
            t.ns = "wp_labels"
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.scale.z = 0.2
            t.color.r = 1.0; t.color.g = 1.0; t.color.b = 1.0; t.color.a = 1.0
            t.pose.position.x = float(x)
            t.pose.position.y = float(y)
            t.pose.position.z = float(z) + 0.25
            t.text = f"WP{i}"
            ma.markers.append(t)

        self.marker_pub.publish(ma)

    @staticmethod
    def _clip(v, vmax):
        return max(min(v, vmax), -vmax)

    @staticmethod
    def _wrap_pi(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("MovementNode interrupted.")
    finally:
        try:
            node._publish_vel(0.0, 0.0, 0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()


