#!/usr/bin/env python3

import math
import random
from enum import Enum
import numpy as np
from scipy.ndimage import label, center_of_mass

import cv2  # OpenCV2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Pose2D, PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray



def wrap_angle(angle):
    """Function to wrap an angle between 0 and 2*Pi"""
    while angle < 0.0:
        angle = angle + 2 * math.pi

    while angle > 2 * math.pi:
        angle = angle - 2 * math.pi

    return angle

def pose2d_to_pose(pose_2d):
    """Convert a Pose2D to a full 3D Pose"""
    pose = Pose()

    pose.position.x = pose_2d.x
    pose.position.y = pose_2d.y

    pose.orientation.w = math.cos(pose_2d.theta / 2.0)
    pose.orientation.z = math.sin(pose_2d.theta / 2.0)

    return pose


class PlannerType(Enum):
    ERROR = 0
    MOVE_FORWARDS = 1
    RETURN_HOME = 2
    RANDOM_WALK = 4
    RANDOM_GOAL = 5

    FRONTIER_EXPLORATION = 6


class FirelandExplorer(Node):
    def __init__(self):
        super().__init__('fireland_explorer')

        # Variables/Flags for mapping
        self.xlim_ = [0.0, 0.0]
        self.ylim_ = [0.0, 0.0]


        # Variables/Flags for planning
        self.planner_type_ = PlannerType.FRONTIER_EXPLORATION

        self.returned_home_ = False

        # Marker for artifact locations
        # See https://wiki.ros.org/rviz/DisplayTypes/Marker

        self.marker_pub_ = self.create_publisher(MarkerArray, 'marker_array_artifacts', 10) # Creat publisher for marker array, but new marker is created for each location
        self.marker_artifacts_array = []

        # Remember the artifact locations
        # Array of type geometry_msgs.Point
        self.artifact_locations_ = []

        # Initialise CvBridge
        self.cv_bridge_ = CvBridge()

        # Prepare transformation to get robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action client for nav2
        self.nav2_action_client_ = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().warn('Waiting for navigate_to_pose action...')
        self.nav2_action_client_.wait_for_server()
        self.get_logger().warn('navigate_to_pose connected')
        self.ready_for_next_goal_ = True
        self.declare_parameter('print_feedback', rclpy.Parameter.Type.BOOL)

        # Publisher for the goal pose visualisation
        self.goal_pose_vis_ = self.create_publisher(PoseStamped, 'goal_pose', 1)

        # Subscribe to the map topic to get current bounds
        self.map_sub_ = self.create_subscription(OccupancyGrid, 'map',  self.map_callback, 1)

        self.cv_bridge_ = CvBridge()
                                                          


        # Timer for main loop
        self.main_loop_timer_ = self.create_timer(0.2, self.main_loop)

        # Parameter for navigation type
        self.declare_parameter('planner_type', 'frontier_exploration')
        planner_value = self.get_parameter('planner_type').value
        self.get_logger().info(f'Planner parameter initialised as: {planner_value}')

        self.current_goal_ = None
        self.goal_timeout_sec_ = 8 # Time in seconds rover will spend going to each goal
        self.goal_start_time_ = None
        self.min_unknown_cell_clusters = 16 # Size of grouped consective cells to be valid frontier goal

        # Goal timeout timer
        self.goal_timeout_timer_ = self.create_timer(0.5, self.check_goal_timeout)

   

        # Stores the active nav2 goal handle so we can cancel
        self.current_goal_handle_ = None

        self.take_off = False




    
    def get_pose_2d(self):
        """Get the 2d pose of the robot"""

        # Lookup the latest transform
        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().error(f'Could not transform: {ex}')
            return

        # Return a Pose2D message
        pose = Pose2D()
        pose.x = t.transform.translation.x
        pose.y = t.transform.translation.y

        qw = t.transform.rotation.w
        qz = t.transform.rotation.z

        if qz >= 0.:
            pose.theta = wrap_angle(2. * math.acos(qw))
        else: 
            pose.theta = wrap_angle(-2. * math.acos(qw))

        # self.get_logger().warn(f'Pose: {pose}')

        return pose

    def map_callback(self, map_msg: OccupancyGrid):
        """New map received, so update x and y limits"""

        # Extract data from message
        self.map_origin_ = map_msg.info.origin
        self.map_resolution_ = map_msg.info.resolution
        self.map_height_ = map_msg.info.height
        self.map_width_ = map_msg.info.width
        self.map_data_ = map_msg.data

        # Set current limits
        self.xlim_ = [self.map_origin_.position.x, self.map_origin_.position.x + self.map_width_ * self.map_resolution_]
        self.ylim_ = [self.map_origin_.position.y, self.map_origin_.position.y + self.map_height_ * self.map_resolution_]



    def planner_go_to_pose2d(self, pose2d, force: bool = False):
        """Go to a provided 2d pose"""

        # Send a goal to navigate_to_pose with self.nav2_action_client_
        action_goal = NavigateToPose.Goal()
        action_goal.pose.header.stamp = self.get_clock().now().to_msg()
        action_goal.pose.header.frame_id = 'map'
        action_goal.pose.pose = pose2d_to_pose(pose2d)

        # Publish visualisation
        self.goal_pose_vis_.publish(action_goal.pose)

        # Decide whether to show feedback or not
        if self.get_parameter('print_feedback').value:
            feedback_method = self.feedback_callback
        else:
            feedback_method = None

        # Send goal to action server
        if self.ready_for_next_goal_ or force:
            self.get_logger().warn(f'Sending goal [{pose2d.x:.2f}, {pose2d.y:.2f}]...')
            self.send_goal_future_ = self.nav2_action_client_.send_goal_async(
                action_goal,
                feedback_callback=feedback_method)
            self.send_goal_future_.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """The requested goal pose has been sent to the action server"""
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Goal response exception: {e}')
            return

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        # store handle so we can cancel later if needed
        self.current_goal_handle_ = goal_handle

        # Goal accepted: get result when it's completed
        self.get_logger().warn('Goal accepted')
        self.goal_start_time_ = self.get_clock().now()
        self.get_result_future_ = goal_handle.get_result_async()
        self.get_result_future_.add_done_callback(self.goal_reached_callback)
        self.ready_for_next_goal_ = False



    def feedback_callback(self, feedback_msg):
        """Monitor the feedback from the action server"""

        feedback = feedback_msg.feedback

        self.get_logger().info(f'{feedback.distance_remaining:.2f} m remaining')

    def goal_reached_callback(self, future):
        """The requested goal has been reached"""
        try:
            result = future.result().result
        except Exception:
            result = None

        self.get_logger().info('Goal reached!')
        # clear stored handle
        self.current_goal_handle_ = None

        self.current_goal_ = None
        self.ready_for_next_goal_ = True
        self.goal_start_time_ = None


    def main_loop(self):
        """
        Set the next goal pose and send to the action server
        See https://docs.nav2.org/concepts/index.html
        """

        """Main decision loop"""

        if not self.take_off:
            self.takeoff()
            self.take_off = True

        planner_str = self.get_parameter('planner_type').value

        self.get_logger().debug(f'Loop running; planner_type = {self.planner_type_}, parameter = {self.get_parameter("planner_type").value}')

        if not self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
            self.get_logger().warn('Waiting for transform... Have you launched SLAM?')
            return


        if not self.ready_for_next_goal_:
            return

        
        if planner_str == 'frontier_exploration':
            self.get_logger().info(f'Calling planner: {self.planner_type_.name}')
            self.planner_frontier_exploration()
  
        else:
            self.get_logger().error('No valid planner selected')
            self.destroy_node()
    
    def takeoff(self):
        """Setting the Z-axis / height of drone """
        pose = Pose()
        pose.position.z = 3.0

        # Send a goal to navigate_to_pose with self.nav2_action_client_
        action_goal = NavigateToPose.Goal()
        action_goal.pose.header.stamp = self.get_clock().now().to_msg()
        action_goal.pose.header.frame_id = 'map'


        # Publish visualisation
        self.goal_pose_vis_.publish(action_goal.pose)

        



    def find_frontiers(self):
        """Find clusters of frontier cells and return centroids as candidate goals (only if enough unknown nearby)."""
 
        if not hasattr(self, 'map_data_'):
            self.get_logger().debug("No map data yet.")
            return []
    
        width = self.map_width_
        height = self.map_height_
        data = np.array(self.map_data_, dtype=np.int8).reshape((height, width))

        # Frontier mask: free cells (0) adjacent to unknown (-1)
        frontier_mask = np.zeros_like(data, dtype=bool)
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if data[y, x] == 0:  # free cell
                    neighborhood = data[y-1:y+2, x-1:x+2]
                    if np.any(neighborhood == -1):
                        frontier_mask[y, x] = True

        # Label contiguous frontier clusters
        structure = np.ones((3, 3), dtype=int)
        labeled, num_features = label(frontier_mask, structure=structure)

        if num_features == 0:
            self.get_logger().debug("No frontier clusters found.")
            return []

        # Compute centroids (in world coordinates)
        centroids = []
        for i in range(1, num_features + 1):
            com = center_of_mass(frontier_mask, labels=labeled, index=i)
            if np.isnan(com[0]):
                continue

            # Convert to world coordinates
            wy = self.map_origin_.position.y + com[0] * self.map_resolution_
            wx = self.map_origin_.position.x + com[1] * self.map_resolution_

            # Check how many unknown cells surround this cluster
            fx = int((wx - self.map_origin_.position.x) / self.map_resolution_)
            fy = int((wy - self.map_origin_.position.y) / self.map_resolution_)
            r = int(1.0 / self.map_resolution_)  
            x_min, x_max = max(0, fx - r), min(width, fx + r)
            y_min, y_max = max(0, fy - r), min(height, fy + r)
            patch = data[y_min:y_max, x_min:x_max]
            unknown_count = np.sum(patch == -1)

            # Only accept frontiers with enough unexplored area
            if unknown_count >= self.min_unknown_cell_clusters:
                centroids.append((wx, wy))


        self.get_logger().info(f"Found {len(centroids)} valid frontier clusters (of {num_features} total).")
        return centroids



    def choose_frontier_goal(self, frontiers, robot_pose, min_dist=1):
        """Select the best frontier based on distance and information gain."""
        if not frontiers:
            return None

        best_score = float('-inf')
        best_frontier = None

        if self.ready_for_next_goal_:

            for f in frontiers:
                dist = math.hypot(f[0] - robot_pose.x, f[1] - robot_pose.y)
                if dist < min_dist:
                    continue

                score = -dist + 1.0 / (1.0 + math.exp(-0.2 * (dist - 1.0)))
                if score > best_score:
                    best_score = score
                    best_frontier = f

            if best_frontier:
                self.get_logger().info(
                    f"Chosen frontier: ({best_frontier[0]:.2f}, {best_frontier[1]:.2f}) with score={best_score:.2f}"
                )
            else:
                self.get_logger().warn("No valid frontier goal selected.")

        return best_frontier


    def planner_frontier_exploration(self):
        """Main frontier exploration loop."""

        # Skip if still travelling to a goal
        if not self.ready_for_next_goal_:
            return
        

        robot_pose = self.get_pose_2d()
        if robot_pose is None:
            self.get_logger().warn("Cannot get robot pose yet.")
            return

        frontiers = self.find_frontiers()

        if not frontiers:
            self.get_logger().warn("No frontiers found. Maybe fully explored?")
            self.ready_for_next_goal_ = True
            return

        # Publish for RViz visualisation
        self.publish_frontier_markers(frontiers)

        # Choose next goal
        goal = self.choose_frontier_goal(frontiers, robot_pose)

        if goal is None:
            self.get_logger().info(f"NO GOAL, trying again")
            self.ready_for_next_goal_ = True
            return




        self.get_logger().info(f"Exploring frontier at ({goal[0]:.2f}, {goal[1]:.2f})")

        goal_pose = Pose2D(x=goal[0], y=goal[1], theta=0.0)
        self.current_goal_ = goal_pose
        self.planner_go_to_pose2d(goal_pose)




    def check_goal_timeout(self):
        """Check if the current goal has timed out."""

        if self.goal_start_time_ is None or self.ready_for_next_goal_:
            return

        elapsed = (self.get_clock().now() - self.goal_start_time_).nanoseconds / 1e9

        if elapsed > self.goal_timeout_sec_:
            self.get_logger().warn(f"Frontier goal timeout after {elapsed:.1f}s. Choosing new frontier.")
            self.ready_for_next_goal_ = True
            self.goal_start_time_ = None
            self.current_goal_ = None



    def publish_frontier_markers(self, frontiers):
        """Visualise detected frontier points in RViz as blue dots."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "frontiers"
        marker.id = 1
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.points = [Point(x=f[0], y=f[1], z=0.0) for f in frontiers]

        marker_array = MarkerArray()
        marker_array.markers = [marker]
        self.marker_pub_.publish(marker_array)


def main():
    # Initialise
    rclpy.init()

    # Create the fireland explorer
    fireland_explorer = FirelandExplorer()

    while rclpy.ok():
        rclpy.spin(fireland_explorer)