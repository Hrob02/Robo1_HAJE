#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import OccupancyGrid

import xml.etree.ElementTree as ET

import fireriskMap as fire

class RunningMap(Node):
    def __init__(self):
        super().__init__('fire_map_node')

        # Get parameters
        self.declare_parameter('world_path', '')


        self.world_path = self.get_parameter('world_path').get_parameter_value().string_value
        self.get_logger().info(f"Using world file: {self.world_path}")


        self.world_size = self.extract_world_size(self.world_path)

        self.fire_map = fire.FireRiskMap(world_size=self.world_size, resolution=0.8)

        # Subscribe to map so we can get origin/resolution from real SLAM map
        self.map_sub_ = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # tree pose subscription (correct message type)
        self.treepose_sub_ = self.create_subscription(Float64MultiArray, '/detected_tree_pose', self.treepose_callback, 10)

        self.small_trees = []
        self.medium_trees = []
        self.large_trees = []

        # Timer for main loop
        self.main_loop_timer_ = self.create_timer(0.2, self.run)

        self.get_logger().info("RunningMap node started. Waiting for /map metadata and /detected_tree_pose messages.")
    
    def extract_world_size(self, world_path):
        try:
            tree = ET.parse(world_path)
            root = tree.getroot()
            size_tag = root.find(".//size")
            if size_tag is not None:
                size_values = [float(x) for x in size_tag.text.split()]
                # Usually in SDF: <size>x y z</size>
                world_size = max(size_values[:2])  # use max of x,y
                self.get_logger().info(f"World size extracted from {world_path}: {world_size}")
                return world_size
            else:
                self.get_logger().warn(f"No <size> tag found in {world_path}, defaulting to 20m.")
                return 20.0
        except Exception as e:
            self.get_logger().error(f"Failed to read world file: {e}")
            return 20.0

    def map_callback(self, msg: OccupancyGrid):
        # Update origin & resolution from actual map metadata
        try:
            origin = msg.info.origin.position
            self.fire_map.origin_x = origin.x
            self.fire_map.origin_y = origin.y
            # update resolution and grid size if map differs
            if abs(self.fire_map.resolution - msg.info.resolution) > 1e-6:
                self.fire_map.resolution = msg.info.resolution
                self.fire_map.grid_size = int(self.fire_map.world_size / self.fire_map.resolution)
                # resize arrays to match new grid_size
                self.fire_map.veg_map = np.zeros((self.fire_map.grid_size, self.fire_map.grid_size), dtype=int)
                self.fire_map.fire_map = np.zeros((self.fire_map.grid_size, self.fire_map.grid_size), dtype=float)
            self.get_logger().info(f"Map metadata updated: origin=({self.fire_map.origin_x:.2f},{self.fire_map.origin_y:.2f}), res={self.fire_map.resolution:.3f}")
        except Exception as e:
            self.get_logger().warning(f"Failed to parse /map metadata: {e}")

    def treepose_callback(self, msg: Float64MultiArray):
        data = msg.data
        n = len(data)

        # clear previous data
        self.small_trees.clear()
        self.medium_trees.clear()
        self.large_trees.clear()

        # every triplet [x, y, z]
        for i in range(0, n, 3):
            try:
                x, y, z = data[i], data[i+1], int(data[i+2])
            except Exception:
                continue

            if z == 1:
                self.small_trees.append((x, y))
            elif z == 2:
                self.medium_trees.append((x, y))
            elif z == 3:
                self.large_trees.append((x, y))

        self.get_logger().debug(f"Received {len(self.small_trees)} small, {len(self.medium_trees)} medium, {len(self.large_trees)} tall trees")

    def run(self):
        shortTrees = self.small_trees
        mediumTrees = self.medium_trees
        tallTrees = self.large_trees

        # Call update plot (this uses origin/resolution set from /map)
        self.fire_map.visibleVegetationMap(shortTrees, mediumTrees, tallTrees, True)

        ffdiValue = self.fire_map.ffdiCalculator(35, 20, 30, 8)
        ffdiRating = self.fire_map.ffdiRating(ffdiValue)
        # self.get_logger().info(f"FFDI {ffdiValue:.1f} -> {ffdiRating}")

def main():
    rclpy.init()
    node = RunningMap()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
