#!/usr/bin/env python3
from geometry_msgs.msg import Pose, Pose2D, PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from visualization_msgs.msg import MarkerArray

import fireriskMap as fire


# shortTrees = [(10, 15), (25, 30), (40, 5)]
# mediumTrees = [(2, 17), (14, 22), (38, 32)]
# tallTrees = [(29, 2), (9, 29), (11, 1)]


class RunningMap(Node):
    def __init__(self):
        super().__init__('fire_map_node')

        self.fire_map = fire.FireRiskMap(world_size=50, resolution=0.8)
        
        #tree pose subscription
        self.treepose_sub_ = self.create_subscription(MarkerArray, '/detected_tree_pose', self.treepose_callback, 10)

        self.small_trees = []
        self.medium_trees = []
        self.large_trees = []

        # Timer for main loop
        self.main_loop_timer_ = self.create_timer(0.2, self.run)
    
    def treepose_callback(self, msg):

        data = msg.data
        n = len(data)

        # clear previous data
        self.small_trees.clear()
        self.medium_trees.clear()
        self.large_trees.clear()

        # every triplet [x, y, z]
        for i in range(0, n, 3):
            x, y, z = data[i], data[i+1], int(data[i+2])

            if z == 1:
                self.small_trees.append((x, y))
            elif z == 2:
                self.medium_trees.append((x, y))
            elif z == 3:
                self.large_trees.append((x, y))


    def run(self):
        shortTrees = self.small_trees
        mediumTrees = self.medium_trees
        tallTrees = self.large_trees
        
        self.fire_map.visibleVegetationMap(shortTrees, mediumTrees, tallTrees, True)
        # self.fire_map.fireRiskMap()
        
        ffdiValue = self.fire_map.ffdiCalculator(35, 20, 30, 8)
        # print(ffdiValue)
        ffdiRating = self.fire_map.ffdiRating(ffdiValue)
        # print(ffdiRating)


def main():
    # Initialise
    rclpy.init()

    # Create the cave explorer
    fireRiskMap = RunningMap()

    while rclpy.ok():
        rclpy.spin(fireRiskMap)

if __name__ == "__main__":
    # runner = RunningMap()
    # runner.run()
    main()
