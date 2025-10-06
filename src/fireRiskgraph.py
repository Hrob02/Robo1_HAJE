
import math
import random

import matplotlib
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

import numpy as np


class Graph:
    def __init__(self, parent_logger, map, grid_step_size):
        self.parent_logger_ = parent_logger
        self.map_ = map

        self.nodes_ = []
        self.groups_ = None

        # Visualisation Marker (you can ignore this)
        self.marker_nodes_ = Marker()
        self.marker_nodes_.header.frame_id = "map"
        self.marker_nodes_.ns = "nodes"
        self.marker_nodes_.id = 0
        self.marker_nodes_.type = Marker.POINTS
        self.marker_nodes_.action = Marker.ADD
        self.marker_nodes_.pose.position.x = 0.0
        self.marker_nodes_.pose.position.y = 0.0
        self.marker_nodes_.pose.position.z = 0.0
        self.marker_nodes_.pose.orientation.x = 0.0
        self.marker_nodes_.pose.orientation.y = 0.0
        self.marker_nodes_.pose.orientation.z = 0.0
        self.marker_nodes_.pose.orientation.w = 1.0
        self.marker_nodes_.scale.x = .03
        self.marker_nodes_.scale.y = .03
        self.marker_nodes_.scale.z = .03
        self.marker_nodes_.color.a = 1.0
        self.marker_nodes_.color.r = 1.0
        self.marker_nodes_.color.g = 0.2
        self.marker_nodes_.color.b = 0.2

        self.marker_start_ = Marker()
        self.marker_start_.header.frame_id = "map"
        self.marker_start_.ns = "start"
        self.marker_start_.id = 0
        self.marker_start_.type = Marker.POINTS
        self.marker_start_.action = Marker.ADD
        self.marker_start_.pose.position.x = 0.0
        self.marker_start_.pose.position.y = 0.0
        self.marker_start_.pose.position.z = 0.0
        self.marker_start_.pose.orientation.x = 0.0
        self.marker_start_.pose.orientation.y = 0.0
        self.marker_start_.pose.orientation.z = 0.0
        self.marker_start_.pose.orientation.w = 1.0
        self.marker_start_.scale.x = .2
        self.marker_start_.scale.y = .2
        self.marker_start_.scale.z = .2
        self.marker_start_.color.a = 1.0
        self.marker_start_.color.r = 1.0
        self.marker_start_.color.g = 1.0
        self.marker_start_.color.b = 0.2

        self.marker_visited_ = Marker()
        self.marker_visited_.header.frame_id = "map"
        self.marker_visited_.ns = "visited"
        self.marker_visited_.id = 0
        self.marker_visited_.type = Marker.POINTS
        self.marker_visited_.action = Marker.ADD
        self.marker_visited_.pose.position.x = 0.0
        self.marker_visited_.pose.position.y = 0.0
        self.marker_visited_.pose.position.z = 0.0
        self.marker_visited_.pose.orientation.x = 0.0
        self.marker_visited_.pose.orientation.y = 0.0
        self.marker_visited_.pose.orientation.z = 0.0
        self.marker_visited_.pose.orientation.w = 1.0
        self.marker_visited_.scale.x = .05
        self.marker_visited_.scale.y = .05
        self.marker_visited_.scale.z = .05
        self.marker_visited_.color.a = 1.0
        self.marker_visited_.color.r = 0.2
        self.marker_visited_.color.g = 0.2
        self.marker_visited_.color.b = 1.0

        self.marker_unvisited_ = Marker()
        self.marker_unvisited_.header.frame_id = "map"
        self.marker_unvisited_.ns = "unvisited"
        self.marker_unvisited_.id = 0
        self.marker_unvisited_.type = Marker.POINTS
        self.marker_unvisited_.action = Marker.ADD
        self.marker_unvisited_.pose.position.x = 0.0
        self.marker_unvisited_.pose.position.y = 0.0
        self.marker_unvisited_.pose.position.z = 0.0
        self.marker_unvisited_.pose.orientation.x = 0.0
        self.marker_unvisited_.pose.orientation.y = 0.0
        self.marker_unvisited_.pose.orientation.z = 0.0
        self.marker_unvisited_.pose.orientation.w = 1.0
        self.marker_unvisited_.scale.x = .06
        self.marker_unvisited_.scale.y = .06
        self.marker_unvisited_.scale.z = .06
        self.marker_unvisited_.color.a = 1.0
        self.marker_unvisited_.color.r = 0.3
        self.marker_unvisited_.color.g = 1.0
        self.marker_unvisited_.color.b = 0.3
        
        self.marker_edges_ = Marker()
        self.marker_edges_.header.frame_id = "map"
        self.marker_edges_.ns = "edges"
        self.marker_edges_.id = 0
        self.marker_edges_.type = Marker.LINE_LIST
        self.marker_edges_.action = Marker.ADD
        self.marker_edges_.pose.position.x = 0.0
        self.marker_edges_.pose.position.y = 0.0
        self.marker_edges_.pose.position.z = 0.0
        self.marker_edges_.pose.orientation.x = 0.0
        self.marker_edges_.pose.orientation.y = 0.0
        self.marker_edges_.pose.orientation.z = 0.0
        self.marker_edges_.pose.orientation.w = 1.0

        self.marker_edges_.scale.x = 0.008
        self.marker_edges_.scale.y = 0.008
        self.marker_edges_.scale.z = 0.008
        self.marker_edges_.color.a = 1.0
        self.marker_edges_.color.r = 1.0
        self.marker_edges_.color.g = 1.0
        self.marker_edges_.color.b = 0.4
        self.marker_edges_.colors = []

        self.create_grid(grid_step_size)


def create_grid(self, grid_step_size):
        """Build regular grid across map"""

        # Create nodes
        idx = 0
        for x in range(self.map_.min_x_, self.map_.max_x_-1, grid_step_size):
            for y in range(self.map_.min_y_, self.map_.max_y_-1, grid_step_size):

                # Check if it is occupied
                occupied = self.map_.is_occupied(x,y)

                # Create the node
                if not occupied:
                    self.nodes_.append(Node(x,y,idx))
                    idx = idx + 1

        # Create edges
        count = 0
        # distance_threshold = math.sqrt(2*(grid_step_size*1.01)**2) # Chosen so that diagonals are connected, but not 2 steps away
        distance_threshold = grid_step_size*1.01 # only 4 connected
        for node_i in self.nodes_:
            count = count + 1
            # self.parent_logger_.info(f'{count} of {len(self.nodes_)}')

            for node_j in self.nodes_:

                # Don't create edges to itself
                if node_i != node_j:

                    # Check if the nodes are close to each other
                    distance = node_i.distance_to(node_j)
                    if distance < distance_threshold:

                        # Check edge is collision free
                        if node_i.is_connected(self.map_.obstacle_map_, node_j):

    

                            energy_cost = distance

                            # Create the edge
                            node_i.neighbours.append(node_j)
                            node_i.neighbour_costs.append(energy_cost)

def get_closest_node(self, xy):
        """
        Find closest node to the given xy point
            input: xy is a point in the form of an array, such that x=xy[0] and y=xy[1]. 
            output: return the index of the node in self.nodes_ that has the lowest Euclidean distance to the point xy. 
        """

        best_dist = 999999999
        best_index = None

        my_pos = Node(xy[0], xy[1], 999999999)

        for i in range(len(self.nodes_)):

            ####################
            ## YOUR CODE HERE ##
            ## Task 1         ##
            ####################
            dist = my_pos.distance_to(self.nodes_[i])
            
            if dist < best_dist:
                best_dist = dist
                best_index = self.nodes_[i].idx



        return best_index


def generate_start_end_markers(self, start_idx, goal_idx):
        """Only publish the start and end markers once per path"""

        self.marker_start_.points = []
        node_i = self.nodes_[start_idx]
        p = self.map_.pixel_to_world(node_i.x, node_i.y)
        point = Point(x=p[0], y=p[1], z=p[2]+0.09)
        self.marker_start_.points.append(point)

        node_i = self.nodes_[goal_idx]
        p = self.map_.pixel_to_world(node_i.x, node_i.y)
        point = Point(x=p[0], y=p[1], z=p[2]+0.07)
        self.marker_start_.points.append(point)

        return self.marker_start_

def generate_search_markers(self, visited_set, unvisited_set):
        """Visualise the nodes with these node indices"""

        self.marker_visited_.points = []
        for i in visited_set:
            node_i = self.nodes_[i]
            p = self.map_.pixel_to_world(node_i.x, node_i.y)
            point = Point(x=p[0], y=p[1], z=p[2]+0.07)
            self.marker_visited_.points.append(point)

        self.marker_unvisited_.points = []
        for i in unvisited_set:
            node_i = self.nodes_[i]
            p = self.map_.pixel_to_world(node_i.x, node_i.y)
            point = Point(x=p[0], y=p[1], z=p[2]+0.08)
            self.marker_unvisited_.points.append(point)

        return self.marker_visited_, self.marker_unvisited_

class Node:
    def __init__(self, x, y, idx):

        # Index of the node in the graph
        self.idx = idx

        # Position of node
        self.x = x
        self.y = y

        # Neighbouring edges
        self.neighbours = []
        self.neighbour_costs = []

        # Search parameters
        self.cost_to_node = 999999999 # A large number
        self.cost_to_node_to_goal_heuristic = 999999999 # A large number
        self.parent_node = None # Invalid parent

    def distance_to(self, other_node):
        return math.sqrt((self.x-other_node.x)**2 + (self.y-other_node.y)**2)

    def is_connected(self, img, other_node):
        p1 = [self.x, self.y]
        p2 = [other_node.x, other_node.y]
        return not is_occluded(img, p1, p2)

def is_occluded(img, p1, p2, threshold=0.5):
    """
    Find if two points can be joined or if there's an obstacle in between
    Draws a line from p1 to p2
    Stops at the first pixel that is a "hit", i.e. above the threshold
    Returns the pixel coordinates for the first hit
    """

    # Extract the vector
    x1 = float(p1[0])
    y1 = float(p1[1])
    x2 = float(p2[0])
    y2 = float(p2[1])

    step = 1.0

    dx = x2 - x1
    dy = y2 - y1
    l = math.sqrt(dx**2. + dy**2.)
    if l == 0:
        return False
    dx = dx / l
    dy = dy / l

    max_steps = int(l / step)

    for i in range(max_steps):

        # Get the next pixel
        x = int(round(x1 + dx*i))
        y = int(round(y1 + dy*i))

        # Check if it's outside the image
        if x < 0 or x >= img.shape[0] or y < 0 or y >= img.shape[1]:
            return False

        # Check for "hit"
        if img[x, y] >= threshold:
            return True

    # No hits found
    return False
