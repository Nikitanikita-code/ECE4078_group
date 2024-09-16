import numpy as np
import math
import sys

# map data
map_objects = {
    "garlic_0": {"y": -0.6, "x": 0.7}, "capsicum_0": {"y": -1.0, "x": 0.2}, 
    "aruco10_0": {"y": 0.0, "x": 0.8}, "aruco4_0": {"y": -0.7, "x": -0.3},
    "aruco3_0": {"y": -1.2, "x": 0.8}, "aruco1_0": {"y": 0.8, "x": 1.2}, 
    "aruco8_0": {"y": -1.2, "x": -0.8}, "lime_0": {"y": -0.1, "x": -1.1}, 
    "lemon_0": {"y": -1.2, "x": -0.4}, "aruco5_0": {"y": 1.2, "x": 0.0}, 
    "pear_1": {"y": 0.5, "x": -0.8}, "pear_0": {"y": 0.9, "x": 0.0}, 
    "pumpkin_0": {"y": 1.2, "x": -0.6}, "aruco9_0": {"y": 0.4, "x": -1.2}, 
    "plum_0": {"y": -0.7, "x": -0.6}, "aruco6_0": {"y": 0.0, "x": -0.6}, 
    "plum_1": {"y": 0.4, "x": 0.3}, "aruco7_0": {"y": 0.8, "x": -0.4}, 
    "tomato_0": {"y": 0.8, "x": 0.8}, "aruco2_0": {"y": -0.6, "x": 1.1}
}

# shopping list
shopping_list = ["garlic_0", "lemon_0", "lime_0", "tomato_0", "pumpkin_0"]

# robot's initial pose
robot_pose = {"x": 0, "y": 0}

# pose for items in shopping list
shopping_positions = [map_objects[item] for item in shopping_list]

# need to modify this to come up with optimal order (using spanning tree or tsp, check messenger chat)
optimal_order = shopping_list

print(f"Optimal order to visit: {optimal_order}")

# define obstacles
obstacle_positions = [pos for key, pos in map_objects.items() if key not in shopping_list]

class RRTC:
    """
    Class for RRT planning
    """
    class Node:
        """
        RRT Node
        """
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

        def __eq__(self, other):
            bool_list = []
            bool_list.append(self.x == other.x)
            bool_list.append(self.y == other.y)
            bool_list.append(np.all(np.isclose(self.path_x, other.path_x)))
            bool_list.append(np.all(np.isclose(self.path_y, other.path_y)))
            bool_list.append(self.parent == other.parent)
            return np.all(bool_list)

    def __init__(self, start=np.zeros(2),
                 goal=np.array([120,90]),
                 obstacle_list=None,
                 width = 160,
                 height=100,
                 expand_dis=3.0, 
                 path_resolution=0.5, 
                 max_points=200):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacle_list: list of obstacle objects
        width, height: search area
        expand_dis: min distance between random node and closest node in rrt to it
        path_resolution: step size to considered when looking for node to expand
        """
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.width = width
        self.height = height
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.max_nodes = max_points
        self.obstacle_list = obstacle_list
        self.start_node_list = [] # Tree from start
        self.end_node_list = [] # Tree from end

    def grow_tree(self, tree, node):
        near_nind = self.get_nearest_node_index(tree, node)
        near_n = tree[near_nind]

        new_node = self.steer(near_n, node, self.expand_dis)
        
        # Check if we should add this node or not, and add it to the tree
        added_new_node = self.is_collision_free(new_node)

        if added_new_node == True:
            tree.append(new_node)
        
        return added_new_node

    def check_trees_distance(self):
        # Find the distance between the trees, return if the trees distance is smaller than self.expand_dis
        # In other word, we are checking if we can connect the 2 trees.

        can_be_connected = False
        
        #TODO: Complete the method -------------------------------------------------------------------------
        for node_t1 in self.start_node_list:
            for node_t2 in self.end_node_list:
                dist, ang = self.calc_distance_and_angle(node_t1, node_t2)
                if dist < self.expand_dis:
                    can_be_connected = True
        
        #ENDTODO ----------------------------------------------------------------------------------------------
        
        return can_be_connected

    def planning(self):
        """
        rrt path planning
        """
        self.start_node_list = [self.start]
        self.end_node_list = [self.end]
        while len(self.start_node_list) + len(self.end_node_list) <= self.max_nodes:
            
            samp_node = self.get_random_node()
            added_samp = self.grow_tree(self.start_node_list, samp_node)

            if added_samp:
                new_node = self.start_node_list[-1]
        
                can_connect = self.check_trees_distance()
                
                end_nind = self.get_nearest_node_index(self.end_node_list, new_node)
                dist, ang = self.calc_distance_and_angle(new_node, self.end_node_list[end_nind])
                    
                if dist < self.expand_dis:
                    if can_connect:
                        added_samp_end = self.grow_tree(self.end_node_list, new_node)
    
                        if added_samp_end:
                            return self.generate_final_course(len(self.start_node_list) - 1, len(self.end_node_list) - 1)
                else:
                    n_samp_node = self.get_random_node()
                    added_n_samp_end = self.grow_tree(self.end_node_list, n_samp_node)

            self.start_node_list, self.end_node_list = self.end_node_list, self.start_node_list
            
        return None  # cannot find path
    
    def steer(self, from_node, to_node, extend_length=float("inf")):
        """
        Given two nodes from_node, to_node, this method returns a node new_node such that new_node 
        is “closer” to to_node than from_node is.
        """
        
        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)
        cos_theta, sin_theta = np.cos(theta), np.sin(theta)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        # How many intermediate positions are considered between from_node and to_node
        n_expand = math.floor(extend_length / self.path_resolution)

        # Compute all intermediate positions
        for _ in range(n_expand):
            new_node.x += self.path_resolution * cos_theta
            new_node.y += self.path_resolution * sin_theta
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)

        new_node.parent = from_node

        return new_node

    def is_collision_free(self, new_node):
        if new_node is None:
            return True

        for i in range(len(new_node.path_x) - 1):
            segment_start = (new_node.path_x[i], new_node.path_y[i])
            segment_end = (new_node.path_x[i+1], new_node.path_y[i+1])
            if any(self.check_segment_collision(segment_start, segment_end, obs) for obs in self.obstacle_list):
                return False
        return True

    def check_segment_collision(self, seg_start, seg_end, obstacle):
        obs_x, obs_y = obstacle['x'], obstacle['y']
        
	# i created a bounding box and checked if obstacle is in it, please improve this collision function
        min_x = min(seg_start[0], seg_end[0])
        max_x = max(seg_start[0], seg_end[0])
        min_y = min(seg_start[1], seg_end[1])
        max_y = max(seg_start[1], seg_end[1])
        return min_x <= obs_x <= max_x and min_y <= obs_y <= max_y
    
    def generate_final_course(self, start_mid_point, end_mid_point):
        """
        Reconstruct path from start to end node
        """
        # First half
        node = self.start_node_list[start_mid_point]
        path = []
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])
        
        # Other half
        node = self.end_node_list[end_mid_point]
        path = path[::-1]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        x = self.width * np.random.random_sample()
        y = self.height * np.random.random_sample()
        rnd = self.Node(x, y)
        return rnd

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):        
        # Compute Euclidean disteance between rnd_node and all nodes in tree
        # Return index of closest element
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y)
                 ** 2 for node in node_list]
        minind = dlist.index(min(dlist))
        return minind

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

old_dictionary = map_objects

map_objects = {"robot_pose": robot_pose}
map_objects.update(old_dictionary)

print(optimal_order)
optimal_order.insert(0, "robot_pose")
print(optimal_order)

# Plan the path for each pair of consecutive items in the optimal order
for i in range(len(optimal_order) - 1):
    start_pos = map_objects[optimal_order[i]]
    end_pos = map_objects[optimal_order[i + 1]]
    
    rrtc = RRTC(start=[start_pos['x'], start_pos['y']], 
                goal=[end_pos['x'], end_pos['y']], 
                obstacle_list=obstacle_positions)
    
    path = rrtc.planning()
    if path is None:
        print(f"Could not find a path between {optimal_order[i]} and {optimal_order[i + 1]}.")
    else:
        print(f"Path found between {optimal_order[i]} and {optimal_order[i + 1]}: {path}")