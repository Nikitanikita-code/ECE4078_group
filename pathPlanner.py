import numpy as np
import math
import sys

# map data
map_objects = {"aruco10_0": {"y": 0.93, "x": -0.28}, "aruco4_0": {"y": -1.21, "x": -0.84}, "aruco3_0": {"y": -1.17, "x": 0.31}, "aruco1_0": {"y": -0.78, "x": 0.7}, "aruco8_0": {"y": 0.34, "x": 0.9}, "aruco5_0": {"y": 0.46, "x": 0.23}, "aruco9_0": {"y": -0.37, "x": 1.17}, "aruco6_0": {"y": -0.55, "x": -1.0}, "aruco7_0": {"y": -0.8, "x": -0.3}, "aruco2_0": {"y": 0.28, "x": -0.64}, "lemon_0": {"y": -0.57, "x": -0.56}, "pear_0": {"y": -0.92, "x": 0.3}, "lime_0": {"y": 0.26, "x": -0.41}, "tomato_0": {"y": 1.17, "x": 1.08}, "capsicum_0": {"y": -0.71, "x": 1.38}, "potato_0": {"y": 0.87, "x": -0.19}, "pumpkin_0": {"y": -1.19, "x": 1.1}, "garlic_0": {"y": -0.02, "x": 1.04}, "capsicum_1": {"y": 0.04, "x": -0.91}, "tomato_1": {"y": -1.04, "x": -1.43}}

# shopping list
shopping_list = ["garlic_0", "lemon_0", "lime_0", "pear_0", "potato_0"]

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
                 expand_dis=0.7, 
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
        
        print(f"added new node is {added_new_node}")
        print(f"\t{new_node.x} \t {new_node.y}")

        if new_node.x > 1.3 or new_node.y > 1.3:
            added_new_node = False

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
            
            print(f"added sample is {added_samp}")
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

        dist = []
        radius = 0.15
        points = np.vstack((new_node.path_x, new_node.path_y)).T
        if self.obstacle_list != None:
            for obs in self.obstacle_list:
                for point in points:
                    dx = obs['x'] - point[0]
                    dy = obs['y'] - point[1]
        
                    dist.append(dx * dx + dy * dy)
                    
    
        return False  # safe

    
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

def path_plan(start_pos, end_pos, obstacle_list):
    # start_pos = [x1, y1]
    # end_pos = [x2, y2]
    # obstacle_list (example) = [{'y': 0.93, 'x': -0.28}, {'y': -1.21, 'x': -0.84}, {'y': -1.17, 'x': 0.31}]
    # returns path with way points: [[x1, y1], [a, b], [x2, y2]]

    rrtc = RRTC(start=start_pos, 
                goal=end_pos, 
                obstacle_list=obstacle_positions)
    path = rrtc.planning()

    if path[0] != start_pos:
        path.reverse()

    if path is None:
        print(f"Could not find a path between {optimal_order[i]} and {optimal_order[i + 1]}.")
    else:
        return path

old_dictionary = map_objects

map_objects = {"robot_pose": robot_pose}
map_objects.update(old_dictionary)

optimal_order.insert(0, "robot_pose")

full_path = []

start_pos = map_objects[optimal_order[0]]
end_pos = map_objects[optimal_order[1]]

path_chosen = path_plan([start_pos['x'], start_pos['y']], 
          [end_pos['x'], end_pos['y']],
          obstacle_list=obstacle_positions)

for i in range(20):
    path_curr = path_plan([start_pos['x'], start_pos['y']], 
          [end_pos['x'], end_pos['y']],
          obstacle_list=obstacle_positions)

    if len(path_curr) < len(path_chosen):
        path_chosen = path_curr

    
print(path_chosen)

start_pos = map_objects[optimal_order[1]]
end_pos = map_objects[optimal_order[2]]

path_chosen1 = path_plan([start_pos['x'], start_pos['y']], 
          [end_pos['x'], end_pos['y']],
          obstacle_list=obstacle_positions)

print(path_chosen1)

# Plan the path for each pair of consecutive items in the optimal order
for i in range(len(optimal_order) - 1):
    start_pos = map_objects[optimal_order[i]]
    end_pos = map_objects[optimal_order[i + 1]]
    
    path_ex = path_plan([start_pos['x'], start_pos['y']], 
          [end_pos['x'], end_pos['y']],
          obstacle_list=obstacle_positions)

    full_path = full_path + path_ex

    print('done')