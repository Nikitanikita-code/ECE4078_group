# from Practical04_Support.Obstacle import *
# from Practical04_Support.path_animation import *
import meshcat.geometry as g
import meshcat.transformations as tf



# from ece4078.Utility import StartMeshcat

import math

import numpy as np
import random
import os
import types

# Import dependencies and set random seed
seed_value = 5
# 1. Set `PYTHONHASHSEED` environment variable at a fixed value
os.environ['PYTHONHASHSEED'] = str(seed_value)
# 2. Set `python` built-in pseudo-random generator at a fixed value
random.seed(seed_value)
# 3. Set `numpy` pseudo-random generator at a fixed value
np.random.seed(seed_value)

import matplotlib.pyplot as plt

# vis = StartMeshcat()

# This is an adapted version of the RRT implementation done by Atsushi Sakai (@Atsushi_twi)
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
                 width = 1.5,
                 height=1.5,
                 expand_dis=1.0, 
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
        self.wayPointCount = 0
        self.discardedPoint = 0

    

    def grow_tree(self, tree, node):
        # Extend a tree towards a specified node, starting from the closest node in the tree,
        # and return a Bolean specifying whether we should add the specified node or not
        # `added_new_node` is the Boolean.
        # If you plan on appending a new element to the tree, you should do that inside this function
        
        #TODO: Complete the method -------------------------------------------------------------------------
        # Extend the tree
        # print("\n======in grow tree")
        added_new_node = False 
        
        nearestNode = tree[self.get_nearest_node_index(tree, node)]

        pathToNode = self.steer(nearestNode,node, self.expand_dis)

        #cehcking for expansion distance 
        d,_ = self.calc_distance_and_angle(node,nearestNode)
        # if(abs(d) < self.expand_dis):
            
        # print(f"adding new node with distance {d} as it's < {self.expand_dis}")
        #checking collisions
        if(self.is_collision_free(pathToNode)):
            added_new_node = True
        
        # print(f"======growing tree")
        # print(f"nearest node at {nearestNode.x} and {nearestNode.y}")
        # print(f"new node at {node.x} and {node.y}")
        
        globalX = nearestNode.x + node.x
        globalY = nearestNode.y + node.y
        
        # print(f"\t=====point to connect is {globalX},{globalY}\n")
        
        if(abs(globalX) >= 1.5 or abs(globalY) >= 1.5):
            # print(f"out of bond, rejected [{globalX},{globalY}]")
            # input()
            
            self.discardedPoint += 1
            # print(f"point discarded counter is at {self.discardedPoint}")
            if(self.discardedPoint > 1500):
                print("\n\n=====NO PATH FOUND DISCARD POINT MAXED=====\n\t New goal point")
                
                return None
                
            
            added_new_node = False        
        else:
            # print(f"adding {globalX},{globalY}")
            
            self.wayPointCount += 1 
            if(self.wayPointCount > 300):
                # input(f"way point counter is now at {self.wayPointCount}")
                print("\n\n=====NO PATH FOUND WAY POINT MAXED=====\n\t New goal point")
                return None
            
        # # Check if we should add this node or not, and add it to the tree
        
        # #ENDTODO ----------------------------------------------------------------------------------------------
        if added_new_node:
            tree.append(pathToNode)
        
        return added_new_node

    def check_trees_distance(self):
        # Find the distance between the trees, return if the trees distance is smaller than self.expand_dis
        # In other word, we are checking if we can connect the 2 trees.
        
        #TODO: Complete the method -------------------------------------------------------------------------
        can_be_connected = False
        #below
        nearestNode = self.end_node_list[self.get_nearest_node_index(self.end_node_list, self.start_node_list[-1])]
        #above
        d,_ = self.calc_distance_and_angle( self.start_node_list[-1],nearestNode)
        if(abs(d) <= self.expand_dis):
            # print(f"\t========the distance is {d}, it can be connected")
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
            
        #TODO: Complete the planning method ----------------------------------------------------------------
            
            # 1. Sample and add a node in the start tree
            # Hint: You should use self.grow_tree above to add a node in the start tree here
            rndNode = self.get_random_node()
            
            growTreeResult = self.grow_tree(self.start_node_list , rndNode)
            
            if(growTreeResult == True):
                    
                    # 2. Check whether trees can be connected
                    # Hint: You should use self.check_trees_distance above to check.
                
                if(self.check_trees_distance()):
                    nearestNode = self.end_node_list[self.get_nearest_node_index(self.end_node_list, self.start_node_list[-1])]

                # 3. Add the node that connects the trees and generate the path
                    finalNode = self.steer(nearestNode, self.start_node_list[-1], self.expand_dis)
                    
                    if(self.is_collision_free(finalNode)):
                        self.end_node_list.append(finalNode)
                    # Note: It is important that you return path found as:
                        return self.generate_final_course(len(self.start_node_list) - 1, len(self.end_node_list) - 1)

                    
                    else:
                        self.grow_tree(self.end_node_list , self.get_random_node())

                
                # 4. Sample and add a node in the end tree
                else:
                    self.grow_tree(self.end_node_list , self.get_random_node())
                
                # 5. Swap start and end trees

                self.start_node_list, self.end_node_list = self.end_node_list, self.start_node_list
            elif(growTreeResult == None):
                return None
            

        #ENDTODO ----------------------------------------------------------------------------------------------
            
        return None  # cannot find path
    
    # ------------------------------DO NOT change helper methods below ----------------------------
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
        """
        Determine if nearby_node (new_node) is in the collision-free space.
        """
        if new_node is None:
            return True
        
        points = np.vstack((new_node.path_x, new_node.path_y)).T
        for obs in self.obstacle_list:
            in_collision,collisionPoint = obs.is_in_collision_with_points(points)
            # in_collision = obs.is_in_collision_with_points(points)
            if in_collision:
                return False
        
        return True  # safe
    
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
        
        if(path[0][0]!=self.start.x or path[0][1]!= self.start.y):
            path.reverse()
            
        
        # print(f"path is {path}")
        # input()
        
        # input(f"last node is {path[0]}, second last node is {path[1]}")
        distanceFromGoal = 0.3 #!!!!ITS A FACTOR NOT ACTAL DISTANCE
        
 
        
        # adjestedX,adjestedY = self.find_point_between([path[0][0],path[0][1]], [path[1][0], path[1][1]])
        endPoint0 = path[-1]
        endPoint1 = path[len(path)-2]
        # print(f"end point 0 {endPoint0} end point 1 = {endPoint1}")
        
        if(self.calc_dist_to_goal(endPoint1[0],endPoint1[1]) < 0.3 ):
            adjestedPath = endPoint1
        else:
        
            adjestedX = endPoint0[0] + distanceFromGoal*(endPoint1[0]-endPoint0[0])
            adjestedY = endPoint0[1] + distanceFromGoal*(endPoint1[1]-endPoint0[1])
            
            adjestedPath = [adjestedX,adjestedY]
            
        path[-1] = adjestedPath
            
        # print(f"adjested path is {path}")
        # input()

        
        return path


    def is_at_target(self, new_node,target):
        """
        Determine if nearby_node (new_node) is in the collision-free space.
        """
        if new_node is None:
            return True
        
        # print(f"new node is {new_node.path_x}")
        # input()
        
        points = np.vstack((new_node.path_x, new_node.path_y)).T
        
        # print(points)
        
        inCollision,collisionPoint = target.is_in_collision_with_points(points)
        
        print(collisionPoint)
        input("above point is at target")
        
        if( inCollision == True):
            return True

        
        return False  




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
    

import matplotlib.pyplot as plt

def plot_single_path_and_obstacles(path, obstacles, start, goal):
    """
    Plots a path and obstacles on the same graph.

    Parameters:
    - path: A list of coordinates [[x1, y1], [x2, y2], ...] representing the path.
    - obstacles: A list of tuples [(x_center, y_center, radius), ...] representing obstacles as circles.
    """
    # Create a figure and axis
    fig, ax = plt.subplots(figsize=(10,10))
    
    square = plt.Rectangle((-1.5, -1.5), 3, 3, fill=False, color='red', label="3x3 Box at Origin")
    plt.gca().add_patch(square)
    

    # Extract x and y coordinates from the path
    x_coords = [point[0] for point in path]
    y_coords = [point[1] for point in path]

    # Plot the path as a line connecting the points
    
    ax.plot(x_coords, y_coords, color='blue', marker='o', linestyle='-', linewidth=1, label='Path')

    # Plot each obstacle as a circle
    for i in range(len(obstacles)):
        
        obs = obstacles[i]
        obsName = obs.name.split('_')[0]
        if obsName[0:5] == 'aruco':
            shape = plt.Rectangle(obs.center, 0.08,0.08, color='blue', fill=False, linewidth=1, label='1')
            ax.text(obs.center[0]+0.05,obs.center[1], f'{obsName[5]}', color='black', ha='center',fontsize=6)
        else:
            shape = plt.Circle(obs.center, obs.radius, color='red', fill=False, linewidth=1, label='1')
            ax.text(obs.center[0],obs.center[1], f'{obs.name}', color='black', ha='center',fontsize=8)
        ax.add_patch(shape)
        
    startCircle = plt.Circle(start,radius=0.05 ,color='purple', fill=False, linewidth=1, label='1')
    ax.add_patch(startCircle)
    goalCircle = plt.Circle(goal,radius=0.5 ,color='green', fill=False, linewidth=1, label='1')
    ax.add_patch(goalCircle)
    
    # Set the aspect of the plot to 'equal' to ensure shapes aren't distorted
    ax.set_aspect('equal', 'box')

    # Set the plot limits (adjust as needed based on your data)
    ax.set_xlim(-3.2,3.2)
    ax.set_ylim(-3.2,3.2)

    # Add labels and a title
    plt.title('Path and Obstacles')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')

    # Show the plot
    plt.show()
    
def plot_all_path_and_obstacles(pathList, obstacles, start, goal,listOfName,display=True):
    """
    Plots a path and obstacles on the same graph.

    Parameters:
    - path: A list of coordinates [[x1, y1], [x2, y2], ...] representing the path.
    - obstacles: A list of tuples [(x_center, y_center, radius), ...] representing obstacles as circles.
    """
    # Create a figure and axis
    fig, ax = plt.subplots(figsize=(10,10))
    
    square = plt.Rectangle((-1.5, -1.5), 3, 3, fill=False, color='red', label="3x3 Box at Origin")
    plt.gca().add_patch(square)
    ax.legend("ARENA")
    

    # Extract x and y coordinates from the path
    num=0
    for path in pathList:
        
        x_coords = [point[0] for point in path]
        y_coords = [point[1] for point in path]

        # Plot the path as a line connecting the points
        
        ax.plot(x_coords, y_coords, marker='o', linestyle='-', linewidth=1, label=f'{listOfName[num]}')

        num+=1
    ax.legend()

    # Plot each obstacle as a circle
    for i in range(len(obstacles)):
        
        obs = obstacles[i]
        obsName = obs.name.split('_')[0]
        if obsName[0:5] == 'aruco':
            shape = plt.Rectangle(obs.center, 0.08,0.08, color='blue', fill=False, linewidth=1, label='1')
            ax.text(obs.center[0]+0.05,obs.center[1], f'{obsName[5]}', color='black', ha='center',fontsize=6)
        else:
            shape = plt.Circle(obs.center, obs.radius, color='red', fill=False, linewidth=1, label='1')
            ax.text(obs.center[0],obs.center[1], f'{obs.name}', color='black', ha='center',fontsize=8)
        ax.add_patch(shape)
        
    startCircle = plt.Circle(start,radius=0.05 ,color='purple', fill=False, linewidth=1, label='1')
    ax.add_patch(startCircle)
    goalCircle = plt.Circle(goal,radius=0.5 ,color='green', fill=False, linewidth=1, label='1')
    ax.add_patch(goalCircle)
    
    # Set the aspect of the plot to 'equal' to ensure shapes aren't distorted
    ax.set_aspect('equal', 'box')

    # Set the plot limits (adjust as needed based on your data)
    ax.set_xlim(-3.2,3.2)
    ax.set_ylim(-3.2,3.2)

    # Add labels and a title
    plt.title('Path and Obstacles')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')

    # Show the plot
    
    
    
    if display:
        plt.show()
    else:
        plt.savefig('PATH_GRAPH/path_plot.png',format = 'png')
        


from Obstacle import *

# Example usage
path = [[0, 0], [2, 2], [4, 5], [8, 8]]  # Path from point to point
obstacles = [
    (3, 3, 1),   # Obstacle 1: circle centered at (3, 3) with radius 1
    (6, 6, 2)    # Obstacle 2: circle centered at (6, 6) with radius 2
]




    #Set parameters


map_objects = {"aruco10_0": {"y": -0.7909090909090909, "x": 0.8909090909090909}, "aruco4_0": {"y": 1.1818181818181819, "x": 0.9636363636363636}, "aruco3_0": {"y": -1.2272727272727273, "x": 1.209090909090909}, "aruco1_0": {"y": -0.00909090909090909, "x": 1.2363636363636363}, "aruco8_0": {"y": 1.309090909090909, "x": -0.5363636363636364}, "aruco5_0": {"y": 0.7636363636363637, "x": 0.7363636363636363}, "aruco9_0": {"y": -1.1545454545454545, "x": -1.0909090909090908}, "aruco6_0": {"y": 0.2, "x": -0.5727272727272728}, "aruco7_0": {"y": -0.01818181818181818, "x": -1.2545454545454546}, "aruco2_0": {"y": -1.0545454545454545, "x": -0.16363636363636364}, "lemon_0": {"y": -1.0363636363636364, "x": 0.14545454545454545}, "pear_0": {"y": 0.6909090909090909, "x": 1.1363636363636365}, "lime_0": {"y": 1.0454545454545454, "x": -0.13636363636363635}, "tomato_0": {"y": -0.00909090909090909, "x": 0.8454545454545455}, "capsicum_0": {"y": 0.2, "x": -0.8545454545454545}, "potato_0": {"y": 0.7363636363636363, "x": 0.12727272727272726}, "pumpkin_0": {"y": 0.6, "x": -1.1818181818181819}, "garlic_0": {"y": -0.6, "x": -0.9363636363636364}, "potato_1": {"y": -0.7181818181818181, "x": -0.35454545454545455}, "capsicum_1": {"y": -0.4727272727272727, "x": 1.009090909090909}}


shopping_list = ["lemon_0","garlic_0","lime_0","pumpkin_0","pear_0"]


# need to modify this to come up with optimal order (using spanning tree or tsp, check messenger chat)

obstacleList = []
shoppingPositions = []

for key,value in map_objects.items():

    xCoord = value.get('x')
    yCoord = value.get('y')
    
    if key not in shopping_list:
        print(f"===== OBSTACLE : the key is {key} and value is {value}")
        radius = 0.1
        
        obstacleList.append(Circle(xCoord, yCoord, radius=radius, name=key))
    
for item in shopping_list:
    itemToAdd = map_objects.get(item)
    
    xCoord = itemToAdd.get('x')
    yCoord = itemToAdd.get('y')
    radius = 0.1
    shoppingPositions.append(Circle(xCoord, yCoord, radius=radius, name=item))
        
startObject = {'y': 0, 'x': 0}
goalObject = {'x': 1.4, 'y': 1.4}


goal = np.array([goalObject.get('x'), goalObject.get('y')])
start = np.array([startObject.get('x'), startObject.get('y')])

allPath = []

for item in shoppingPositions:
    print(item.name)



#####uncommoent below 

for shoppingItem in shoppingPositions:
    allObstacle = []
    allObstacle.extend(obstacleList)
    print(f"\n========Now getting {shoppingItem.name} at [{shoppingItem.center}]")
    print(f"\tstart: [{start}]")
    goal = shoppingItem.center
    
    
    
    #++++TODO
    for i in shoppingPositions:
        # allObstacle.append(i)
        if shoppingItem != i:
            allObstacle.append(i)
        
                #++++TODO
            
    
    
    plannedPath =None
    targetGoal = goal
    
    expandDistance = 1
    
    while plannedPath == None:
    
        rrtc = RRTC(start=start, goal=targetGoal, width=3, height=3, obstacle_list=allObstacle,
                expand_dis=expandDistance, path_resolution=0.01) #expend dis was 0.9
        # rrtc = RRTC(start=start, goal=goal, width=3, height=3, obstacle_list=allObstacle,
        #         expand_dis=0.4, path_resolution=0.0001)
        
        plannedPath = rrtc.planning()
        
        targetGoalInArea = True
        
        newRandomX = random.uniform(-0.2, 0.2)
        newRandomY = random.uniform(-0.2, 0.2)
        targetGoal = [goal[0] + newRandomX ,  goal[1] + newRandomY]
        
        if(abs(targetGoal[0]) > 1.5 or abs(targetGoal[1]) > 1.5):
            targetGoalInArea = False
        
        
        while targetGoalInArea == False:
            newRandomX = random.uniform(-0.3, 0.3)
            newRandomY = random.uniform(-0.3, 0.3)
            targetGoal = [goal[0] + newRandomX ,  goal[1] + newRandomY]
            
            if(abs(targetGoal[0]) > 1.5 or abs(targetGoal[1]) > 1.5):
                targetGoalInArea = False
            else:
                targetGoalInArea = True
                
        if expandDistance > 0.5:
            expandDistance -= 0.1
            print("\n====changed expand distance")
        
        # print(f"===============updated target goat as {targetGoal}")
        # input("hereee")
        
        
    
    
    allPath.append(plannedPath)
    
    # plot_single_path_and_obstacles(plannedPath,allObstacle,start, goal)
    start = plannedPath[-1]
    
    # input(f"\t==== path is \n{plannedPath}")
    
allObstacle.append(shoppingItem)
    
    
print("\n\n===== the full planned path is =======")
print(f"\t\t {allPath}")


plot_all_path_and_obstacles(allPath,allObstacle,[0,0],[2,2],shopping_list)



# checkConsistance = []
# for i in range(5):
#     start = [0,0]
#     goal = [1.0,-1.3]
    
#     rrtc = RRTC(start=start, goal=goal, width=3, height=3, obstacle_list=allObstacle,
#             expand_dis=0.6, path_resolution=0.001) #expend dis was 0.9
#     # rrtc = RRTC(start=start, goal=goal, width=3, height=3, obstacle_list=allObstacle,
#     #         expand_dis=0.4, path_resolution=0.0001)
#     plannedPath = rrtc.planning()
    
#     checkConsistance.append(plannedPath)
    

# plot_all_path_and_obstacles(checkConsistance,allObstacle,[0,0],[2,2])  
    
    

# startObject = {'y': 0, 'x': 0}
# goalObject =  {"x": -0.6, "y": 0}


# goal = np.array([goalObject.get('x'), goalObject.get('y')])
# start = np.array([startObject.get('x'), startObject.get('y')])
    
# for i in allObstacle:
#     if

# rrtc = RRTC(start=start, goal=goal, width=3, height=3, obstacle_list=allObstacle,
#           expand_dis=0.5, path_resolution=0.01)

#expanddis =0.1
# pathres = 0.01



# path = rrtc.planning()

# plot_single_path_and_obstacles(path,allObstacle,start, goal)

# plotPath(path,all_obstacles)

# print(path)






