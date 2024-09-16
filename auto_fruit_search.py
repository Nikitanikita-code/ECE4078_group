# M4 - Autonomous fruit searching

# basic python packages
import sys, os
import cv2
import numpy as np
import json
import argparse
import time
import pandas





import random

# import SLAM components
sys.path.insert(0, "{}/slam".format(os.getcwd()))
from ekfNoLM import EKF
from slam.robot import Robot
import slam.aruco_detector as aruco


# import utility functions
sys.path.insert(0, "util")
from util.pibot import PenguinPi
import util.measure as measure


from YOLO.detector import Detector
import pathPlannerV2
from Obstacle import *
from math_functions import *



from scipy.cluster.hierarchy import linkage, fcluster


def plan_new_path(trueMap, shoppingList,currentPose):
    
    
    
    obstacleList = []
    shoppingPositions = []


    for key,value in trueMap.items():

        xCoord = value.get('x')
        yCoord = value.get('y')
        
        if key not in shoppingList:
            print(f"===== OBSTACLE : the key is {key} and value is {value}")
            radius = 0.1
            
            obstacleList.append(Circle(xCoord, yCoord, radius=radius, name=key))
        
    for item in shoppingList:
        itemToAdd = trueMap.get(item)
        
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
        
            rrtc = pathPlannerV2.RRTC(start=start, goal=targetGoal, width=3, height=3, obstacle_list=allObstacle,
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
    
    pathPlannerV2.plot_all_path_and_obstacles(allPath,allObstacle,[0,0],[2,2],shoppingList,False)
    
    return allPath


    
        


def read_true_map(fname):
    """Read the ground truth map and output the pose of the ArUco markers and 5 target fruits&vegs to search for

    @param fname: filename of the map
    @return:
        1) list of targets, e.g. ['lemon', 'tomato', 'garlic']
        2) locations of the targets, [[x1, y1], ..... [xn, yn]]
        3) locations of ArUco markers in order, i.e. pos[9, :] = position of the aruco10_0 marker
    """
    
    print(f"fname us {fname}")
    # input()
    with open(fname, 'r') as fd:
        gt_dict = json.load(fd)

        return gt_dict


def read_search_list():
    """Read the search order of the target fruits

    @return: search order of the target fruits
    """
    search_list = []
    with open('M4_prac_shopping_list.txt', 'r') as fd:
        fruits = fd.readlines()

        for fruit in fruits:
            search_list.append(fruit.strip())

    return search_list


def print_target_fruits_pos(search_list, fruit_list, fruit_true_pos):
    """Print out the target fruits' pos in the search order

    @param search_list: search order of the fruits
    @param fruit_list: list of target fruits
    @param fruit_true_pos: positions of the target fruits
    """

    print("Search order:")
    n_fruit = 1
    for fruit in search_list:
        for i in range(len(fruit_list)): # there are 5 targets amongst 10 objects
            if fruit == fruit_list[i]:
                print('{}) {} at [{}, {}]'.format(n_fruit,
                                                  fruit,
                                                  np.round(fruit_true_pos[i][0], 1),
                                                  np.round(fruit_true_pos[i][1], 1)))
        n_fruit += 1


def turnDegree(degreeToTurn):
    travelVel = 50
    parameter = 0.1
    
    ppi.set_velocity([1,0], tick = travelVel, time = degreeToTurn/parameter)
    
    
def get_distance_robot_to_goal(robot_state=np.zeros(3), goal=np.zeros(3)):
	"""
	Compute Euclidean distance between the robot and the goal location
	:param robot_state: 3D vector (x, y, theta) representing the current state of the robot
	:param goal: 3D Cartesian coordinates of goal location
	"""

	if goal.shape[0] < 3:
		goal = np.hstack((goal, np.array([0])))

	x_goal, y_goal,_ = goal
	x, y,_ = robot_state
	x_diff = x_goal - x
	y_diff = y_goal - y

	rho = np.hypot(x_diff, y_diff)

	return rho
    
def get_angle_robot_to_goal(robot_state, goal):
    if goal.shape[0] < 3:
        goal = np.hstack((goal, np.array([0])))

    x_goal, y_goal,_ = goal
    
    # print(f"getting heading... \n\tcurrent robot heading is {robot_state[2] * (180/np.pi)}")
    

    x, y, theta = robot_state
    x_diff = x_goal - x
    y_diff = y_goal - y

    alpha = clamp_angle(np.arctan2(y_diff, x_diff) - theta)
    
    print(f"the new heading is {alpha * (180/np.pi)}")
    # input()
    return alpha

def clamp_angle(rad_angle=0, min_value=-np.pi, max_value=np.pi):
	"""
	Restrict angle to the range [min, max]
	:param rad_angle: angle in radians
	:param min_value: min angle value
	:param max_value: max angle value
	"""

	if min_value > 0:
		min_value *= -1

	angle = (rad_angle + max_value) % (2 * np.pi) + min_value

	return angle




# Waypoint navigation
# the robot automatically drives to a given [x,y] coordinate
# note that this function requires your camera and wheel calibration parameters from M2, and the "util" folder from M1
# fully automatic navigation:
# try developing a path-finding algorithm that produces the waypoints automatically
def drive_to_point(waypoint, robot_pose):
    # control_clock = time.time()
    # imports camera / wheel calibration parameters 
    
    #it should look for landmarks when running to see if there's anything on the way
    #   if there's something in the way,stop the robot 
    
    fileS = "calibration/param/scale.txt"
    scale = np.loadtxt(fileS, delimiter=',')
    fileB = "calibration/param/baseline.txt"
    baseline = np.loadtxt(fileB, delimiter=',')
    
    ####################################################
    # TODO: replace with your codes to make the robot drive to the waypoint


    #robot state is [x,y,th] in the gobal sense
    
    wayPointGobal = waypoint + robot_pose.flatten()[:-1] #the distance of the waypoint w.r.t to gobal robot state

    #finding the angle to turn
    heading = get_angle_robot_to_goal(robot_pose,np.array(waypoint))
    distanceFromGoal = get_distance_robot_to_goal(robot_pose , np.array(waypoint))
    

    wheel_vel = 30 # tick
    
    # wheel_vel = 70
    
    # turn towards the waypoint
    print(f"\n===== heading is =======\n\t{heading*(180/np.pi)}")
    print(f"\n===== distance is =======\n\t{distanceFromGoal}")
    # input(f"from [{robot_pose[0]},{robot_pose[1]}] to {waypoint}")
    
    #parameter to be tuned with floor
    timeForFullTurn = 5.8-0.05
    timeFor1Meter = 12.3
    dt = 0.1
    
    lv = 0
    rv = 0
    
    # heading = robot.state[2] - heading #!!!!
    
    turn_time = abs((heading)/(2*np.pi) * (timeForFullTurn) )# replace with your calculation
    drive_time = abs(distanceFromGoal*timeFor1Meter)

        
    if(heading>0):
        lv,rv = ppi.set_velocity([0, 1], turning_tick=wheel_vel)
    else:
        lv,rv = ppi.set_velocity([0, -1], turning_tick=wheel_vel)
        
    
    startTime = time.time()
    controlTime = time.time()
    
    LMcollusion = False
    
    previousHeading=  robot.state[2]
    
    while time.time() -startTime <turn_time:
        LMcollusion,lmID,landmarkPosition = lookForLM()
        # drive_meas = control(currentCommand,wheel_vel,controlTime,lv,rv)
        # controlTime = time.time()
        # update_slam(drive_meas,landmarkPosition)
        update_robot_state(controlTime,lv,rv,landmarkPosition)
        controlTime = time.time()
        
    
        
        
    ppi.set_velocity([0, 0], tick=wheel_vel) #stopping the robot
    
    # after turning, drive straight to the waypoint

    
    lv,rv = ppi.set_velocity([1, 0], tick=wheel_vel) #start driving foward

    startTime = time.time()
    controlTime = time.time()
    
    while time.time()-startTime < drive_time:
        LMcollusion,lmID,landmarkPosition = lookForLM() #checking to see if there's landmark in the way 
        if LMcollusion == True:
            print(f"=======STOPPING======\n\t LM{lmID} is below 0.1m")
            break
        
        update_robot_state(controlTime,lv,rv,landmarkPosition)
        controlTime = time.time()
    
    ppi.set_velocity([0, 0], tick=wheel_vel)#stopping the robot
    
    print("Arrived at [{}, {}]".format(waypoint[0], waypoint[1]))
    
    print(f"robot state is {robot.state}")



def update_robot_state(controlTime, lv, rv, landmarkPosition):
    drive_meas = control(controlTime,lv,rv)
    update_slam(drive_meas,landmarkPosition)
    
   
def update_slam(drive_meas, lms):
    
    ekf.predict(drive_meas)
    ekf.add_landmarks(lms)
    ekf.update(lms)
    
def control(control_clock,lv,rv):
    # lv,rv = ppi.set_velocity(driveCommand, wheel_vel)
    dt = time.time() - control_clock
    drive_meas = measure.Drive(lv,-rv,dt)
    control_clock = time.time()
    
    return drive_meas
    
        
        
def lookForLM():
    minDisFromLM = 0.05
    
    image = ppi.get_image()
    
    landmarkPosition, _ = aruco_det.detect_marker_positions(image)
    
    for landmark in landmarkPosition:    
        print(f"==========LM {landmark.tag} \n\t {landmark.position[0]} {landmark.position[1]}\n\n")
        
        prepDis = np.sqrt((landmark.position[0])**2+ (landmark.position[1])**2)
        
        if prepDis <  minDisFromLM:
            return True,landmark.tag, landmarkPosition
    
    # if len(landmarkPosition) !=0:
        
    #     input("see an LM")
    
    return False,-1,landmarkPosition
    
        
def detectImage(img):
    landmarkPosition, _ = aruco_det.detect_marker_positions(img)
    
    for landmark in landmarkPosition:    
        print(f"==========LM {landmark.tag} \n\t {landmark.position[0]} {landmark.position[1]}\n\n")
    


def get_robot_pose():
    ####################################################
    # TODO: replace with your codes to estimate the pose of the robot
    # We STRONGLY RECOMMEND you to use your SLAM code from M2 here
    
    robot_pose = robot.state
    
    #capture a picture
    image = ppi.get_image()
    
    landmarkPosition, _ = aruco_det.detect_marker_positions(image)
    
    for landmark in landmarkPosition:    
        print(f"==========LM {landmark.tag} \n\t {landmark.position[0]} {landmark.position[1]}\n\n")
        
        LMGobal = ekf.compute_gobal_marker(landmark)
        
        print(f"\t it's gobal position is {LMGobal}")
        
        
        

    # update the robot pose [x,y,theta]
    # robot_pose = [0.0,0.0,0.0] # replace with your calculation
    ####################################################

    return robot_pose



def estimate_pose(camera_matrix, obj_info, robot_pose):
    """
    function:
        estimate the pose of a target based on size and location of its bounding box and the corresponding robot pose
    input:
        camera_matrix: list, the intrinsic matrix computed from camera calibration (read from 'param/intrinsic.txt')
            |f_x, s,   c_x|
            |0,   f_y, c_y|
            |0,   0,   1  |
            (f_x, f_y): focal length in pixels
            (c_x, c_y): optical centre in pixels
            s: skew coefficient (should be 0 for PenguinPi)
        obj_info: list, an individual bounding box in an image (generated by get_bounding_box, [label,[x,y,width,height]])
        robot_pose: list, pose of robot corresponding to the image (read from 'lab_output/images.txt', [x,y,theta])
    output:
        target_pose: dict, prediction of target pose
    """
    # read in camera matrix (from camera calibration results)
    focal_length = camera_matrix[0][0]

    # there are 8 possible types of fruits and vegs
    ######### Replace with your codes #########
    # TODO: measure actual sizes of targets [width, depth, height] and update the dictionary of true target dimensions
    target_dimensions_dict = {'pear': [156,151,0.0914], 'lemon': [1.0,1.0,0.0527], 
                              'lime': [1.0,1.0,0.053], 'tomato': [1.0,1.0,0.058], 
                              'capsicum': [1.0,1.0,0.0963], 'potato': [1.0,1.0,0.0563], 
                              'pumpkin': [1.0,1.0,0.074], 'garlic': [1.0,1.0,0.0713],'orange': [1, 1, 79]}
    #########

    # estimate target pose using bounding box and robot pose
    target_class = obj_info[0]     # get predicted target label of the box
    target_box = obj_info[1]       # get bounding box measures: [x,y,width,height]
    true_height = target_dimensions_dict[target_class][2]   # look up true height of by class label

    # compute pose of the target based on bounding box info, true object height, and robot's pose
    pixel_height = target_box[3]
    pixel_center = target_box[0]
    distance = true_height/pixel_height * focal_length  # estimated distance between the robot and the centre of the image plane based on height
    # training image size 320x240p
    image_width = 320 # change this if your training image is in a different size (check details of pred_0.png taken by your robot)
    x_shift = image_width/2 - pixel_center              # x distance between bounding box centre and centreline in camera view
    theta = np.arctan(x_shift/focal_length)     # angle of object relative to the robot
    ang = theta + robot_pose[2]     # angle of object in the world frame
    
   # relative object location
    distance_obj = distance/np.cos(theta) # relative distance between robot and object
    x_relative = distance_obj * np.cos(theta) # relative x pose
    y_relative = distance_obj * np.sin(theta) # relative y pose
    relative_pose = {'x': x_relative, 'y': y_relative}
    #print(f'relative_pose: {relative_pose}')

    # location of object in the world frame using rotation matrix
    delta_x_world = x_relative * np.cos(robot_pose[2]) - y_relative * np.sin(robot_pose[2])
    delta_y_world = x_relative * np.sin(robot_pose[2]) + y_relative * np.cos(robot_pose[2])
    # add robot pose with delta target pose
    target_pose = {'y': (robot_pose[1]+delta_y_world)[0],
                   'x': (robot_pose[0]+delta_x_world)[0]}
    #print(f'delta_x_world: {delta_x_world}, delta_y_world: {delta_y_world}')
    #print(f'target_pose: {target_pose}')

    return target_pose
def merge_estimations(target_pose_dict):
    """
    function:
        merge estimations of the same target
    input:math.dist
        target_pose_dict: dict, generated by estimate_pose
    output:
        target_est: dict, target pose estimations after merging
    """
    # x_values=[]
    # y_values=[]
    target_est = {}
    sorted_target_data ={}
    amount={}
    ######### Replace with your codes #########
    # TODO: replace it with a solution to merge the multiple occurrences of the same class type (e.g., by a distance threshold)
    # print(target_pose_dict)
    for key, value in target_pose_dict.items():
    # Split the key by the underscore to separate detection type and occurrence
        detection, occurrence = key.rsplit('_', 1)
        # print(detection)
        if detection not in sorted_target_data:
            sorted_target_data[detection] = []
    # Add the detection to the corresponding list in individual_detections
        sorted_target_data[detection]=np.append(sorted_target_data[detection],value)
        amount[detection]=len(sorted_target_data[detection])
        
    #print(sorted_target_data)
    # print(sorted_target_data[detection])
    
    for detection in sorted_target_data:
        if amount[detection]<2:
            
            target_est[detection+'_'+str(0)]=sorted_target_data[detection][0]
        else:
            numeric_data = [[d['y'], d['x']] for d in sorted_target_data[detection]]
            #print((numeric_data[0]))
            
            groups = linkage(numeric_data, 'single', 'euclidean')
            
            clusters = fcluster(groups, 1, criterion='distance')
            # n = 2 if len(sorted_target_data[detection]) > 1 else len(sorted_target_data[detection])
            # clustered = KMeans(init="k-means++",n_clusters=n,random_state=0)
            #estimated = clustered.fit_predict(numeric_data)
            #print(clusters)
            # print((numeric_data))
            x_values=[]
            y_values=[]
            for cls in np.unique(clusters):
                # if cls>1:
                #     target_index=np.where(clusters==cls)
                #     sorted_target_data[detection] = np.delete(sorted_target_data[detection], target_index, axis=0)
                # else:
                target_index=np.where(clusters==cls)
                coords_cls=sorted_target_data[detection][target_index]
                print(coords_cls)   
                for i in range(len(coords_cls)):
                    # print(sorted_target_data[detection][i])
                    
                    #print(coords['x'])
                    x_values.append(coords_cls[i]['x'])
                    y_values.append(coords_cls[i]['y'])
                    
                # print(sorted_target_data)
                x_med=np.mean(x_values)
                # print(detection)
                # print(x_values)
                #print(x_values)
                y_med=np.mean(y_values)
                target_est[detection+'_'+str(cls-1)] = {'x': x_med, 'y':y_med}

            #print( x_values)
    #########
   
    return target_est
def update_map(map,target_est):
    fruit={k:map[k] for k in list(map.keys())[10:]}
    print(fruit)
    list_of_known=[]
    list_keys=[]
    pos_known=[]
    for key, value in fruit.items():# need to put this outsde the loop
        detection, occurrence = key.rsplit('_', 1)
        list_of_known.append(detection)
        list_keys.append(key)
        pos_known.append(value)
    for key, value in target_est.items():
    # Split the key by the underscore to separate detection type and occurrence
        detection, occurrence = key.rsplit('_', 1)
        occurrence=int(occurrence)
        if detection not in list_of_known:
            map[key]=target_est[key]
        elif key not in list_keys and math.dist([target_est[key]['x'],target_est[key]['y']],[fruit[detection+'_'+str(occurrence-1)]['x'],fruit[detection+'_'+str(occurrence-1)]['y']])>0.5 and occurrence<2:
            map[key]=target_est[key]#dist thresh this also dont use key use det occcurence+1 also think edge cases if tomato_1
        elif key not in list_keys and math.dist([target_est[detection+'_'+str(occurrence-1)]['x'],target_est[detection+'_'+str(occurrence-1)]['y']],[fruit[detection+'_'+str(occurrence-1)]['x'],fruit[detection+'_'+str(occurrence-1)]['y']])>0.5 and occurrence<2:
            map[key]=target_est[detection+'_'+str(occurrence-1)]
    return map
def create_target_dict():
    image_taken=ppi.get_image()
    # input_image = cv2.imread(image_taken)
    states=[robot.state[0],robot.state[1],robot.state[2]]
    bounding_boxes, bbox_img = yolo.detect_single_image(image_taken)
    for detection in bounding_boxes:
        # count the occurrence of each target type
        occurrence = detected_type_list.count(detection[0])
        target_pose_dict[f'{detection[0]}_{occurrence}'] = estimate_pose(camera_matrix, detection, states)

        detected_type_list.append(detection[0])
    return target_pose_dict


# main loop
if __name__ == "__main__":
    parser = argparse.ArgumentParser("Fruit searching")
    parser.add_argument("--map", type=str, default='M4_prac_map_full.txt') # change to 'M4_true_map_part.txt' for lv2&3
    parser.add_argument("--ip", metavar='', type=str, default='192.168.50.1')
    parser.add_argument("--port", metavar='', type=int, default=8080)
    args, _ = parser.parse_known_args()
    
    
    fileS = "calibration/param/scale.txt"
    scale = np.loadtxt(fileS, delimiter=',')
    fileB = "calibration/param/baseline.txt"
    baseline = np.loadtxt(fileB, delimiter=',')
    fileC = "calibration/param/intrinsic.txt"
    camera_matrix = np.loadtxt(fileC, delimiter=',')
    fileD = "calibration/param/distCoeffs.txt"
    dist_coeffs = np.loadtxt(fileD, delimiter=',')
    
    #YOLO broken code
    script_dir = os.path.dirname(os.path.abspath(__file__))     # get current script directory (TargetPoseEst.py)
    
    model_path = f'{script_dir}/YOLO/model/best.pt'
    yolo = Detector(model_path)
    
    #get rid of above 

    #TODO uncomment below
    ppi = PenguinPi(args.ip,args.port)
    ppi.set_velocity([0, 0], tick= 30)
    robot = Robot(baseline,scale,camera_matrix,dist_coeffs)
    ekf = EKF(robot)
    
    # pathPlanner = pathPlannerV2()
    

    control_clock = time.time()
    
    aruco_det = aruco.aruco_detector(robot,marker_length=0.07) #initalise the aruco detector

    # read in the true map
    
    trueMap = read_true_map(args.map)
    search_list = read_search_list()
    
    
    
    
    #TODO uncomment below 
    
    
    # print_target_fruits_pos(search_list, fruits_list, fruits_true_pos)

    print(f"=========== search list=======\n\t{trueMap}")
    for i in range(len(search_list)):
        print(search_list[i])
        
        search_list[i] = search_list[i]+"_0"
        
        print(f"modifed version is {search_list[i]}")
        # input()
        
        
        
        target_pose_dict = {}
        detected_type_list = []
    map_objects = trueMap
        
        
    try_for_level3=input("[Y/N] ")  
    # create a dictionary of all the saved images with their corresponding robot pose
    target_est = {}
    photo_path=[[0.00001,0.00001],[0.00001,0.000005],[0,0.00001],[-0.00001,0.000005],[-0.00001,0.00001],[-0.00001,0],[-0.000005,0.00001],[-0.00001,-0.00001],[0,-0.00001],[-0.000005,-0.00001],[0.00001,-0.00001],[0.000005,-0.00001],[0,0]]   
    while try_for_level3=='Y':
        target_pose_dict=create_target_dict()
        for i in range(len(photo_path)):
            robot_pose = robot.state
            drive_to_point(photo_path[i-1],robot_pose)
            target_pose_dict=create_target_dict()
        target_est = merge_estimations(target_pose_dict)
        map_objects=update_map(map_objects,target_est)
        try_for_level3='N'
            
    trueMap = map_objects
    
    generatedPath = plan_new_path(trueMap=trueMap,shoppingList=search_list,currentPose=[0,0])
    
    # print(f"\n\nbelow ar genreated path {generatedPath}")
    # plan_new_path()
   

    # The following is only a skeleton code for semi-auto navigation
    while True:
        
        
    
        get_robot_pose()
        # enter the waypoints
        # instead of manually enter waypoints, you can give coordinates by clicking on a map, see camera_calibration.py from M2

        # try_for_level3=input("[Y/N] ")        
            
        

        autoNav = input("AUTO NAV [Y/N] ")
        
        # python auto_fruit_search.py     
        
        # path = [[0, 0], [-0.324860853924341, -0.009509236959331648], [-0.32568372013289204, -0.009533323671129728], [-0.9013706999610294, -0.17860283971805768], [-1.1082741399922058, -0.6117205679436115]]
        
        pathList = [[[0, 0], [-0.22621209794745933, -0.35743123358515566], [-0.22629743775057287, -0.35756607655499384], [-0.714724977902232, -0.4645171776311353], [-0.9963935843873644, -0.567777993629879]], [[-0.9963935843873644, -0.567777993629879], [-0.5103608429065181, -0.45042308658020747], [-0.04848927826448851, -0.2589168000642774], [0.12557964962561075, 0.20980487464598477], [0.6172742951189232, 0.3005593543877573], [0.6173693881393448, 0.30037594976178644], [0.6836519425925404, 0.17253768094074845], [0.6806562461744449, -0.32745334478168353], [0.732262498469778, -0.6229813379126734]], [[0.732262498469778, -0.6229813379126734], [0.6806016085954282, -0.12565734647711724], [0.6404125318398303, 0.15649480770486257], [0.5109206837334056, 0.22393264036564903], [0.398188084417936, 0.4324041277484055], [0.3978427010488033, 0.43304283017447603], [-0.09578201310290546, 0.5126332917803911], [-0.4790058634449182, 0.19147993195203886], [-0.7036023453779673, -0.00740802721918446]], [[-0.7036023453779673, -0.00740802721918446], [-0.2086725161277671, 0.0636163637910556], [0.2342840088656641, 0.2955420350000546], [0.23417720196068798, 0.29609327185150613], [0.18567088078427518, 0.5464373087406025]], [[0.18567088078427518, 0.5464373087406025], [0.3545173760883302, 0.18051400936826856], [0.41688306747356907, 0.08990208665364452], [0.4170621499078315, 0.08964189545985665], [0.05779473835586851, -0.2581035962388118], [-0.2915130385464162, -0.6158522188715072], [-0.45660521541856647, -0.866340887548603]]]
        
        pathList = generatedPath
        
            

        
        if autoNav == "Y":
            print("starting auto nav...")
            for path in pathList:
                for i in path:
                    print(f"\n\n=========The next waypoint is {i}")
                    
                    robot_pose = robot.state
                    drive_to_point(i,robot_pose)
                    # robot_pose = update_robot_position(robot_pose,waypoint)
                    
                    
                    robot_pose = robot.state
                    # input(f"robot is now at {robot.state}")
                    
                print("\n\n=======arrive at fruit ========\n\n")
                time.sleep(2)
                
                    
        
        x,y = 0.0,0.0
        x = input("X coordinate of the waypoint: ")
        try:
            x = float(x)
        except ValueError:
            print("Please enter a number.")
            continue
        y = input("Y coordinate of the waypoint: ")
        try:
            y = float(y)
        except ValueError:
            print("Please enter a number.")
            continue

        # estimate the robot's pose
        # robot_pose = get_robot_pose(robot_pose)

        # robot drives to the waypoint
        waypoint = [x,y]
        robot_pose = robot.state
        drive_to_point(waypoint,robot_pose)
        
        # robot_pose = update_robot_position(robot_pose,waypoint)
        
        
        robot_pose = robot.state

        
        print("Finished driving to waypoint: {}; New robot pose: {}".format(waypoint,robot_pose))

        # exit
        ppi.set_velocity([0, 0])
        uInput = input("Add a new waypoint? [Y/N]")
        if uInput == 'N':
            break
