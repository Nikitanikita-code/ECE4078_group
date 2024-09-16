import numpy as np
from slam.mapping_utils import MappingUtils
import cv2
import math
import pygame



from util.pibot import PenguinPi    # access the robot
import util.DatasetHandler as dh    # save/load functions
import util.measure as measure      # measurements
import pygame                       # python package for GUI
import shutil                       # python package for file operations


from slam.ekf import EKF
from slam.robot import Robot
import slam.aruco_detector as aruco

# import YOLO components 
from YOLO.detector import Detector


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


for key, value in map_objects.items():
    
    name= key[:5]   # "aruco" (first 5 characters)
    number_part = key[5:key.index('_')] 
    
    if(name == "aruco"):
        x_coord = value["x"]
        y_coord = value["y"]
        print(f" {key}: LM {number_part} X: {x_coord}, Y: {y_coord}")


class EKF:
    # Implementation of an EKF for SLAM
    # The state is ordered as [x; y; theta; l1x; l1y; ...; lnx; lny]

    ##########################################
    # Utility
    # Add outlier rejection here
    ##########################################

    def __init__(self, robot, trueMap =None):
        # State componentsWeek02-04/diy_prints/LabPrintingMarker_A3_Compact.pdf
        self.robot = robot
        self.markers = np.zeros((2,0))
        self.taglist = []

        # if trueMap != None:
            
            
        
        # else:
        self.taglist = []

        # Covariance matrix
        self.P = np.zeros((3,3))
        self.init_lm_cov = 1e3
        self.robot_init_state = None
        self.lm_pics = []
        for i in range(1, 11):
            f_ = f'./pics/8bit/lm_{i}.png'
            self.lm_pics.append(pygame.image.load(f_))
        f_ = f'./pics/8bit/lm_unknown.png'
        self.lm_pics.append(pygame.image.load(f_))
        self.pibot_pic = pygame.image.load(f'./pics/8bit/pibot_top.png')
        
    def reset(self):
        self.robot.state = np.zeros((3, 1))
        self.markers = np.zeros((2,0))
        self.taglist = []
        # Covariance matrix
        self.P = np.zeros((3,3))
        self.init_lm_cov = 1e3
        self.robot_init_state = None

    def number_landmarks(self):
        return int(self.markers.shape[1])

    def get_state_vector(self):
        state = np.concatenate(
            (self.robot.state, np.reshape(self.markers, (-1,1), order='F')), axis=0)
        return state
    
    def set_state_vector(self, state):
        self.robot.state = state[0:3,:]
        self.markers = np.reshape(state[3:,:], (2,-1), order='F')
    
    def save_map(self, fname="slam_map.txt"):
        if self.number_landmarks() > 0:
            utils = MappingUtils(self.markers, self.P[3:,3:], self.taglist)
            utils.save(fname)

    def recover_from_pause(self, measurements):
        if not measurements:
            return False
        else:
            lm_new = np.zeros((2,0))
            lm_prev = np.zeros((2,0))
            tag = []
            for lm in measurements:
                if lm.tag in self.taglist:
                    lm_new = np.concatenate((lm_new, lm.position), axis=1)
                    tag.append(int(lm.tag))
                    lm_idx = self.taglist.index(lm.tag)
                    lm_prev = np.concatenate((lm_prev,self.markers[:,lm_idx].reshape(2, 1)), axis=1)
            if int(lm_new.shape[1]) > 2:
                R,t = self.umeyama(lm_new, lm_prev)
                theta = math.atan2(R[1][0], R[0][0])
                self.robot.state[:2]=t[:2]
                self.robot.state[2]=theta
                return True
            else:
                return False
        
    ##########################################
    # EKF functions
    # Tune your SLAM algorithm here
    # ########################################

    # the prediction step of EKF
    def predict(self, raw_drive_meas):

        F = self.state_transition(raw_drive_meas)
        Q = self.predict_covariance(raw_drive_meas) 
        
            
        Q_weight = 1
        print("\n\n============at predic======\n\n")
        print(f"\tthe current state of the robot is {self.robot.state[0:2,:]}")
        if(abs(self.robot.state[0:2,:][0][0]) < 0.02 and abs(self.robot.state[0:2,:][1][0]) < 0.02):
            print("robot at x0 y0")
            Q_weight = 0.1
        
        self.robot.drive(raw_drive_meas) 
        self.P = F @ self.P @ F.T + Q_weight*Q #updating P for the next time step
    
    def compute_gobal_marker(self,lm):
        
        th = self.robot.state[2]
        robot_xy = self.robot.state[0:2,:]
        R_theta = np.block([[np.cos(th), -np.sin(th)],[np.sin(th), np.cos(th)]])
        
        lm_bff = lm.position
        lm_inertial = robot_xy + R_theta @ lm_bff


        
        return lm_inertial
    
        
        

    # the update step of EKF
    def update(self, measurements):

        
        print("\n\n=====at update step======\n\n")
        
        if not measurements:
            return

        # Construct measurement index list
        tags = [lm.tag for lm in measurements]
        idx_list = [self.taglist.index(tag) for tag in tags]
        
        # Stack measurements and set covariance
        z = np.concatenate([lm.position.reshape(-1,1) for lm in measurements], axis=0)
        
        R = np.zeros((2*len(measurements),2*len(measurements))) 
        
        robotPos = self.robot.state
        
        
        for i in range(len(measurements)):
            lmTag = measurements[i].tag
            lmLocalPos = measurements[i].position
            lmGobelPos = self.compute_gobal_marker(measurements[i])
            

            print(f"the lm we see is {lmTag}")
            print(f"\tthe robot have state x: {robotPos[0]} y: {robotPos[1]} th: {robotPos[2]}")
            
            print(f"\tgobal x,y for LM {lmTag} is [{np.round(lmGobelPos[0],3)}, {np.round(lmGobelPos[1],3)}")
            
            prepDist = math.sqrt((lmLocalPos[0])**2 + (lmLocalPos[1])**2)
            # if prepDist < 1.5:
            weight = 250*(prepDist-0.2)**2 #200 be good
            
            print(f"\tlocal x,y of the LM{lmTag} is [{np.round(lmLocalPos[0],3)},{np.round(lmLocalPos[1],3)}")
            print(f"\tthe direct distance is {prepDist}, gives a weight of {weight}")
            
            # rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.distortion_params)
            # print(f"\tunmodified weight is {measurements[i].covariance}")
            # weight = 1/weight
            
            R[2*i:2*i+2,2*i:2*i+2] = weight*(measurements[i].covariance)
            

        # Compute own measurements
        z_hat = self.robot.measure(self.markers, idx_list)
        z_hat = z_hat.reshape((-1,1),order="F")
        H = self.robot.derivative_measure(self.markers, idx_list) #jacobian of error 

        x = self.get_state_vector()
        
        Y = z - z_hat #error term between the sensor meaurements and the state
        S = H @ self.P @ H.T + R
        
        S_inv = np.linalg.inv(S) #finding S^-1

        K = self.P @ (H.T @ S_inv)#updating K, the kalman gain, it acts as a step size and tell you how much you need to update the states based on the observation 
        # print(f"\n\n========= current state is {x[0:2]}=============\n\n")
        # for lm in measurements:
        #     print(f"tag {lm.tag}")
        #     print(lm.position)
        #     print(f"the prep distance is {math.sqrt((lm.position[0])**2 + (lm.position[1])**2)}")
        self.set_state_vector(x+K@Y)
        
        identitySize = ((K @ H).shape)[0]

        
        self.P =  (np.eye(identitySize) - K @ H)@self.P #TODO NEED OT CHECK ^^^^^^

        # TODO: add your codes here to compute the updated x


    def state_transition(self, raw_drive_meas):
        n = self.number_landmarks()*2 + 3
        F = np.eye(n)
        F[0:3,0:3] = self.robot.derivative_drive(raw_drive_meas)
        return F
    
    def predict_covariance(self, raw_drive_meas):
        n = self.number_landmarks()*2 + 3
        Q = np.zeros((n,n))


        
        Q[0:3,0:3] = self.robot.covariance_drive(raw_drive_meas)+ 0.009*np.eye(3)
        return Q

    def add_landmarks(self, measurements):
        if not measurements:
            return

        th = self.robot.state[2]
        robot_xy = self.robot.state[0:2,:]
        R_theta = np.block([[np.cos(th), -np.sin(th)],[np.sin(th), np.cos(th)]])

        # Add new landmarks to the state
        for lm in measurements:
            if lm.tag in self.taglist:
                # ignore known tags
                continue
            
            lm_bff = lm.position
            lm_inertial = robot_xy + R_theta @ lm_bff

            self.taglist.append(int(lm.tag))
            self.markers = np.concatenate((self.markers, lm_inertial), axis=1)
            
            # print(f"at predict covariance P prev is {self.P.shape}")

            # Create a simple, large covariance to be fixed by the update step
            self.P = np.concatenate((self.P, np.zeros((2, self.P.shape[1]))), axis=0)
            self.P = np.concatenate((self.P, np.zeros((self.P.shape[0], 2))), axis=1)
            self.P[-2,-2] = self.init_lm_cov**2
            self.P[-1,-1] = self.init_lm_cov**2
            
            print(f"after that P is  {self.P.shape}")

    ##########################################
    ##########################################
    ##########################################

    @staticmethod
    def umeyama(from_points, to_points):

    
        assert len(from_points.shape) == 2, \
            "from_points must be a m x n array"
        assert from_points.shape == to_points.shape, \
            "from_points and to_points must have the same shape"
        
        N = from_points.shape[1]
        m = 2
        
        mean_from = from_points.mean(axis = 1).reshape((2,1))
        mean_to = to_points.mean(axis = 1).reshape((2,1))
        
        delta_from = from_points - mean_from # N x m
        delta_to = to_points - mean_to       # N x m
        
        cov_matrix = delta_to @ delta_from.T / N
        
        U, d, V_t = np.linalg.svd(cov_matrix, full_matrices = True)
        cov_rank = np.linalg.matrix_rank(cov_matrix)
        S = np.eye(m)
        
        if cov_rank >= m - 1 and np.linalg.det(cov_matrix) < 0:
            S[m-1, m-1] = -1
        elif cov_rank < m-1:
            raise ValueError("colinearility detected in covariance matrix:\n{}".format(cov_matrix))
        
        R = U.dot(S).dot(V_t)
        t = mean_to - R.dot(mean_from)
    
        return R, t

    # Plotting functions
    # ------------------
    @ staticmethod
    def to_im_coor(xy, res, m2pixel):
        w, h = res
        x, y = xy
        x_im = int(-x*m2pixel+w/2.0)
        y_im = int(y*m2pixel+h/2.0)
        return (x_im, y_im)

    def draw_slam_state(self, res = (320, 500), not_pause=True):
        # Draw landmarks
        m2pixel = 100
        if not_pause:
            bg_rgb = np.array([213, 213, 213]).reshape(1, 1, 3)
        else:
            bg_rgb = np.array([120, 120, 120]).reshape(1, 1, 3)
        canvas = np.ones((res[1], res[0], 3))*bg_rgb.astype(np.uint8)
        # in meters, 
        lms_xy = self.markers[:2, :]
        robot_xy = self.robot.state[:2, 0].reshape((2, 1))
        lms_xy = lms_xy - robot_xy
        robot_xy = robot_xy*0
        robot_theta = self.robot.state[2,0]
        # plot robot
        start_point_uv = self.to_im_coor((0, 0), res, m2pixel)
        
        p_robot = self.P[0:2,0:2]
        axes_len,angle = self.make_ellipse(p_robot)
        canvas = cv2.ellipse(canvas, start_point_uv, 
                    (int(axes_len[0]*m2pixel), int(axes_len[1]*m2pixel)),
                    angle, 0, 360, (0, 30, 56), 1)
        # draw landmards
        if self.number_landmarks() > 0:
            for i in range(len(self.markers[0,:])):
                xy = (lms_xy[0, i], lms_xy[1, i])
                coor_ = self.to_im_coor(xy, res, m2pixel)
                # plot covariance
                Plmi = self.P[3+2*i:3+2*(i+1),3+2*i:3+2*(i+1)]
                axes_len, angle = self.make_ellipse(Plmi)
                # print("\n AXES LEN, ", axes_len, " angle: ", angle, "\n")
                canvas = cv2.ellipse(canvas, coor_, 
                    (int(axes_len[0]*m2pixel), int(axes_len[1]*m2pixel)),
                    angle, 0, 360, (244, 69, 96), 1)

        surface = pygame.surfarray.make_surface(np.rot90(canvas))
        surface = pygame.transform.flip(surface, True, False)
        surface.blit(self.rot_center(self.pibot_pic, robot_theta*57.3),
                    (start_point_uv[0]-15, start_point_uv[1]-15))
        if self.number_landmarks() > 0:
            for i in range(len(self.markers[0,:])):
                xy = (lms_xy[0, i], lms_xy[1, i])
                coor_ = self.to_im_coor(xy, res, m2pixel)
                try:
                    surface.blit(self.lm_pics[self.taglist[i]-1],
                    (coor_[0]-5, coor_[1]-5))
                except IndexError:
                    surface.blit(self.lm_pics[-1],
                    (coor_[0]-5, coor_[1]-5))
        return surface

    @staticmethod
    def rot_center(image, angle):
        """rotate an image while keeping its center and size"""
        orig_rect = image.get_rect()
        rot_image = pygame.transform.rotate(image, angle)
        rot_rect = orig_rect.copy()
        rot_rect.center = rot_image.get_rect().center
        rot_image = rot_image.subsurface(rot_rect).copy()
        return rot_image       

    @staticmethod
    def make_ellipse(P):
        e_vals, e_vecs = np.linalg.eig(P)
        idx = e_vals.argsort()[::-1]   
        e_vals = e_vals[idx]
        e_vecs = e_vecs[:, idx]
        alpha = np.sqrt(4.605)
        axes_len = e_vals*2*alpha
        if abs(e_vecs[1, 0]) > 1e-3:
            angle = np.arctan(e_vecs[0, 0]/e_vecs[1, 0])
        else:
            angle = 0
        return (axes_len[0], axes_len[1]), angle
    

