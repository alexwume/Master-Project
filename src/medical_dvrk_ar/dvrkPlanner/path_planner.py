#!/usr/bin/env python 
import rospy
from dvrk import psm
import math
import PyKDL
import numpy as np
from tf_conversions import posemath
import os.path

'''
Task_Planner Class (Object)
    - Descriptions:
        A Task_Planner class is to plan a reasonable trajectory for a dVRK robot arm(PSM1) 
        to visit all the points given. Point data are given in the form of a dictionary which 
        contains the positions (x, y, z) and the normal vectors(vx, vy, vz). The goal of this 
        planner is not just to visit all the points in the dictionary but also with the proper
        angles. The reason for this is to obtain better quality blaser results. This planner
        also has the ability to predict the movements of the points based on input freq. and 
        amplitude. By doing so, the path planner is capable of doing motion compensation to a 
        certain degree.

    - Inputs:
        1) Organ movement frequency (double)
        2) Organ movement amplitude (double)
        3) Points data(nested dictionary)  (this might be read from a .txt file or a .csv file)
            {
             1: {'pos_x': (double)x_value, 'pos_y': (double)y_value, 'pos_z': (double)z_value, 
                  'n_vx': (double)nvx_value, 'n_vy': (double)nvy_value, 'n_vz': (double)nvz_value},
             2: {'pos_x': (double)x_value, 'pos_y': (double)y_value, 'pos_z': (double)z_value, 
                  'n_vx': (double)nvx_value, 'n_vy': (double)nvy_value, 'n_vz': (double)nvz_value}
            }
            
        
    - Outputs:
        1) Robot motions
'''
class Task_Planner:
    # Exp: Initiating a Task_Planner class based on the given inputs.
    def __init__(self, data = {}, frequency = 0.5, amplitude = 1):    
        rospy.init_node('controller', anonymous=True)
	    self.robot = psm('PSM1')
	    self.rate = rospy.Rate(100) 
        # self.robot_pose is used to store the robot pose
        self.robot_pose = self.robot.get_current_position()
        # self.predict_period is the update period of the simulation
        self.predict_period = 100
        # self.sim_start_time is the start time (ros time) of the simulation
        self.sim_start_time = None
        # self.freq is the estimated frequency of the liver
        self.freq = frequency
        # self.amp is the estimated amplitude of liver
        self.amp = amplitude
        # self.ori_data is the orginal data
        self.ori_data = data
        # self.ordered_data is the sorted data
        self.ordered_data = None
        # self.cur_point stores the current iteration id
        self.cur_point = 0
        # self.number_of_data is the total number of data points
        self.number_of_data = len(self.data)

    # Exp: The main code runner.
    def run(self):
        # command the robot to home position
        self.robot.home()
        # call the function to sort the raw data
        self.ordered_data = self.visit_plan()


        safe_pos = PyKDL.Frame( PyKDL.Rotation(PyKDL.Vector(0, 1, 0),
                                               PyKDL.Vector(1, 0, 0),
                                               PyKDL.Vector(0, 0,-1)), 
                                PyKDL.Vector(-0.05,0,-0.10))
        self.robot.move(safe_pos)
        '''
            The While loop below is for the overall planner after incorporating motion compensation
        '''
        while True:
            # record the start time of the simulation
            self.sim_start_time = rospy.Time.now().to_sec()
            # self.predicted_data = self.points_prediction()
            
            # when cur_time < t < end_time the system will not update its freq and amp 
            cur_time = rospy.Time.now().to_sec()
            end_time = cur_time + self.predict_period
            while (cur_time <= end_time):
                #Visit points in the data 
                for point in range (self.cur_point, self.number_of_data):
                    # update current robot pose
                    self.robot_pose = self.robot.get_current_position()
                    # update current time in ros time
                    cur_time = rospy.Time.now().to_sec()
                    # estimate the position of the next point in the future
                    estimated_pos = self.estimated_point(self.robot_pose, self.cur_point + 1)
                    # move to next point
                    estimated_point_vec = (estimated_point['pos_x'], estimated_point['pos_y'], estimated_point['pos_z'])
                    self.robot.move(PyKDL.Vector(estimated_point_vec))
                    # update current point
                    self.cur_point += 1
                    # break if reach the end of the list
                    if point >= self.number_of_data:
                        return
        # '''
        #     For PR7, there liver is going to be static. So the robot just needs to go to each of the points
        #     passed through the dictionary. The While loop below is to get a 3D point cloud of the static liver
        # '''
        # for key in data:
        #     point_x = data[key]['pos_x']
        #     point_y = data[key]['pos_y']
        #     point_z = data[key]['pos_z']
        #     '''
        #         Ignored the normal vectors for now since the quality of the obtained point cloud
        #         on hardware didn't seem to depend on it; will add it in if we discover that it matters in simulation
        #     '''
        #     self.robot.move(PyKDL.Vector(point_x, point_y, point_z))

        return 

    # Exp: Plan the order of the points to be visited and reordered self.data
    def visit_plan(self):
        '''
            If we assign IDs to points in the same raster-scan order, this function is not necessary
        '''
        # Input: 1) self.data
        # Output: 1) ordered self.data
        # TODO

        return 
    # Exp: Predict the locations of given points in the future time  0 <= t <= self.time_frame.
    def points_estimation(self,next_point_id, time):
        # Input:    1) next_point_id: the id of the next point to visit
        #           2) time: future time instant
        # Output:   1) predicted_pose(nested_dictionary): Predicated data within the time period 

        pose = self.ordered_data[next_point_id]
        run_time = time - self.sim_start_time
        pose[2] = pose[2] + self.amp * math.sin(self.freq * run_time)

        return predicted_pose

    # Exp: Predict the position of a certain point by estimating a reached time and search in the 
    #      predicted table to obtain the location
        self.estimated_point(self.robot_pose, self.cur_point + 1)
    def estimated_point(self, cur_pose, next_point_id)
        # Input:    1) cur_point: current point
        #           2) next_point: which point to go next
        # Output:   1) estimated_point: estimated location (x,y,z,vx,vy,vz) of the next point 
        
        # initial guess of the time to reach the next point
        t_guess = 1

        threshold = 0.05
        # learning rate
        alph = 0.05
        # initalized t_error
        t_error = 1
        # dVRK moving speed
        robot_velocity = 1
        # current time in ros time
        current_time = rospy.Time().now().to_sec()
        # current robot pose
        current_pose = self.robot.get_current_position()

        while (t_error > threshold):
            estimated_point = self.point_estimation(next_point_id, current_time + t_guess)
            dist = np.linalg.norm(estimated_point - current_pose)
            t_actual = dist / robot_velocity
            t_error = t_guess - t_actual
            t_guess -= alph * t_error

    return estimated_point



if __name__=="__main__":
    data = {
            1: {'pos_x': 0.5, 'pos_y': 0.0, 'pos_z': 0.1, 'n_vx': 0, 'n_vy': 0, 'n_vz': 1},
            2: {'pos_x': 0.6, 'pos_y': 0.2, 'pos_z': 0.1, 'n_vx': 0, 'n_vy': 0, 'n_vz': 1},
            3: {'pos_x': 0.7, 'pos_y': 0.3, 'pos_z': 0.3, 'n_vx': 0, 'n_vy': 0, 'n_vz': 1},
            4: {'pos_x': 0.8, 'pos_y': 0.3, 'pos_z': 0.4, 'n_vx': 0, 'n_vy': 0, 'n_vz': 1}
            }
    frequency = 0.5
    amplitude = 1
    planner = Task_Planner(data, frequency, amplitude)
    planner.run()
