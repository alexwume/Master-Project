#!/usr/bin/env python
'''
Author: Arti Anantharaman
Date: 09/14/2020

'''
import numpy as np
import rospy
from dvrk import psm
import PyKDL

def arm_planner(trajectory):
	rospy.init_node('controller', anonymous=True)
	robot = psm('PSM1')
	rate = rospy.Rate(100) # 10hz	
	for traj in trajectory:
		robot.move(traj)
		rate.sleep()


def main():
	# liver_points = np.load('/home/arti/catkin_dvrk_ws/src/Medical-DVRK-AR/data/liverGrid_norm_sept16.npy')	
	liver_points = np.load('/home/cora/dvrk/src/Medical-DVRK-AR/data/80degree_norm.npy')	
	print(liver_points.shape)
	all_points = []

	for row in liver_points:
		# rotation_mat = PyKDL.Rotation.RPY(row[3], row[4], row[5])
		rospy.init_node('controller', anonymous=True)
		robot = psm('PSM1')
		rate = rospy.Rate(100) # 10hz
		robot.home()

		rotation_mat = PyKDL.Rotation.Quaternion(row[3], row[4], row[5], row[6])

		vec = PyKDL.Vector(row[0],row[1],row[2])
		each_frame = PyKDL.Frame(rotation_mat, vec)
		all_points.append(each_frame)

	arm_planner(all_points)
	while not rospy.is_shutdown():
		print("dVRK has scanned the liver")
		rate.sleep()

if __name__ == '__main__':
	main()

