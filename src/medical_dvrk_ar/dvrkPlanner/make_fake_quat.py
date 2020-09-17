#!/usr/bin/env python
'''
Author: Arti Anantharaman
Date: 09/14/2020

'''
import numpy as np
import rospy
from dvrk import psm
import PyKDL

def main():
	rospy.init_node('controller', anonymous=True)
	robot = psm('PSM1')
	rate = rospy.Rate(100) # 10hz
	robot.home()
	position_start = robot.get_current_position()

	safe_pos = PyKDL.Frame( PyKDL.Rotation(PyKDL.Vector(0, 1, 0),
										   PyKDL.Vector(1, 0, 0),
										   PyKDL.Vector(0, 0,-1)), 
							PyKDL.Vector(-0.05,0,-0.10))
	robot.move(safe_pos)

	# pos = safe_pos.p
	# rot = PyKDL.Rotation(safe_pos.M)
	# rot	= rot.GetQuaternion()
	# print('Quaternion = ')
	# print(rot)

	fake_numpy = np.array([[0.05,0,-0.10, 0.7071067811865475, 0.7071067811865476, 0.0, 0.0], \
						   [0.05, 0.03, -0.1, 0.7071067811865475, 0.7071067811865476, 0.0, 0.0], \
						   [0.03, 0.03, -0.1, 0.7071067811865475, 0.7071067811865476, 0.0, 0.0], \
						   [0.03, 0.02, -0.1, 0.7071067811865475, 0.7071067811865476, 0.0, 0.0], \
						   [0.02, 0.01, -0.1, 0.7071067811865475, 0.7071067811865476, 0.0, 0.0], \
						   [0.00,0,-0.10, 0.7071067811865475, 0.7071067811865476, 0.0, 0.0], \
						   [0.01, 0.03, -0.1, 0.7071067811865475, 0.7071067811865476, 0.0, 0.0], \
						   [0.03, 0.03, -0.1, 0.7071067811865475, 0.7071067811865476, 0.0, 0.0], \
						   [0.03, 0.02, -0.1, 0.7071067811865475, 0.7071067811865476, 0.0, 0.0], \
						   [0.04, 0.01, -0.1, 0.7071067811865475, 0.7071067811865476, 0.0, 0.0]])
	print(fake_numpy.shape)
	np.save('fake_quat', fake_numpy)	
	
	while not rospy.is_shutdown():
		print("Numpy File saved")
		rate.sleep()

if __name__ == '__main__':
	main()

