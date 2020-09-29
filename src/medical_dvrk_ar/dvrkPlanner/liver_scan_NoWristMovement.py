#!/usr/bin/env python
import rospy
from dvrk import psm
import PyKDL

def main():
    rospy.init_node('controller', anonymous=True)
    robot = psm('PSM1')
    rate = rospy.Rate(100) # 10hz
    robot.home()

    step_val_x = 0.004
    step_val_y = 0.001
    step_val_z = 0.0001
    dir = 1
    z_dir = 1
    amp = 0.05
    freq = 0.5

    for i in range(50):
        for j in range(90):
            if j > 45:
                z_dir = -1
            robot.dmove(PyKDL.Vector(0, step_val_y*dir, step_val_z*z_dir))
        robot.dmove(PyKDL.Vector(-step_val_x, 0, 0))
        dir *= -1
        z_dir = 1
    while not rospy.is_shutdown():
        print("Scan complete")
        rate.sleep()


if __name__ == '__main__':
    main()
