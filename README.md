# DVRK Augmented reality using laser scanner

## Installation

1. Install ROS (please use ros-kinetic)
2. Install dVRK simulation evironments 
```
$ sudo apt-get install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig flite sox espeak cmake-curses-gui cmake-qt-gui libopencv-dev git subversion gfortran libcppunit-dev
$ sudo apt-get install qt5-default
$ sudo apt-get install python-catkin-tools
$ source /opt/ros/kinetic/setup.bash
$ mkdir -p ~/catkin_ws/src  
  #Change [catkin_ws] to whatever name you want
$ cd ~/catkin_ws/
$ catkin init #This creates a hidden .catkin_tools directory in the specified workspace.
$ cd ~/catkin_ws/src
$ git clone https://github.com/jhu-cisst/cisst-saw --recursive
$ cd ~/catkin_ws
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release 
  # At this point you might see there things that are still missing from the package "build", "devel". These things would be built in the next step.
$ catkin build #This might take a while
$ source devel/setup.bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/jhu-dvrk/dvrk-ros
$ git clone https://github.com/jhu-dvrk/dvrk-gravity-compensation
$ cd ~/catkin_ws
$ catkin build
$ cd src 
$ git clone https://github.com/biorobotics/Medical-DVRK-AR.git
$ cd ~/catkin_ws
$ catkin build
```
## Structure
```
your_workspace/
  build/
  devel/
  logs/
  src/
    cisst-saw/
    dvrk-gravity-compensation/
    dvrk-ros/
    Medical-DVRK-AR/
      config/
      data/
      launch/
      src/
      srv/
       :
      [blablablabla]
       :
```
## dVRK Simulation
**First terminal:**
```
$ roscore 
```
**Second terminal:**

```
$ source ~/your_workspace/devel/setup.bash
$ roslaunch medical_dvrk_ar dvrk_arm_rviz.launch arm:=PSM1
```
Press Power-On, then Home

**Third terminal:**
```
$ rosrun medical_dvrk_ar blaser_sim.py -j ~/your_workspace/src/Medical-DVRK-AR/config/blaser_SIMULATED.json
```

**In Rviz:**
After launching Rviz, add in the following two rostopics to visualize the results 1)pointcloud2 2)makerarray.

## How to import different .stl files into Rviz
1. Add the .stl file in the "data/" folder.
2. Edit "/config/blaser_SIMULATED.json" line:47(mesh_resource) to the new .stl file path.
3. Run blaser_sim.py .

## How to get 3D point cloud of the stl file with simulated arm scanning (blaser stitching)
1. Run "python Modeling/pc_sticher.py" to start listener.
2. Add the "Pointcloud2" topic by the name "organ_3d_point_cloud".
3. Run any robot control script under the dvrkPlanner folder.


## Path planning key points visualization
1. cd /your_file_path/src/Medical-DVRK-AR/src/medical_dvrk_ar/Modeling/
2. python3 visualize_points_norm.py --path '../../../data/60degree_norm.npy'
2. you can change '60degree_norm.npy' to '80degree_norm.npy' which contain more norm vectors
![alt text](https://github.com/biorobotics/Medical-DVRK-AR/blob/master/data/position_vis.png)
![alt text](https://github.com/biorobotics/Medical-DVRK-AR/blob/master/data/norm_vis.png)
