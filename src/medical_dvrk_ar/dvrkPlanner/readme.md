# Planning and Control of Dvrk Robot

When running the blaser, it listens to the robot end effector position. This is published at a very high rate (90Hz) making the Blaser lag a lot. I suggest throttling the topic so that it's only at 10Hz which will make the Blaser not lag as much.

run this command in a separate terminal:
rosrun topic_tools throttle messages dvrk/PSM1/position_cartesian_current 10.0

### liver_scan_NoWristMovement.py - Chang

@Chang, please add the main description here

### robot_scanning.py - Chang

@Chang, please add the main description here

### scan_segmented_liver_NoWristMovement.py - Chang

@Chang, please add the main description here

### test_program.py test_program_circle.py - Chang

@Chang, please add the main description here
