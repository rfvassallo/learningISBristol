

In this lesson, you will see how to correct the robot's odometry using an ArUco marker present in the workspace, when the robot sees the marker.

IMPORTANT 01: Note that in this case we are correcting the robot's odometry just when the robot sees the marker. We are not using the marker attached to the robot's back.

This was our first try to correct robot's odometry but it was not so precise and frequent as needed. So we changed later to use the marker at the robot's back. 

IMPORTANT 02: This code may contain errors or may be not updated, since we decided not to use this approach anymore.


###
* control_robot_correct_odometry.py

Control the robot with the keyboard and correct its odometry when the robot sees an ArUco marker

###

* plot_odometry.py

Plot robot's odometry while it moves. You can use that to see if the odometry correction is working. 
 
###
