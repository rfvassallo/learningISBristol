

In this lesson, you will see how to send the robot to a goal/destination defined by (x,y) coordinates.
You can also use this program to test the difference between moving the robot using NavigateTo and MoveTo services.

The main difference is:

NavigateTo uses obstacle avoidance and turns the robot in the direction of the movement. The message carries just (x,y) coordinates. 

MoveTo doesn't use obstacle avoidance, just the low level security behavior, and you have to indicate the robot's heading if you want it to change. The message carries (x,y) coordinates and heading.


IMPORTANT 01: Note that in this case we are correcting the robot's odometry just when the robot sees the marker. We are not using the marker attached to the robot's back.

IMPORTANT 02: This code may contain errors or may be not updated, since we decided not to use this approach anymore.


###
* go_to.py

Make the robot move until the goal destination using either NavigateTo or MoveTo. You have to enter the goal coordinates directly in the program. Also you must select with a boolean variable if you want to use NavigateTo or MoveTo.

###

