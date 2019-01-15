


In this lesson we show to use the path planning to make the robot navigate from its current position (detected by the intelligent space) to a desire destination


Important: The robot must have an ArUco marker attached to its back

###
Main program:

* navigate_path.py - Localize the robot and make it go to the destination (x,y) entered as an argument to the program.

###

Additonal programs:

* plot_map.py - Plot the map passed as argument

* get_FrameTransf_Robot_ArUco.py - Read the estimated Frame Transformation between the ArUco marker attached to the robot's back and its base. That will be used to correct robot's odometry during navigation whenever the intelligent space can see the marker

The following programs were used to test if the activation and deactivation of the robot awareness was working:

* test_awareness_API.py - Turn off and on the robot awareness using the robot's API

* test_awareness_gateway.py - Turn off and on the robot awareness using the robot gateway in the Intelligent Space

The following programs were used to check if the odometry reading was working:

* read_odometry_API - Test reading robot's odometry using the robot's API directly

* read_odometry_gateway - Test reading robot's odometry using the robot gateway in the Intelligent Space

###
