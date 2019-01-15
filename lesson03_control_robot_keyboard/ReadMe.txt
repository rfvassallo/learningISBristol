

In this lesson, you will control the robot using the computer keyboard. To move the robot you will use the arrows, to stop you press 's' and to quit the program use 'q'

###
* control_robot_keyboard.py

Control the robot using the keyboard arrows. To do that the program sends messages with velocities using the robot gateway.

###

* consume_odometry.py

Print the robot's odometry, received through messages provided by the FrameTransformation service.

###

* control_robot_consume_odometry.py

Control the robot and read its odometry at the same time.

###

Important:

To test the programs, you'll need to clone the repository of the gateway for the robot.

*   is-pepper-gateways

and also follow the instructions on the Readme from the repository, which are:

1- Download the naoqi python sdk
2- Remeber to correctly export sdk python path, e.g: export PYTHONPATH=$HOME/pynaoqi-python2.7-2.5.5.5-linux64/lib/python2.7/site-packages
3- Install other dependencies pip install --user -r requirements.txt
4- Start any gateway by running the respective service.py file python. For example: ./robot-gateway/service.py
   The gateway parameters are passed via environment variables.

You can use either a virtual robot (which you can see using Choregraphe) or the real robot. You just have to change the IP adress and port
