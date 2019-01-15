

In this lesson, you will control the robot using the computer keyboard, but if the robot goes beyond a safety area, a warning  message is printed on the screen

###
* check_safety_zone.py

Move the robot and read its odometry. If the robot is out of the safety zone, prints a warning.

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
