import curses
import os
from is_msgs.robot_pb2 import RobotConfig
from is_wire.core import Channel, Message, Subscription
from is_msgs.common_pb2 import Tensor
import numpy as np
import math



# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])


def makeMessage (linear,angular):
  # Create message with command for 
  # linear and angular velocities  
  message = Message()
  robotConfig = RobotConfig()
  robotConfig.speed.linear = linear
  robotConfig.speed.angular = angular
  message.pack(robotConfig)
  message.topic = "RobotGateway.0.SetConfig"
  # Public message
  channel.publish(message)

#######################
# Using keypad to control the robot with some 
# feedback on the screen

# Create a channel to connect to the broker
channel = Channel("amqp://10.10.2.20:30000")
# Create a subscription 
subscription = Subscription(channel)
# Subscribe to get the relation from the initial reference frame where the robot was
# turned on and its current reference frame (dead reckoning odometry)
subscription.subscribe("FrameTransformation.2000.2001")


stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)

stdscr.addstr(0, 10, "Hit 'q' to quit")
stdscr.refresh()

key = ''
linear,angular = 0,0



while True:
    # listen the channel
    message = channel.consume()
    # Check if the message received is the robot's odometry - FrameTransformation type
    if (message.topic == "FrameTransformation.2000.2001"):
        # unpack the message according to its format
        tensor = message.unpack(Tensor)
        # get the transformation matrix corresponding to the current rotation and position of the robot
        matrix=np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
        posX = matrix[0,3]
        posY = matrix[1,3]
        posZ = matrix[2,3]
        strPosition = 'X: ' + str(posX) + '   Y: ' + str(posY) + '   Z: ' + str(posZ)
        rotationMatrix = matrix[0:3,0:3] 
        angles = rotationMatrixToEulerAngles(rotationMatrix)
        strRotation = 'Yaw: ' + str(angles[2]) + '   Pitch: ' + str(angles[1]) + '    Roll: ' + str(angles[0])

        stdscr.addstr(5, 20, strPosition)
        stdscr.addstr(8, 20, strRotation)


    # waits for a key hit but doesn't block the program
    stdscr.nodelay(1)
    key = stdscr.getch()
    stdscr.addch(20, 25, key)
    stdscr.refresh()


    if key == curses.KEY_UP:
       stdscr.addstr(3, 20, "Up   ")
       linear = 10
       angular = 0

    elif key == curses.KEY_DOWN:
       stdscr.addstr(3, 20, "Down ")
       linear = -10
       anglar = 0

    elif key == curses.KEY_LEFT:
       stdscr.addstr(3, 20, "Left ")
       linear = 0
       angular = 0.3

    elif key == curses.KEY_RIGHT:
       stdscr.addstr(3, 20, "Right  ")
       linear = 0
       angular = -0.3
    elif key == ord('s'):
       stdscr.addstr(3, 20, "Stop   ")
       linear = 0
       angular = 0
    elif key == ord('q'):
        break


    makeMessage(linear,angular)
    



curses.endwin()
makeMessage(0,0)

