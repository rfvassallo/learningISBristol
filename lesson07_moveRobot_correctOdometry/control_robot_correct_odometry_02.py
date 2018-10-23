import curses
import os
from is_msgs.robot_pb2 import RobotConfig
from is_wire.core import Channel, Message, Subscription
from is_msgs.common_pb2 import Tensor
from is_msgs.camera_pb2 import FrameTransformation
from np_pb import to_tensor, to_np
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

# Create message with command for linear and angular velocities  
def makeMessage (linear,angular):
  
  message = Message()
  robotConfig = RobotConfig()
  robotConfig.speed.linear = linear
  robotConfig.speed.angular = angular
  message.pack(robotConfig)
  message.topic = "RobotGateway.0.SetConfig"
  # Public message
  channel.publish(message)


 




####  Main Program #######################
# Using keypad to control the robot with some 
# feedback on the screen

# Create a channel to connect to the broker
channel = Channel("amqp://10.10.2.20:30000")
# Create a subscription 
subscription = Subscription(channel)
# Subscribe to get the relation from the initial reference frame (2001) where the robot was
# turned on and its current reference frame (2000). This Frame Transforamtion corressponds
# to the dead reckoning odometry
subscription.subscribe("FrameTransformation.2000.2001")
subscription.subscribe("FrameTransformation.2001.1003")

stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)

stdscr.addstr(0, 10, "Hit 'q' to quit")
stdscr.refresh()

key = ''
linear,angular = 0,0

# Initialize transformation matrices used for storing odometry and correction
pepperPose = np.identity(4)
robotToWorld = np.identity(4)

# Create message used to send the odometry to the 
# program that plots the odometry using matplotlib
messageRobotPose = Message()
messageRobotPose.topic = "NewRobotPose"

while True:
    # listen the channel
    message = channel.consume()
    # Check if the message received is the robot's odometry - FrameTransformation type
    if (message.topic == "FrameTransformation.2000.2001"):
        # unpack the message according to its format
        frameTransf = message.unpack(FrameTransformation)
        tensor = frameTransf.tf
        
        # get the transformation matrix corresponding to the current rotation and position of the robot
        lastOdometry = np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
        pepperPose = np.matmul(robotToWorld,lastOdometry)
        # Separate the position
        pepperPosX = pepperPose[0,3]
        pepperPosY = pepperPose[1,3]
        pepperPosZ = pepperPose[2,3]
        strPosition = 'X: ' + str(pepperPosX) + '   Y: ' + str(pepperPosY) + '   Z: ' + str(pepperPosZ)
        # Separate the rotation angles
        rotationMatrix = pepperPose[0:3,0:3] 
        angles = rotationMatrixToEulerAngles(rotationMatrix)
        strRotation = 'Yaw: ' + str(angles[2]) + '   Pitch: ' + str(angles[1]) + '    Roll: ' + str(angles[0])

        # Print position and orinetation on the screen
        stdscr.addstr(5, 20, strPosition)
        stdscr.addstr(8, 20, strRotation)
        stdscr.addstr(12,20, "Reading normal odometry")


        # Convert the transformation matrix to tensor to be sent as a message
        msgContent = to_tensor(np.asarray(pepperPose))
        messageRobotPose.pack(msgContent)
        channel.publish(messageRobotPose)
               

    elif (message.topic == "FrameTransformation.2001.1003"):
        # unpack the message according to its format
        frameTransf = message.unpack(FrameTransformation)
        tensor = frameTransf.tf
        
        # get the transformation matrix corresponding to the current pose of the robot corrected when
        # it sees an ArUco marker
        robotToWorld = np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
        stdscr.addstr(14,20, "Both robot and IS saw the ArUco marker. Correcting odometry.")
    

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
    

# Close the screen layout
curses.endwin()
# Send message to stop the robot
makeMessage(0,0)

