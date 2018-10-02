import curses
import os
from is_msgs.robot_pb2 import RobotConfig
from is_wire.core import Channel, Message, Subscription
from is_msgs.common_pb2 import Tensor
import numpy as np
import math
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D





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

def create_world_frame():
    # Create the world frame
    u_vectors = np.array([1,0,0])
    v_vectors = np.array([0,1,0])
    w_vectors = np.array([0,0,1])
    posx = np.array([0,0,0])
    posy = np.array([0,0,0])
    posz = np.array([0,0,0])

    return posx, posy, posz, u_vectors, v_vectors, w_vectors

def get_robot_pose(a):

    # Separate the position and orientation of the camera  frame and axis
    u_robot_vec=np.array(a[0,0:3])
    v_robot_vec=np.array(a[1,0:3])
    w_robot_vec=np.array(a[2,0:3])
    robot_posx=np.array([a[0,3], a[0,3], a[0,3]])
    robot_posy=np.array([a[1,3], a[1,3], a[1,3]])
    robot_posz=np.array([a[2,3], a[2,3], a[2,3]])

    return robot_posx, robot_posy, robot_posz, u_robot_vec, v_robot_vec, w_robot_vec

def update_axis (ax,posx,posy):
    ax.cla()
    ax.text(1,0,0,"X")
    ax.text(0,1,0,"Y")
    ax.text(0,0,1,"Z")
    ax.text(0,0,0,"World")
    ax.set_xlim(-10,posx+10)
    ax.set_ylim(-10,posy+10)
    ax.set_zlim(-0.5,2.5) 
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')








#######################
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


#plt.ion()
# Create a 3D figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
update_axis (ax,0,0)
# world reference frame
posx,posy,posz,u_vectors,v_vectors,w_vectors = create_world_frame()
# plot frame
ax.quiver(posx,posy,posz,u_vectors,v_vectors,w_vectors)
plt.draw()
plt.pause(0.5)

pepperPose = np.identity(4)
robotToWorld = np.identity(4)



while True:
    # listen the channel
    message = channel.consume()
    # Check if the message received is the robot's odometry - FrameTransformation type
    if (message.topic == "FrameTransformation.2000.2001"):
        # unpack the message according to its format
        tensor = message.unpack(Tensor)

        # get the transformation matrix corresponding to the current rotation and position of the robot
        lastOdometry = np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
        pepperPose = np.matmul(robotToWorld,lastOdometry)
        pepperPosX = pepperPose[0,3]
        pepperPosY = pepperPose[1,3]
        pepperPosZ = pepperPose[2,3]
        strPosition = 'X: ' + str(pepperPosX) + '   Y: ' + str(pepperPosY) + '   Z: ' + str(pepperPosZ)
        rotationMatrix = pepperPose[0:3,0:3] 
        angles = rotationMatrixToEulerAngles(rotationMatrix)
        strRotation = 'Yaw: ' + str(angles[2]) + '   Pitch: ' + str(angles[1]) + '    Roll: ' + str(angles[0])

        stdscr.addstr(5, 20, strPosition)
        stdscr.addstr(8, 20, strRotation)

        # get new pose of the robot
        robot_posx, robot_posy, robot_posz, u_robot_vec, v_robot_vec, w_robot_vec = get_robot_pose(pepperPose)
        update_axis (ax,robot_posx[0],robot_posy[0])
        # plot world frame and robot frame
        ax.quiver(posx,posy,posz,u_vectors,v_vectors,w_vectors)
        ax.quiver(robot_posx,robot_posy,robot_posz,u_robot_vec,v_robot_vec,w_robot_vec)
        # add label to robot frame
        ax.text(pepperPosX,pepperPosY,pepperPosZ,"Robot")
        
        plt.draw()
        plt.pause(0.01)

        

    elif (message.topic == "FrameTransformation.2001.1003"):
        # unpack the message according to its format
        tensor = message.unpack(Tensor)
        # get the transformation matrix corresponding to the current pose of the robot corrected when
        # it sees an ArUco marker
        robotToWorld = np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
        
    




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

