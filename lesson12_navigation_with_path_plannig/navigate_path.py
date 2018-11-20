
from is_wire.core import Channel, Message, Subscription
from is_msgs.common_pb2 import Position, Pose
from is_msgs.camera_pb2 import FrameTransformation
from is_msgs.robot_pb2 import RobotConfig
import matplotlib.pyplot as plt
import numpy as np
import numpy.matlib
from numpy.linalg import inv
import numpy.matlib
import math
import sys
import qi
sys.path.append('../lesson10_path_planning/')
#from path_planning_PRM import PRM_planning, create_virtualObstacles, read_map




def read_frameTransformation(fileName):
# Function to read the file that contains the map
    with open(fileName, 'r') as f:
        # read lines
        line = f.readline()
        header = []
        frameMatrix = np.matlib.zeros((4,4))
        # Go through the lines to get the (x,y) coordinates of the map
        i=0
        while line:
            if line.startswith('#'):
                header.append(line)
            else:
                # split the line when find a space
                data = line.split()
                frameMatrix[i,0] = float(data[0])
                frameMatrix[i,1] = float(data[1])
                frameMatrix[i,2] = float(data[2])
                frameMatrix[i,3] = float(data[3])
                i=i+1
            line = f.readline()
        
        
    return frameMatrix



with_robot = True

if with_robot:
    # Create a channel to connect to the broker
    channel = Channel("amqp://10.10.2.20:30000")
    # Create a subscription 
    subscription = Subscription(channel)

# Subscribe to get the relation from the initial reference frame where the robot was
# turned on and its current reference frame (dead reckoning odometry)

'''
session = qi.Session()
try:
    session.connect("tcp://10.10.0.111:9559")
except RuntimeError:
    print ("Can't connect to Naoqi")

motion_service  = session.service("ALMotion")
'''
##################subscription.subscribe("FrameTransformation.1000.2000")

robotArUco = 8
worldFrame = 1000

if with_robot:
    topicGetRobotOdometry = "FrameTransformation.2000.2001"
    topicGetArUcoRobotBack = "FrameTransformation."+str(robotArUco+100)+"."+str(worldFrame)
    subscription.subscribe(topicGetRobotOdometry)
    subscription.subscribe(topicGetArUcoRobotBack)



# Initialize transformation matrices used for storing odometry and correction
pepperPose = np.identity(4)
robotOriginToWorld = np.identity(4)
lastOdometry = np.identity(4)

fileName = "frameArUcoRobot.dat"
arUcoToRobot = read_frameTransformation(fileName)
robotToArUco = inv(arUcoToRobot)


# Parameters for Path-planning
mapFile = "map0511.dat"
step = 2
robotLocalized = False


if with_robot:
    while not robotLocalized:
        # Source must be the current position of the robot in the world frame
        message = channel.consume()
        if (message.topic == topicGetRobotOdometry):
            # unpack the message according to its format
            frameTransf = message.unpack(FrameTransformation)
            tensor = frameTransf.tf
            
            # get the transformation matrix corresponding to the current rotation and position of the robot
            lastOdometry = np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
            subscription.unsubscribe(topicGetRobotOdometry)
    


        if (message.topic == topicGetArUcoRobotBack):
            # unpack the message according to its format
            frameTransf = message.unpack(FrameTransformation)
            tensor = frameTransf.tf
            #print("Space saw Aruco on Pepper back")
            # get the transformation matrix corresponding to the current pose of the robot corrected when
            # it sees an ArUco marker
            arUcoToWorld = np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
            pepperPose = arUcoToWorld*robotToArUco
            robotOriginToWorld = pepperPose*inv(lastOdometry)
            subscription.unsubscribe(topicGetArUcoRobotBack)
            sourceX = pepperPose[0,3]
            sourceY = pepperPose[1,3]
            print("x= ",sourceX," y= ",sourceY)
            robotLocalized = True
        ##################################
        
else: 
    sourceX = 2
    sourceY = 19


# Goal in the World Frame
goalX = 0.5
goalY = 18.5
print("Goal: ",goalX,"  ",goalY)
#goalWorldFrame = math.reshape(math.matrix([goalX, goalY, 0, 1]), [4, 1])
goalWorldFrame = np.matrix([goalX, goalY, 0, 1]).T
robot_size = 0.6
N_KNN = 40 # number of edge from one sampled point
MAX_EDGE_LEN = 3 # [m] Maximum edge length
show_animation = False

# Create obstacles
'''
ox, oy = create_virtualObstacles()


rx, ry = PRM_planning(mapFile,sourceX, sourceY, goalX, goalY, ox, oy, robot_size, step, N_KNN, MAX_EDGE_LEN)

# Check if a path was found/home/raquel/ProgrammingIS/learningISBristol//
assert len(rx) != 0, 'Cannot found path'

# Read map just to plot 
map_x, map_y = read_map(mapFile,step)

# Plot map points and obstacles
if show_animation:
    plt.plot(ox, oy, ".k")
    plt.plot(sourceX, sourceY, "^r")
    plt.plot(goalX, goalY, "^c")
    plt.plot(map_x, map_y, ".b")
    plt.grid(True)
    plt.axis("equal")
    plt.plot(rx, ry, "-r")
    plt.show()

'''

#sys.exit(0)

subscription.subscribe(topicGetArUcoRobotBack)
subscription.subscribe(topicGetRobotOdometry)

try:
    while True:
        # listen the channelprint(frameArUcoRobot)
        message = channel.consume()
        # Check if the message received is the robot's odometry - FrameTransformation type
        if (message.topic == topicGetRobotOdometry):
            # unpack the message according to its format
            frameTransf = message.unpack(FrameTransformation)
            tensor = frameTransf.tf
            
            # get the transformation matrix corresponding to the current rotation and position of the robot
            lastOdometry = np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
            pepperPose = robotOriginToWorld*lastOdometry
            '''
            # Convert the transformation matrix to tensor to be sent as a message
            msgContent = to_tensor(np.asarray(pepperPose))
            messageRobotPose.pack(msgContent)
            channel.publish(messageRobotPose)
            '''       
            #print(pepperPose)

        elif (message.topic == topicGetArUcoRobotBack):
            # unpack the message according to its format
            frameTransf = message.unpack(FrameTransformation)
            tensor = frameTransf.tf
            #print("Space saw Aruco on Pepper back")
            
            # get the transformation matrix corresponding to the current pose of the robot corrected when
            # it sees an ArUco marker
            
            arUcoToWorld = np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
            pepperPose = arUcoToWorld*robotToArUco
            robotOriginToWorld = pepperPose*inv(lastOdometry)
            #print("x= ",pepperPose[0,3]," y= ",pepperPose[1,3])
        
        if goalWorldFrame.size > 0:
            # matrix Inverse
            toPepperFrame = inv(pepperPose)
            # transform the goal from the world frame to the robot frame
            goalPepperFrame = toPepperFrame*goalWorldFrame
            posX = float(goalPepperFrame[0])
            posY = float(goalPepperFrame[1])
            posZ = float(goalPepperFrame[2])

            if (True):
                goal = Position()
                goal.x = posX
                goal.y = posY
                #goal.z = posZ
                goal.z = 0
                #print("goalX = ",goal.x," goalY = ",goal.y,"goalZ = ",goal.z)
                goalMessage = Message()
                goalMessage.pack(goal)
                goalMessage.topic = "RobotGateway.0.NavigateTo"
            else: 
                goal = Pose()
                goal.position.x = posX
                goal.position.y = posY
                goal.position.z = posZ
                #print(goal.position.x,"#",goal.position.y,"#",goal.position.z)
                goal.orientation.yaw = 90*3.14/180
                goal.orientation.pitch = 0
                goal.orientation.roll = 0
                goalMessage = Message()
                goalMessage.pack(goal)
                goalMessage.topic = "RobotGateway.0.MoveTo"


            channel.publish(goalMessage)
        dist = math.sqrt(posX**2 + posY**2)
        print(dist)
        if dist < 0.4 :
            print("Goal achieved")
            #motion_service.stopMove()
            break
    
    goal = Position()
    goal.x = 0
    goal.y = 0
    goal.z = 0
    #print("goalX = ",goal.x," goalY = ",goal.y,"goalZ = ",goal.z)
    goalMessage = Message()
    goalMessage.pack(goal)
    goalMessage.topic = "RobotGateway.0.NavigateTo"
    channel.publish(goalMessage)

    #motion_service.stopMove()
    

except KeyboardInterrupt:
    #motion_service.stopMove()
    goal = Position()
    goal.x = 0
    goal.y = 0
    goal.z = 0
    #print("goalX = ",goal.x," goalY = ",goal.y,"goalZ = ",goal.z)
    goalMessage = Message()
    goalMessage.pack(goal)
    goalMessage.topic = "RobotGateway.0.NavigateTo"
    channel.publish(goalMessage)

    


