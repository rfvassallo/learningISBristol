
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
sys.path.append('../lesson10_path_planning/')
from path_planning_PRM import PRM_planning, create_virtualObstacles, read_map




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

# Create a channel to connect to the broker
channel = Channel("amqp://10.10.2.20:30000")
# Create a subscription 
subscription = Subscription(channel)

# Subscribe to the following topics:
# - To get the robot's current position in relation where it was turned on, i.e., dead reckoning odometry
# - To get the position of the ArUco marker attached to the robot's back

robotArUco = 8  # ArUco marker ID attached to the robot's back
worldFrame = 1000    # Number of the world reference frame in the HPN Intelligent Space

topicGetRobotOdometry = "FrameTransformation.2000.2001"
topicGetArUcoRobotBack = "FrameTransformation."+str(robotArUco+100)+"."+str(worldFrame)
subscription.subscribe(topicGetRobotOdometry)
subscription.subscribe(topicGetArUcoRobotBack)



# Initialize transformation matrices used for storing odometry and correction
pepperPose = np.identity(4)
robotOriginToWorld = np.identity(4)
lastOdometry = np.identity(4)

# Load the frame transformation between the ArUco marker on the robot's back and the robot's base frame.
fileName = "frameArUcoRobot.dat"
arUcoToRobot = read_frameTransformation(fileName)
robotToArUco = inv(arUcoToRobot)


# Parameters for Path-planning
mapFile = "map1311.dat"
step = 2
robot_size = 0.6
N_KNN = 40 # number of edge from one sampled point
MAX_EDGE_LEN = 3 # [m] Maximum edge length
show_animation = False

# Goal in the World Frame ###### CHANGE THIS TO ARGUMENT
goalX = 1.5
goalY = 19
print("Goal: ",goalX,"  ",goalY)
goalWorldFrame = np.matrix([goalX, goalY, 0, 1]).T


# Localize the robot before start path planning
robotLocalized = False

while not robotLocalized:
    # Source must be the current position of the robot in the world frame
    message = channel.consume()
    if (message.topic == topicGetRobotOdometry):
        # unpack the message according to its format
        frameTransf = message.unpack(FrameTransformation)
        tensor = frameTransf.tf
        
        # get the transformation matrix corresponding to the current rotation and position of the robot
        lastOdometry = np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
        # unsubscribe to not accumulate messages
        subscription.unsubscribe(topicGetRobotOdometry)



    if (message.topic == topicGetArUcoRobotBack):
       
        # unpack the message according to its format
        frameTransf = message.unpack(FrameTransformation)
        tensor = frameTransf.tf
        # get the frame transformation betweeb the ArUco marker on the robot's back and the world
        arUcoToWorld = np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
        # calculates the robot pose in the world frame
        pepperPose = arUcoToWorld*robotToArUco
        # calculate the frame transformation needed to correct robot's odometry while the ArUco marker is not seen by the intelligent space
        robotOriginToWorld = pepperPose*inv(lastOdometry)
        # unsubscribe to not accumulate messages
        subscription.unsubscribe(topicGetArUcoRobotBack)
        sourceX = pepperPose[0,3]
        sourceY = pepperPose[1,3]
        print("x= ",sourceX," y= ",sourceY)
        robotLocalized = True

        
#sys.exit(0)


# Create obstacles
ox, oy = create_virtualObstacles()

# Call path planning
# rx, ry contains the positions in the path
rx, ry = PRM_planning(mapFile,sourceX, sourceY, goalX, goalY, ox, oy, robot_size, step, N_KNN, MAX_EDGE_LEN)

# Check if a path was found/home/raquel/ProgrammingIS/learningISBristol//
assert len(rx) != 0, 'Cannot found path'


# Plot map points and obstacles
if show_animation:
    map_x, map_y = read_map(mapFile,step)
    plt.plot(ox, oy, ".k")
    plt.plot(sourceX, sourceY, "^r")
    plt.plot(goalX, goalY, "^c")
    plt.plot(map_x, map_y, ".b")
    plt.grid(True)
    plt.axis("equal")
    plt.plot(rx, ry, "-r")
    plt.plot(rx,ry,"og")
    plt.show()




rx = rx[::-1]
ry = ry[::-1]
print(rx)
print(ry)

#sys.exit(0)


# Subscribe to the previous topics
subscription.subscribe(topicGetArUcoRobotBack)
subscription.subscribe(topicGetRobotOdometry)

i=0
k=0
dist = 100.0

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
                
            #print(pepperPose)

        elif (message.topic == topicGetArUcoRobotBack):

            print("Odometry Corrected")
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
        
        
        # matrix Inverse
        toPepperFrame = inv(pepperPose)
        # transform the goal from the world frame to the robot frame
        goalWorldFrame[0] = rx[i]
        goalWorldFrame[1] = ry[i]
        goalPepperFrame = toPepperFrame*goalWorldFrame
        posX = float(goalPepperFrame[0])
        posY = float(goalPepperFrame[1])
        #posZ = float(goalPepperFrame[2])

        distPrevious = dist
        dist = math.sqrt(posX**2 + posY**2)
        #print(dist)
        if dist < 0.2 :
            i = i + 1
            print(dist)
            print("Path index: ", i, "  of ", len(rx))
            if i >= len(rx):
                print("Goal achieved")
                break
            print("Next point in the path")
            goalWorldFrame[0] = rx[i]
            goalWorldFrame[1] = ry[i]
            goalPepperFrame = toPepperFrame*goalWorldFrame

            goal = Pose()
            goal.position.x = float(goalPepperFrame[0])
            goal.position.y = float(goalPepperFrame[1])
            goal.position.z = 0
            goal.orientation.yaw = math.atan2(goal.position.y,goal.position.x)
            goal.orientation.pitch = 0
            goal.orientation.roll = 0
            
            goalMessage = Message()
            goalMessage.pack(goal)
            goalMessage.topic = "RobotGateway.0.MoveTo"
            channel.publish(goalMessage)
            
            '''
            goal = Position()
            goal.x = float(goalPepperFrame[0])
            goal.y = float(goalPepperFrame[1])
            goal.z = 0
            print("goalX = ",goal.x," goalY = ",goal.y,"goalZ = ",goal.z)
            goalMessage = Message()
            goalMessage.pack(goal)
            goalMessage.topic = "RobotGateway.0.NavigateTo"
            channel.publish(goalMessage)
            '''
        
        elif abs(dist-distPrevious) < 0.005:
            k = k + 1
            if k == 20:
                print(dist)
                k=0
                print("Ooops.... I got stuck.... I will try to move")
                goalWorldFrame[0] = rx[i]
                goalWorldFrame[1] = ry[i]
                goalPepperFrame = toPepperFrame*goalWorldFrame

                goal = Pose()
                goal.position.x = float(goalPepperFrame[0])
                goal.position.y = float(goalPepperFrame[1])
                goal.position.z = 0
                goal.orientation.yaw = math.atan2(goal.position.y,goal.position.x)
                goal.orientation.pitch = 0
                goal.orientation.roll = 0
                goalMessage = Message()
                goalMessage.pack(goal)
                goalMessage.topic = "RobotGateway.0.MoveTo"
                channel.publish(goalMessage)
            elif k > 200:
                print ("Navigation Failed.")
                break
                    

except KeyboardInterrupt:
    
    goal = Position()
    goal.x = 0
    goal.y = 0
    goal.z = 0
    goalMessage = Message()
    goalMessage.pack(goal)
    goalMessage.topic = "RobotGateway.0.NavigateTo"
    channel.publish(goalMessage)

    


