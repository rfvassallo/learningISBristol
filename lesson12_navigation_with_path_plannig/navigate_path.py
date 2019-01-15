
from is_wire.core import Channel, Message, Subscription
from is_msgs.common_pb2 import Position, Pose
from is_msgs.camera_pb2 import FrameTransformation
from is_msgs.robot_pb2 import RobotConfig
from google.protobuf.struct_pb2 import Struct
import matplotlib.pyplot as plt
import numpy as np
import numpy.matlib
from numpy.linalg import inv
import math
import sys
import argparse
import time
from datetime import datetime
from dateutil.relativedelta import relativedelta
sys.path.append('../lesson10_path_planning/')
from path_planning_PRM import PRM_planning, create_virtualObstacles, read_map
import socket


###################################################
# Unpack message to extract the Frame Transformation carried in its content 
def unpackFrameTransformation (message):
    # unpack the message according to the Frame Transformation format
    frameTransf = message.unpack(FrameTransformation)
    tensor = frameTransf.tf

    # get the transformation matrix from the tensor in the message
    transfMatrix = np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
    return transfMatrix
###################################################

###################################################
# Apply frame transformation to a point and return its new coordinates 
def changeRefFrame(x,y,z,transfMatrix):
    # transform the poitn (x,y) to the frame represented by the transformation matrix
    origPoint = np.matrix([x, y, z, 1]).T
    transfPoint = transfMatrix*origPoint
    posX = float(transfPoint[0])
    posY = float(transfPoint[1])
    posZ = float(transfPoint[2])
    return posX, posY, posZ
####################################################

###################################################
# Publish the message to execute the robot command MoveTo    
def commandMoveTo(posX,posY,channel):

    goal = Pose()
    goal.position.x = posX
    goal.position.y = posY
    goal.position.z = 0
    goal.orientation.yaw = math.atan2(goal.position.y,goal.position.x)
    goal.orientation.pitch = 0
    goal.orientation.roll = 0
    
    goalMessage = Message()
    goalMessage.pack(goal)
    goalMessage.topic = "RobotGateway.0.MoveTo"
    channel.publish(goalMessage)
###################################################

##################################################
# Publish the massage to execute the robot command NavigateTo    
def commandNavigateTo(posX,posY,channel):
    goal = Position()
    goal.x = posX
    goal.y = posY
    goal.z = 0
    
    goalMessage = Message()
    goalMessage.pack(goal)
    goalMessage.topic = "RobotGateway.0.NavigateTo"
    channel.publish(goalMessage)
##################################################


###################################################
# Read frame transforamation from a file
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
######################################################

def makeTurn (linear,angular,channel):
    # Create message with command for 
    # # linear and angular velocities  
    message = Message()
    robotConfig = RobotConfig()
    robotConfig.speed.linear = linear
    robotConfig.speed.angular = angular
    message.pack(robotConfig)
    message.topic = "RobotGateway.0.SetConfig"
    # Public message
    channel.publish(message)

    t_a = datetime.now()
    t_b = t_a
    t_diff = relativedelta(t_b, t_a)
    while t_diff.seconds < 4:
        t_b = datetime.now()
        t_diff = relativedelta(t_b, t_a)

    robotConfig.speed.linear = 0
    robotConfig.speed.angular = 0
    message.pack(robotConfig)
    message.topic = "RobotGateway.0.SetConfig"
    # Public message
    channel.publish(message)

    t_a = datetime.now()
    t_b = t_a
    t_diff = relativedelta(t_b, t_a)
    while t_diff.seconds < 2:
        t_b = datetime.now()
        t_diff = relativedelta(t_b, t_a)


def goBack (linear,angular,channel):
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
    t_a = datetime.now()
    t_b = t_a
    t_diff = relativedelta(t_b, t_a)
    while t_diff.seconds < 4:
        t_b = datetime.now()
        t_diff = relativedelta(t_b, t_a)

    robotConfig.speed.linear = 0
    robotConfig.speed.angular = 0
    message.pack(robotConfig)
    message.topic = "RobotGateway.0.SetConfig"
    # Public message
    channel.publish(message)



def awarenessOff(channel):
    
    message = Message()
    condition = Struct()
    condition["enabled"] = False
    message.pack(condition)
    message.topic = "RobotGateway.0.SetAwareness"
    channel.publish(message) 
   

def awarenessOn(channel):
    
    message = Message()
    condition = Struct()
    condition["enabled"] = True
    message.pack(condition)
    message.topic = "RobotGateway.0.SetAwareness"
    channel.publish(message)
  
#######################################################
# Main program

def navigate(goalX,goalY,robotArUco, worldFrame, mapFile,step,robot_size,N_KNN,MAX_EDGE_LEN,show_path):


    # Create a channel to connect to the broker
    channel = Channel("amqp://10.10.2.23:30000")
    # Create a subscription 
    subscription = Subscription(channel)

    # Subscribe to the following topics:
    # - To get the robot's current position in relation where it was turned on, i.e., dead reckoning odometry
    # - To get the position of the ArUco marker attached to the robot's back

    #robotArUco = 8  # ArUco marker ID attached to the robot's back
    #worldFrame = 1000    # Number of the world reference frame in the HPN Intelligent Space

    awarenessOff(channel)
    time.sleep(6)  # to wait some time to be sure the awareness if off

    topicGetRobotOdometry = "FrameTransformation.2000.2001"
    topicGetArUcoRobotBack = "FrameTransformation."+str(robotArUco+100)+"."+str(worldFrame)
    

    subscription.subscribe(topicGetRobotOdometry)
    subscription.subscribe(topicGetArUcoRobotBack)




    # Initialise transformation matrices used for storing odometry and correction
    pepperPose = np.identity(4)
    robotOriginToWorld = np.identity(4)
    lastOdometry = np.identity(4)

    # Load the frame transformation between the ArUco marker on the robot's back and the robot's base frame.
    fileName = "frameArUcoRobot.dat"
    arUcoToRobot = read_frameTransformation(fileName)
    robotToArUco = inv(arUcoToRobot)

    print("Goal: ",goalX,"  ",goalY)


    # Localize the robot before start path planning
    
    robotLocalized = False
    notSeen = 0
    count = 0

    while not robotLocalized:
        # Source must be the current position of the robot in the world frame
        message = channel.consume()
        if (message.topic == topicGetRobotOdometry):
            print("reading odometry")
            # get the transformation matrix corresponding to the current rotation and position of the robot
            lastOdometry = unpackFrameTransformation (message)
            notSeen = notSeen + 1
            print(notSeen)
            
        

        if (message.topic == topicGetArUcoRobotBack):
            print("correcting odometry")
            # get the frame transformation betweeb the ArUco marker on the robot's back and the world
            arUcoToWorld = unpackFrameTransformation (message)
            # calculates the robot pose in the world frame
            pepperPose = arUcoToWorld*robotToArUco
            # calculate the frame transformation needed to correct robot's odometry while the ArUco marker is not seen by the intelligent space
            robotOriginToWorld = pepperPose*inv(lastOdometry)
            
            sourceX = pepperPose[0,3]
            sourceY = pepperPose[1,3]
            print("x= ",sourceX," y= ",sourceY)
            robotLocalized = True
            notSeen = 0

        
        
        if notSeen > 30:
            notSeen = 0
            count = count + 1            
            # unsubscribe to not accumulate messages
            subscription.unsubscribe(topicGetRobotOdometry)
            subscription.unsubscribe(topicGetArUcoRobotBack)
            
            if count > 4:
                print("I can't get localized by the Intelligent Space.")
                print("Please take me to a spot where the marker on my back can be seen by one of the cameras.")
                sys.exit(0)
            
            makeTurn(0,0.3,channel)
            subscription.subscribe(topicGetRobotOdometry)
            subscription.subscribe(topicGetArUcoRobotBack)

            
    # unsubscribe to not accumulate messages
    subscription.unsubscribe(topicGetRobotOdometry)
    subscription.unsubscribe(topicGetArUcoRobotBack)
    
    

    #sys.exit(0)


    # Create obstacles
    ox, oy = create_virtualObstacles()

    # Call path planning
    # rx, ry contains the positions in the path
    rx, ry = PRM_planning(mapFile,sourceX, sourceY, goalX, goalY, ox, oy, robot_size, step, N_KNN, MAX_EDGE_LEN)

    # Check if a path was found/home/raquel/ProgrammingIS/learningISBristol//
    #assert len(rx) != 0, 'Cannot found path'
    if len(rx) ==0:
        print('Cannot find path')
        raise SystemError
        
       

    #sys.exit(0)

    # Plot map points and obstacles
    if show_path:
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



    # Reverse the order of the path (list) returned by the path-planning algorithm
    # The original list contains the goal at the beginning and the source at the end. We need the reverse
    rx = rx[::-1]
    ry = ry[::-1]
    print(rx)
    print(ry)

    #sys.exit(0)


    # Subscribe to the previous topics
    subscription.subscribe(topicGetRobotOdometry)
    subscription.subscribe(topicGetArUcoRobotBack)


    i=0
    k=0
    stuck = 0
    dist = 100.0
    threshold = 0.2

    try:
        while True:


            try:
                        
                # listen the channelprint(frameArUcoRobot)
                message = channel.consume(timeout=0.5)
                # Check if the message received is the robot's odometry - FrameTransformation type
                if (message.topic == topicGetRobotOdometry):
                    # get the transformation matrix corresponding to the current rotation and position of the robot
                    lastOdometry = unpackFrameTransformation (message)
                    pepperPose = robotOriginToWorld*lastOdometry
                        
                    #print(pepperPose)

                elif (message.topic == topicGetArUcoRobotBack):

                    print("Odometry Corrected")
                    # get the transformation matrix corresponding to the current pose of the robot corrected when
                    # it sees an ArUco marker
                    arUcoToWorld = unpackFrameTransformation (message)
                    pepperPose = arUcoToWorld*robotToArUco
                    robotOriginToWorld = pepperPose*inv(lastOdometry)
            except socket.timeout:

                print("Time out")
                    
            
            # matrix Inverse
            toPepperFrame = inv(pepperPose)
            # transform the goal from the world frame to the robot frame
            posX, posY, posZ = changeRefFrame(rx[i],ry[i],0,toPepperFrame)
            
            distPrevious = dist
            dist = math.sqrt(posX**2 + posY**2)
            #print(dist)
            # If distance to current goal is less than the threshold, pick the next point in the path to be the next goal
            if dist < threshold :
                i = i + 1
                stuck = 0
                print(dist)
                print("Path index: ", i, "  of ", len(rx))

                if i == (len(rx)-1):
                    threshold = 0.5

                # If that was the last point in the path, finish navigation. Goal achieved.
                if i >= len(rx):
                    
                    print("Goal achieved")
                    break
                # If not the last point in the path, get the next point and converte it to robot frame
                print("Next point in the path")
                posX, posY, posZ = changeRefFrame(rx[i],ry[i],0,toPepperFrame)
                # Command robot to move
                commandMoveTo(posX,posY,channel)
                #commandNavigateTo(posX,posY,channel)
                
            # If distance to current goal is greater than the threshold, check if robot is stopped 
            # To check if robot is stopped, calculate the difference between current distance and previous one   
            elif abs(dist-distPrevious) < 0.005:  # if difference is less than 0.5 cm
                k = k + 1  # accumulate 
                if k == 20: #see if situation remains for 20 times
                    # Robot is stuck. Let's send a move command again
                    print(dist)
                    k=0
                    print("Ooops.... I got stuck.... I will try to move. Stuck = ", stuck)
                    posX, posY, posZ = changeRefFrame(rx[i],ry[i],0,toPepperFrame)
                    
                    stuck = stuck +1
                    if stuck == 4:

                        goBack(-10,0,channel)
                        stuck =0
                        posX, posY, posZ = changeRefFrame(rx[i],ry[i],0,toPepperFrame)
                        # Command robot to move
                        commandMoveTo(posX,posY,channel)
                    else: 
                        commandMoveTo(posX,posY,channel)
            
    
                        
        

    except KeyboardInterrupt:
        
        commandMoveTo(0,0,channel)
        
    #awarenessOff(channel)
    
    awarenessOn(channel)
    time.sleep(6)
    #awarenessOn(channel)


def main(args):
    print(__file__ + " start!!")
    # parameters
    mapFile = args["mapfile"]
    robotArUco = args["aruco"]
    worldFrame = args["worldframe"]
    step = args["grain"]
    goal = args["target"]
    gx = goal[0]
    gy = goal[1]
    rr = args["robotradius"]
    N_KNN = args["nknn"]  # number of edge from one sampled point
    MAX_EDGE_LEN = args["maxedge"] # [m] Maximum edge length
    show_path = False

    navigate(gx,gy,robotArUco, worldFrame, mapFile,step,rr,N_KNN,MAX_EDGE_LEN,show_path)





if __name__ == '__main__':
    # construct the argument parse and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-m", "--mapfile", default="../lesson09_mapping_with_ArUco/map1311.dat",
        help="Name of the file that will contain the map")
    ap.add_argument("-g", "--grain", type=int, default=2,
        help="Granularity for creating the roadmap from the points saved in the map file")
    ap.add_argument("-t", "--target", nargs='+', type=float,
        help="x and y coordinates of the target for the path planning")
    ap.add_argument("-a","--aruco",type=int, default=8,
        help="Code of the ArUco marker used as robot ID and attached to robot's back")
    ap.add_argument("-w","--worldframe",type=int, default=1000,
        help="Code of the World Reference Frame")    
    ap.add_argument("-r", "--robotradius", type=float, default=0.6,
        help="Robot radius size")
    ap.add_argument("-n", "--nknn", type=int, default= 20,
        help="Number of edges (KNN) from one sampled point used for building the roadmap")
    ap.add_argument("-e", "--maxedge", type=int, default= 3,
        help="Maximum edge length [m] used for avoiding collisions")

    args = vars(ap.parse_args())

    main(args)
