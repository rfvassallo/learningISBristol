from is_wire.core import Channel, Message, Subscription
from is_msgs.common_pb2 import Position, Pose
from is_msgs.camera_pb2 import FrameTransformation
import numpy as np
import numpy.matlib
from numpy.linalg import inv
import argparse

#import time


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


    

def main(args):

    # Create a channel to connect to the broker
    channel = Channel("amqp://10.10.2.20:30000")
    # Create a subscription 
    subscription = Subscription(channel)

    with_robot = True
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


    target = args["target"]
    gx = target[0]
    gy = target[1]


    goalWorldFrame = np.matrix([gx,gy, 0, 1]).T
    toPepperFrame = inv(pepperPose)
    # transform the goal from the world frame to the robot frame
    goalPepperFrame = toPepperFrame*goalWorldFrame
    goal = Position()
    goal.x = float(goalPepperFrame[0])
    goal.y = float(goalPepperFrame[1])
    goal.z = 0    #float(goafrom is_msgs.camera_pb2 import FrameTransformationlPepperFrame[2])

    print("First comand")
    print("goalX = ",goal.x," goalY = ",goal.y,"goalZ = ",goal.z)
    goalMessage = Message()
    goalMessage.pack(goal)
    goalMessage.topic = "RobotGateway.0.NavigateTo"
    channel.publish(goalMessage)


    '''
    #time.sleep(2)

    goal = Position()
    goal.x = 0.0
    goal.y = -2.0
    goal.z = 0
    print("Second comand")
    print("goalX = ",goal.x," goalY = ",goal.y,"goalZ = ",goal.z)
    goalMessage = Message()
    goalMessage.pack(goal)
    goalMessage.topic = "RobotGateway.0.NavigateTo"
    channel.publish(goalMessage)


    goal = Position()
    goal.x = -2.0
    goal.y = 0.0
    goal.z = 0
    print("Second comand")
    print("goalX = ",goal.x," goalY = ",goal.y,"goalZ = ",goal.z)
    goalMessage = Message()
    goalMessage.pack(goal)
    goalMessage.topic = "RobotGateway.0.NavigateTo"
    channel.publish(goalMessage)


    goal = Position()
    goal.x = 2.0
    goal.y = 0.0
    goal.z = 0
    print("Second comand")
    print("goalX = ",goal.x," goalY = ",goal.y,"goalZ = ",goal.z)
    goalMessage = Message()
    goalMessage.pack(goal)
    goalMessage.topic = "RobotGateway.0.NavigateTo"
    channel.publish(goalMessage)

    '''


if __name__ == '__main__':
    # construct the argument parse and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-t", "--target", nargs='+', type=float,
        help="x and y coordinates of the target for the path planning")
    args = vars(ap.parse_args())

    main(args)