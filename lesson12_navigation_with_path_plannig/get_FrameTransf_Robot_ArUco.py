
from is_wire.core import Channel, Message, Subscription
#from is_msgs.common_pb2 import Position, Pose
from is_msgs.camera_pb2 import FrameTransformation
import numpy as np#
from numpy.linalg import inv
import math


# Create a channel to connect to the broker
channel = Channel("amqp://10.10.2.20:30000")
# Create a subscription 
subscription = Subscription(channel)

# Subscribe to get the relation from the initial reference frame where the robot was
# turned on and its current reference frame (dead reckoning odometry)
##################subscription.subscribe("FrameTransformation.1000.2000")

robotArUco = 8 
worldFrame = 1003

getRobotOdometry = "FrameTransformation.2000.2001"
getRobotWorldPosition = "FrameTransformation.2001." + str(worldFrame)
getArUcoRobotBack = "FrameTransformation."+str(robotArUco+100)+"."+str(worldFrame)
subscription.subscribe(getRobotOdometry)
subscription.subscribe(getRobotWorldPosition)
subscription.subscribe(getArUcoRobotBack)



# Initialize transformation matrices used for storing odometry and correction
robotOdometry = np.identity(4)
robotWorldPosition = np.identity(4)
arUcoRobotBack = np.identity(4)
n = 1
k = 1
fileName = "frameArUcoRobot.dat"

testPoint = np.matrix([[0],[0],[0],[1]])



try:
    while True:
        # listen the channel
        message = channel.consume()
        # Check if the message received is the robot's odometry - FrameTransformation type
        if (message.topic == getRobotOdometry):
            # unpack the message according to its format
            frameTransf = message.unpack(FrameTransformation)
            tensor = frameTransf.tf
            #print("Got robot Odometry")
            # get the transformation matrix corresponding to the current rotation and position of the robot
            robotOdometry = np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
            
        elif (message.topic == getRobotWorldPosition):
            # unpack the message according to its format
            frameTransf = message.unpack(FrameTransformation)
            tensor = frameTransf.tf
            #print("Got robot world position")
            # get the transformation matrix corresponding to the current rotation and position of the robot
            aux = np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
            if n ==1:
                robotWorldPosition = aux
                n = 2
            robotWorldPosition = (robotWorldPosition+aux)/2
            

           
        elif (message.topic == getArUcoRobotBack):
            # unpack the message according to its format
            frameTransf = message.unpack(FrameTransformation)
            tensor = frameTransf.tf
            #print("Got aruco on robot back")
            # get the transformation matrix corresponding to the current rotation and position of the robot
            aux2 = np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
            if k==1:
                arUcoRobotBack = aux2
                k=2
            arUcoRobotBack = (arUcoRobotBack+aux2)/2

        
        
        frameTransfRobotArUco = inv(arUcoRobotBack)*robotWorldPosition*robotOdometry
        print("##### Frame Transformation between the Robot Base and the ArUco Marker on its back #####")
        print (inv(frameTransfRobotArUco))
        
        validate = inv(frameTransfRobotArUco)*testPoint
        print(validate)
        print("########################################################################################")


        
except KeyboardInterrupt:
    frameArUcoRobot = inv(frameTransfRobotArUco)
    with open(fileName, 'w') as f:
        f.write('{:.4f} {:.4f} {:.4f} {:.4f}\n'.format(frameArUcoRobot[0,0], frameArUcoRobot[0,1], frameArUcoRobot[0,2], frameArUcoRobot[0,3]))
        f.write('{:.4f} {:.4f} {:.4f} {:.4f}\n'.format(frameArUcoRobot[1,0], frameArUcoRobot[1,1], frameArUcoRobot[1,2], frameArUcoRobot[1,3]))
        f.write('{:.4f} {:.4f} {:.4f} {:.4f}\n'.format(frameArUcoRobot[2,0], frameArUcoRobot[2,1], frameArUcoRobot[2,2], frameArUcoRobot[2,3]))
        f.write('{:.4f} {:.4f} {:.4f} {:.4f}\n'.format(frameArUcoRobot[3,0], frameArUcoRobot[3,1], frameArUcoRobot[3,2], frameArUcoRobot[3,3]))
        
    


