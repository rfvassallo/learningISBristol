
from is_wire.core import Channel, Message, Subscription
from is_msgs.common_pb2 import Position, Pose
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

subscription.subscribe("FrameTransformation.2000.2001")
subscription.subscribe("FrameTransformation.2001.1003")


# Goal in the World Frame
# Change the goals value to make the robot move to different coordinates.
goalX = 0
goalY = 0
#goalWorldFrame = math.reshape(math.matrix([goalX, goalY, 0, 1]), [4, 1])
goalWorldFrame = np.matrix([goalX, goalY, 0, 1]).T

# Initialize transformation matrices used for storing odometry and correction
pepperPose = np.identity(4)
robotToWorld = np.identity(4)

# Change this variable to test between NavigateTo and MoveTo
useNavigate = True

try:
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
        
        elif (message.topic == "FrameTransformation.2001.1003"):
            # unpack the message according to its format
            frameTransf = message.unpack(FrameTransformation)
            tensor = frameTransf.tf
            print("Pepper saw the ArUco")
            
            # get the transformation matrix corresponding to the current pose of the robot corrected when
            # it sees an ArUco marker
            robotToWorld = np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
        
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
                goal.z = posZ
                #print(goal.x,"#",goal.y,"#",goal.z)
                goalMessage = Message()
                goalMessage.pack(goal)
                goalMessage.topic = "RobotGateway.0.NavigateTo"
            else: 
                goal = Pose()
                goal.position.x = posX
                goal.position.y = posY
                goal.position.z = posZ
                goal.orientation.yaw = 90*3.14/180
                goal.orientation.pitch = 0
                goal.orientation.roll = 0
                goalMessage = Message()
                goalMessage.pack(goal)
                goalMessage.topic = "RobotGateway.0.MoveTo"


            channel.publish(goalMessage)
        if math.sqrt(posX**2 + posY**2) < 0.40 :
            print("Goal achieved")
            break

except KeyboardInterrupt:
    pass


