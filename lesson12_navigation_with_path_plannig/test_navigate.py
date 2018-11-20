from is_wire.core import Channel, Message, Subscription
from is_msgs.common_pb2 import Position, Pose
from is_msgs.camera_pb2 import FrameTransformation
import time
import numpy as np
import qi




# Create a channel to connect to the broker
channel = Channel("amqp://10.10.2.20:30000")
# Create a subscription 
subscription = Subscription(channel)
topicGetRobotOdometry = "FrameTransformation.2000.2001"
subscription.subscribe(topicGetRobotOdometry)

session = qi.Session()
try:
    session.connect("tcp://10.10.0.111:9559")
except RuntimeError:
    print ("Can't connect to Naoqi")

motion_service  = session.service("ALMotion")
result = motion_service.getRobotPosition(True)
print ("First odometry - Robot API", result)

while True:
    message = channel.consume()
    if (message.topic == topicGetRobotOdometry):
        # unpack the message according to its format
        frameTransf = message.unpack(FrameTransformation)
        tensor = frameTransf.tf
        
        # get the transformation matrix corresponding to the current rotation and position of the robot
        lastOdometry = np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
        print("First odometry")
        print(lastOdometry)
        subscription.unsubscribe(topicGetRobotOdometry)
        break


subscription.subscribe(topicGetRobotOdometry)
goal = Position()
goal.x = 0.0
goal.y = 2.0
goal.z = 0
print("First comand")
print("goalX = ",goal.x," goalY = ",goal.y,"goalZ = ",goal.z)
goalMessage = Message()
goalMessage.pack(goal)
goalMessage.topic = "RobotGateway.0.NavigateTo"
channel.publish(goalMessage)

time.sleep(10)
result = motion_service.getRobotPosition(True)
print ("Second odometry - Robot API", result)

while True:
    message1 = channel.consume()
    if (message1.topic == topicGetRobotOdometry):
        # unpack the message according to its format
        frameTransf = message1.unpack(FrameTransformation)
        tensor = frameTransf.tf
        
        # get the transformation matrix corresponding to the current rotation and position of the robot
        lastOdometry = np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
        print("Second odometry")
        print(lastOdometry)
        subscription.unsubscribe(topicGetRobotOdometry)
        break

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



time.sleep(10)
subscription.subscribe(topicGetRobotOdometry)

result = motion_service.getRobotPosition(True)
print ("Third odometry - Robot API", result)


while True:
    message1 = channel.consume()
    if (message1.topic == topicGetRobotOdometry):
        # unpack the message according to its format
        frameTransf = message1.unpack(FrameTransformation)
        tensor = frameTransf.tf
        
        # get the transformation matrix corresponding to the current rotation and position of the robot
        lastOdometry = np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
        print("Third odometry")
        print(lastOdometry)
        subscription.unsubscribe(topicGetRobotOdometry)
        break

