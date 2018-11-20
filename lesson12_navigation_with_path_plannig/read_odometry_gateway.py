from is_wire.core import Channel, Message, Subscription
from is_msgs.camera_pb2 import FrameTransformation
import numpy as np
import qi



session = qi.Session()
try:
    session.connect("tcp://10.10.0.111:9559")
except RuntimeError:
    print ("Can't connect to Naoqi")

motion_service  = session.service("ALMotion")


# Create a channel to connect to the broker
channel = Channel("amqp://10.10.2.20:30000")
# Create a subscription 
subscription = Subscription(channel)
topicGetRobotOdometry = "FrameTransformation.2000.2001"
subscription.subscribe(topicGetRobotOdometry)


try:
    while True:
        message = channel.consume()
        if (message.topic == topicGetRobotOdometry):
            # unpack the message according to its format
            frameTransf = message.unpack(FrameTransformation)
            tensor = frameTransf.tf
            # get the transformation matrix corresponding to the current rotation and position of the robot
            lastOdometry = np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
            print("######################################")
            print("Last odometry")
            print(lastOdometry)
            print("######################################")
            result = motion_service.getRobotPosition(True)
            print("######################################")
            print ("Odometry - Robot API")
            print (result)
            print ("#############################")
       
except KeyboardInterrupt:
    pass

