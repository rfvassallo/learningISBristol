from is_msgs.common_pb2 import Tensor
from is_wire.core import Channel, Message, Subscription
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




# Create a channel to connect to the broker
channel = Channel("amqp://10.10.2.20:30000")
# Create a subscription 
subscription = Subscription(channel)

# Subscribe to get the relation from the initial reference frame where the robot was
# turned on and its current reference frame (dead reckoning odometry)
subscription.subscribe("FrameTransformation.2000.2001")


while True:
    # listen the channel
    message = channel.consume()
    # unpack the message according to its format
    tensor = message.unpack(Tensor)
    # get the transformation matrix corresponding to the current rotation and position of the robot
    matrix=np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
    posX = matrix[0,3]
    posY = matrix[1,3]
    posZ = matrix[2,3]
    strPosition = 'X: ' + str(posX) + '   Y: ' + str(posY) + '   Z: ' + str(posZ)
    rotationMatrix = matrix[0:3,0:3] 
    angles = rotationMatrixToEulerAngles(rotationMatrix)
    strRotation = 'Yaw: ' + str(angles[2]) + '   Pitch: ' + str(angles[1]) + '    Roll: ' + str(angles[0])

    print strPosition
    print strRotation
