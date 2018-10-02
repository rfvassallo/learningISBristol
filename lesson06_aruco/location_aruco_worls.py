from __future__ import print_function
from is_wire.core import Channel, Subscription
from is_msgs.common_pb2 import Tensor
from is_msgs.camera_pb2 import FrameTransformation
import numpy as np

# Connect to the broker
channel = Channel("amqp://guest:guest@10.10.2.20:30000")

# Subscribe to the desired topic(s)
subscription = Subscription(channel)
topic01 = "FrameTransformation.7.1003"
subscription.subscribe(topic01)
topic02 = "ArUco.7.Pose"
subscription.subscribe(topic02)
topic03 = "FrameTransformation.100.7.1003"
subscription.subscribe(topic03)

# ... subscription.subscribe(topic="Other.Topic")

# Blocks forever waiting for message

extrincMatrix = np.matrix('0,0,0,0;0,0,0,0;0,0,0,0;0,0,0,0')
diretArucoPose = np.matrix('0,0,0,0;0,0,0,0;0,0,0,0;0,0,0,0')
matrix = np.matrix('0,0,0,0;0,0,0,0;0,0,0,0;0,0,0,0')


# Compare the Frame Transforamtion obtained directly from the Aruco detecion service  when an Aruco Marker is seen
# with the calculated Frame Transforamtion when considering the camera extrinsic matrix and the Aruco Marker position in the world
while True: 
  message = channel.consume()

  # Get the extrinsic matrix that represents the pose (osition and orientation) of camera 7 in the reference frame 1003
  if message.topic == topic01 :
    tensor = message.unpack(Tensor)
    if tensor.HasField("shape") :
      extrincMatrix=np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
      print ("---------- Extrinsec Matrix  --------")
      print (extrincMatrix)
      print ("------------------------------------\n")

  # Get the Frame Transformation that represents the pose of the Aruco mark in the camera 7 reference frame   
  elif message.topic == topic03 :
    tensor = message.unpack(Tensor)
    if tensor.HasField("shape") :
      directArucoPose=np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
      print ("---------- Direct Pose of the Aruco Marker --------")
      print (directArucoPose)
      print ("-------------------------------------\n")

  # Get the transformation that represents the Aruco pose in the reference frame 1003.
  # Then multiply by the extrinsic matrix of camera 7 to obtain the Aruco pose in camera 7 reference frame  
  elif message.topic == topic02 :
    frameTrans = message.unpack(FrameTransformation)
    tensor = frameTrans.tf
    if frameTrans.HasField("tf") :                
      matrix = np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
      poseAruco = extrincMatrix*matrix    
      print ("---------- Aruco Pose in the camera 7 referential frame --------")
      print (poseAruco)
      print ("-------------------------------------\n")


