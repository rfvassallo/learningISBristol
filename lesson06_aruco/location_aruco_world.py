from __future__ import print_function
from is_wire.core import Channel, Subscription
from is_msgs.camera_pb2 import FrameTransformation, FrameTransformations
import numpy as np

# Connect to the broker
channel = Channel("amqp://10.10.2.23:30000")

# Subscribe to the desired topic(s)
subscription = Subscription(channel)
topic01 = "FrameTransformation.7.1003"	     # get relation between camera 7 and the world frame 1003
subscription.subscribe(topic01)
topic02 =  "ArUco.7.FrameTransformations"    # get relation between camera 7 and the ArUco detected
subscription.subscribe(topic02)
topic03 = "FrameTransformation.100.7.1003"   # get relation between the ArUco (ID 0) and the world frame 1003, through camera 7
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
    # Message returns a Frame Transformation which should have a tensor that represents the transformation matrix
    frameTransf = message.unpack(FrameTransformation)
    tensor = frameTransf.tf
    if tensor.HasField("shape") :
      extrincMatrix=np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
      print ("---------- Extrinsec Matrix  --------")
      print (extrincMatrix)
      print ("------------------------------------\n")

  # Get the Frame Transformation that represents the pose of the Aruco mark in the camera 7 reference frame 
  # Then multiply by the extrinsic matrix of camera 7 to obtain the Aruco pose in world reference frame
  elif message.topic == topic02 :
    # This message returns a list of Frame Transformations
    frameTransList = message.unpack(FrameTransformations)
    # Check if the list is not empty
    if frameTransList.tfs :
      # Go through the list
      for frameTrans in frameTransList.tfs :
        # If the current Frame Transformation has a tensor, we print the transformation
        # IMPORTANT: This program was done considering just the ArUco with code 0 (zero)
        #            So we are expecting to get just one Frame Transformation in the list 
        if frameTrans.HasField("tf") :
          tensor = frameTrans.tf                          
          matrix = np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
          poseAruco = extrincMatrix*matrix    
          print ("---------- Aruco Pose in the world referential frame seen by camera 7 --------")
          print (poseAruco)
          print ("-------------------------------------\n")

   # Get the transformation that represents the Aruco pose in the world reference frame 1003.
         
  elif message.topic == topic03 :
    # Message returns a Frame Transformation which should have a tensor that represents the transformation matrix
    frameTransf = message.unpack(FrameTransformation)
    tensor = frameTransf.tf
    if tensor.HasField("shape") :
      directArucoPose=np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
      print ("---------- Direct Pose of the Aruco Marker --------")
      print (directArucoPose)
      print ("-------------------------------------\n")



