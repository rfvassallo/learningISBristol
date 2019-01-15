from __future__ import print_function
from is_wire.core import Channel, Subscription
from is_msgs.common_pb2 import Tensor
from is_msgs.camera_pb2 import FrameTransformations
import numpy as np

# Connect to the broker
channel = Channel("amqp://10.10.2.23:30000")


# Subscribe to the desired topic(s)
subscription = Subscription(channel)
topic02 = "ArUco.7.FrameTransformations"   # get relation between camera 7 and the ArUco detected
subscription.subscribe(topic02)


# Blocks forever waiting for message


diretArucoPose = np.matrix('0,0,0,0;0,0,0,0;0,0,0,0;0,0,0,0')
matrix = np.matrix('0,0,0,0;0,0,0,0;0,0,0,0;0,0,0,0')


# Compare the Frame Transforamtion obtained directly from the Aruco detecion service  when an Aruco Marker is seen
# with the calculated Frame Transforamtion when considering the camera extrinsic matrix and the Aruco Marker position in the world
while True: 
  message = channel.consume()

  
  # Get the Frame Transformation that represents the pose of the Aruco mark in the camera 7 reference frame 
  # Then multiply by the extrinsic matrix of camera 7 to obtain the Aruco pose in world reference frame
  if message.topic == topic02 :
    # Message returns a list of Frame Transformations
    frameTransList = message.unpack(FrameTransformations)
    # Check if there is any element (Frame Transformation) in the list
    if frameTransList.tfs :
      # Go through the list 
      for frameTrans in frameTransList.tfs :
        # Check if the Frame transformation has a tensor containing the transformation matrix
        if frameTrans.HasField("tf") :
          tensor = frameTrans.tf
          print(frameTrans)  
          #arucoID = frameTrans - 100
            




