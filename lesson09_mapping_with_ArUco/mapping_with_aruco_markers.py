from __future__ import print_function
from is_wire.core import Channel, Subscription
from is_msgs.camera_pb2 import FrameTransformation
import numpy as np
import argparse




# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-a", "--aruco", type=int, default=0,
	help="ID of the ArUco marker to be used on map building")
ap.add_argument("-m", "--mapfile", default="map.dat",
	help="Name of the file that will contain the map")
args = vars(ap.parse_args())

arucoID = args["aruco"] + 100
mapFile = args["mapfile"]



# Connect to the broker
channel = Channel("amqp://10.10.2.23:30000")


# Subscribe to the desired topic(s)
subscription = Subscription(channel)
topic01 = "FrameTransformation."+str(arucoID)+".1000"
subscription.subscribe(topic01)




# Blocks forever waiting for message


matrix = np.matrix('0,0,0,0;0,0,0,0;0,0,0,0;0,0,0,0')

ii=0


with open(mapFile, 'w') as f:
  # Compare the Frame Transforamtion obtained directly from the Aruco detecion service  when an Aruco Marker is seen
  # with the calculated Frame Transforamtion when considering the camera extrinsic matrix and the Aruco Marker position in the world
  try:
    while True: 
      message = channel.consume()
     

      
      # Get the Frame Transformation that represents the pose of the Aruco mark in the camera 7 reference frame 
      if message.topic == topic01 :
        # Message returns a list of Frame Transformations
        frameTrans = message.unpack(FrameTransformation)
        
        # Check if the Frame transformation has a tensor containing the transformation matrix
        if frameTrans.HasField("tf") :
          tensor = frameTrans.tf
          matrix=np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
          
          posX = matrix[0,3]
          posY = matrix[1,3]
          posZ = matrix[2,3]
          strPosition = 'X: ' + str(posX) + '   Y: ' + str(posY) + '   Z: ' + str(posZ)
         
          ii+= 1
          print(ii)
          f.write('{:.4f} {:.4f}\n'.format(posX, posY))
  except KeyboardInterrupt:
    pass
        

