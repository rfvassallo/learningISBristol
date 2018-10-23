from __future__ import print_function
from is_wire.core import Channel, Subscription
from is_msgs.common_pb2 import Tensor
from is_msgs.camera_pb2 import FrameTransformation
from transformation import TransformationFetcher
import numpy as np
import matplotlib.pyplot as plt
import thread



#def get_keyPressed(event):
#    keyPressed["letter"] = event.key

def input_thread(key):
    raw_input()
    key.append(True)


# Connect to the broker
#channel = Channel("amqp://guest:guest@10.10.2.20:30000")
channel = Channel("amqp://10.10.2.20:30000")


# Subscribe to the desired topic(s)
subscription = Subscription(channel)
topic01 = "FrameTransformation.100.1000"
subscription.subscribe(topic01)
topic02 = "FrameTransformation.100.1001"
subscription.subscribe(topic02)
topic03 = "FrameTransformation.100.1003"
subscription.subscribe(topic03)


topicList = [topic01,topic02,topic03]

# Blocks forever waiting for message

tfFetcher = TransformationFetcher("amqp://10.10.2.20:30000")
diretArucoPose = np.matrix('0,0,0,0;0,0,0,0;0,0,0,0;0,0,0,0')
matrix = np.matrix('0,0,0,0;0,0,0,0;0,0,0,0;0,0,0,0')
x = []
y = []
worldFrame = 1000

#key = []
#thread.start_new_thread(input_thread, (key,))

ii=0


with open('map.dat', 'w') as f:
  # Compare the Frame Transforamtion obtained directly from the Aruco detecion service  when an Aruco Marker is seen
  # with the calculated Frame Transforamtion when considering the camera extrinsic matrix and the Aruco Marker position in the world
  try:
    while True: 
      message = channel.consume()
      #if key: break

      
      # Get the Frame Transformation that represents the pose of the Aruco mark in the camera 7 reference frame 
      # Then multiply by the extrinsic matrix of camera 7 to obtain the Aruco pose in world reference frame
      if message.topic in topicList :
        # Message returns a list of Frame Transformations
        frameTrans = message.unpack(FrameTransformation)
        
        # Check if the Frame transformation has a tensor containing the transformation matrix
        if frameTrans.HasField("tf") :
          tensor = frameTrans.tf
          matrix=np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)
          print("**********")

          if frameTrans.to != worldFrame:
            tf = tfFetcher.get_transformation(frameTrans.to, worldFrame)
            #tfMatrix = np.matrix(tf.doubles).reshape(tf.shape.dims[0].size,tf.shape.dims[1].size)
            matrix = tf*matrix    
            print("#################")  
          '''
          print ("---------- Extrinsec Matrix  --------")
          print (matrix)
          print ("-------------------------------------")
          '''
          posX = matrix[0,3]
          posY = matrix[1,3]
          posZ = matrix[2,3]
          strPosition = 'X: ' + str(posX) + '   Y: ' + str(posY) + '   Z: ' + str(posZ)
          '''
          print ("-------- Position ------------------")
          print (strPosition)
          print ("------------------------------------\n")
          '''
          #x.append (posX)
          #y.append (posY)
          ii+= 1
          print(ii)
          f.write('{:.4f} {:.4f}\n'.format(posX, posY))
  except KeyboardInterrupt:
    pass
        
'''
fig=plt.figure()
plt.axis([-5,20,-5,20])
plt.plot(x,y,'b.')
plt.show()
'''
#with open('map.dat', 'w') as f:
 #   for i in range(len(x)):
  #      f.write('{:.4f} {:.4f}\n'.format(x[i], y[i]))


