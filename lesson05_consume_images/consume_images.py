from __future__ import print_function
from is_wire.core import Channel, Subscription
from is_msgs.image_pb2 import Image
import numpy as np
import cv2
import sys
import re

# This program allows to see the images from one of the 8 cameras of
# the Intelligent Space
# User can chage the camera by pressing a number between 0 and 7


# Connect to the broker
channel = Channel("amqp://10.10.2.23:30000")

# Subscribe to the desired topic(s)
subscription = Subscription(channel)
# Create the topic name to the camera gateway that can be changed later 
Topic= ["CameraGateway.0.Frame"]
subscription.subscribe(Topic[0])
# ... subscription.subscribe(topic="Other.Topic")


# Function to change the camera gateway
def changeCamera(camID):    
    # First we must unsubscribe from the latest topic (camera gateway)
    subscription.unsubscribe(Topic[0])
    # Change for the new topic
    Topic[0] = "CameraGateway."+camID+".Frame"
    # Subscribe in the new topic
    subscription.subscribe(Topic[0])



# Keep executing until 'q' is pressed

while True:

   # Blocks forever waiting for one message from any subscription
   message = channel.consume()

   # Check if the message received is an image from one of the cameras' gateway
   if re.match(r'CameraGateway.\d.Frame',message.topic):
       # Unpack the message
       image = message.unpack(Image)
       # Prepare the image matrix to be visualized
       nparr = np.fromstring(image.data, np.uint8)
       img_np = cv2.imdecode(nparr, -1)
       # Show image
       cv2.imshow('image',img_np)

   # Get key pressed
   key = cv2.waitKey(1)    
   keyChar = key & 255   

   # Check if 'q' was pressed
   if 'q' == chr(keyChar):
      sys.exit(0)
   # or if a number between 0 and 7 was pressed to change the camera
   elif keyChar >= ord('0') and keyChar <= ord('7'): 
      changeCamera(chr(keyChar))





