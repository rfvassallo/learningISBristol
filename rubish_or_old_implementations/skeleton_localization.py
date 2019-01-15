
from __future__ import print_function
from is_msgs.image_pb2 import ObjectAnnotations
from is_wire.core import Channel, Message, Subscription
import matplotlib.pyplot as plt
#from .utils import to_pb_image
from transformation import TransformationFetcher, transform_object_annotations
#import curses
#from is_msgs.common_pb2 import Tensor
#from np_pb import to_tensor, to_np
import numpy as np
#import math
import thread


def get_keyPressed(event):
    keyPressed["letter"] = event.key

def input_thread(key):
    raw_input()
    key.append(None)
 

####  Main Program #######################
# Using keypad to control the robot with some 
# feedback on the screen

# Create a channel to connect to the broker
channel = Channel("amqp://10.10.2.20:30000")
# Create a subscription 
subscription = Subscription(channel)

topic = "Skeletons.Localization"
subscription.subscribe(topic)

worldFrame = 1000
tfFetcher = TransformationFetcher("amqp://10.10.2.20:30000")
x = []
y = []

threshold = 0.5
jointList = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]

#stdscr = curses.initscr()
#curses.cbreak()
#stdscr.keypad(1)
#stdscr.addstr(0,10,"Hit 'q' to quit")
#stdscr.refresh()

#keyPressed = {"letter":None} 


key = []
thread.start_new_thread(input_thread, (key,))

#key=''
while True:     #key != ord('q'):
    # listen the channel
    message = channel.consume()
    #key = stdscr.getch()
    #stdscr.addch(20,25,key)
    #stdscr.refresh()
    if key: break

    # Check if the message received is the robot's odometry - FrameTransformation type
    if (message.topic == topic):
        annotations = message.unpack(ObjectAnnotations)
        #print annotations.frame_id
        if annotations.frame_id > 0:
            
            if len(annotations.objects) > 0:

                #print (annotations.frame_id)
                
                skeletons = annotations

                if annotations.frame_id != worldFrame :
                    tf = tfFetcher.get_transformation(annotations.frame_id, worldFrame)
                    skeletons = transform_object_annotations(annotations, tf, worldFrame)

                for sk in skeletons.objects :

                    for keypoint in sk.keypoints :
                        #i=2
                        if keypoint.id in jointList:
                            
                            if keypoint.position.z > threshold :

                                #strKeypoint = 'Keypoint ' + str(keypoint.id) + ' is from a standing person'
                                #stdscr.addstr(i, 20, strKeypoint)
                                #i = i + 1
                                print ('Keypoint ',keypoint.id,' is from a standing person')
                                if abs(keypoint.position.x) <= 25 or abs(keypoint.position.x) <= 25
                                    x.append (keypoint.position.x)
                                    y.append (keypoint.position.y)
                                    #plt.plot(keypoint.position.x,keypoint.position.y,'b.')
                                    #plt.draw()
                                    #plt.pause(0.001)
                                    # plot points
    # Get key pressed
    #cid = plt.gcf().canvas.mpl_connect('key_press_event', get_keyPressed)
    #key = keyPressed["letter"]
    

    
#curses.endwin()
fig=plt.figure()
plt.axis([-5,20,-5,20])
plt.plot(x,y,'b.')
plt.show()

with open('map.dat', 'w') as f:
    for i in range(len(x)):
        f.write('{:.4f} {:.4f}\n'.format(x[i], y[i]))

#plt.waitforbuttonpress()                                #

