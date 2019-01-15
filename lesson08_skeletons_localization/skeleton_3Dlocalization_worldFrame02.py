
from __future__ import print_function
from is_msgs.image_pb2 import ObjectAnnotations
from is_wire.core import Channel, Message, Subscription
import matplotlib.pyplot as plt
import numpy as np
import thread



 


####  Main Program #######################
# Colleting all the 3D positions where skeletons were detected


# Create a channel to connect to the broker
channel = Channel("amqp://10.10.2.23:30000")
# Create a subscription 
subscription = Subscription(channel)

# Subscribe in the topic that can directly provide the 3D localizations of the detected skeletons
topic="SkeletonsGrouper.3DLocalization"
subscription.subscribe(topic)

x = []
y = []


mapFile = "mapTest02.dat"


with open(mapFile, 'w') as f:
  
    try:
        while True: 
            message = channel.consume()
            
            # Check if the message is of the type expected    
            if (message.topic == topic):
                
                # Unpack the message to get the skeletons
                annotations = message.unpack(ObjectAnnotations)
                
                # Check if there is one or more skeletons (objects)
                if len(annotations.objects) > 0:
                    
                    # Get the coordinates of the joints of the skeletons 
                    for sk in annotations.objects :

                        for keypoint in sk.keypoints :
                            
                            if keypoint.id in range(0,21):
                                # Add the (x,y) coordinates to the lists and map file
                                x.append (keypoint.position.x)
                                y.append (keypoint.position.y)
                                f.write('{:.4f} {:.4f}\n'.format(keypoint.position.x, keypoint.position.y))
                                print(keypoint.position.x, keypoint.position.y)
                                


        
    except KeyboardInterrupt:
        # Plot the collect positions
        fig=plt.figure()
        plt.axis([-5,20,-5,20])
        plt.plot(x,y,'b.')
        plt.show()        
        pass

