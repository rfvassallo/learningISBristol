
from __future__ import print_function
from is_msgs.image_pb2 import ObjectAnnotations
from is_wire.core import Channel, Message, Subscription
import matplotlib.pyplot as plt
from transformation import TransformationFetcher, transform_object_annotations
import numpy as np
import thread


# Unpack message and check in which reference frame are the skeletons.
# If the reference frame is different from the world reference 1000, transform the skeletons so they
# are represented in the world frame 1000

def get_skeletons_3Dlocalization(message):
    
    annotations = message.unpack(ObjectAnnotations)    

    # Check if there is one or more skeletons (objects)
    if len(annotations.objects) > 0:

        skeletons = annotations
        # Check if the skeletons reference frame is diferent from the world reference frame 1000
        if annotations.frame_id != worldFrame :
            # If different, convert the skeletons to the world frame
            tf = tfFetcher.get_transformation(annotations.frame_id, worldFrame)
            skeletons = transform_object_annotations(annotations, tf, worldFrame)
            
        return skeletons

    else:
        return annotations



####  Main Program #######################
# Colleting all the 3D positions where skeletons were detected

# Create a channel to connect to the broker
channel = Channel("amqp://10.10.2.23:30000")
# Create a subscription 
subscription = Subscription(channel)

# Subscribe in all the topics that can return the skeletons localization in the different areas
# In this case the areas are 0, 1 and 2
topics=[]
for i in range(0,3):
    topics.append("SkeletonsGrouper."+str(i)+".Localization")
    subscription.subscribe(topics[i])


worldFrame = 1000
tfFetcher = TransformationFetcher("amqp://10.10.2.20:30000")
x = []
y = []


mapFile = "mapTest.dat"


with open(mapFile, 'w') as f:

    try:
        while True: 
            message = channel.consume()
            
            # Check if the message is of the type expected    
            if (message.topic in topics):
                # Get the skeletons in the world reference frame 1000
                skeletons = get_skeletons_3Dlocalization(message)

                # Get the coordinates of the joints of the skeletons
                for sk in skeletons.objects :

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


        
