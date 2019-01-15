
from is_wire.core import Channel, Message, Subscription
from is_msgs.camera_pb2 import FrameTransformation
#import numpy as np
#import numpy.matlib
import sys
import argparse
sys.path.append('../lesson12_navigation_with_path_plannig/')
from navigate_path import unpackFrameTransformation, navigate


#######################################################
# Main program

def call_via_aruco(arucoID):

    # parameters for navigation 
    mapFile = "../lesson09_mapping_with_ArUco/map3011.dat"
    robotArUco = 8
    worldFrame = 1000
    step = 2
    robotRadius = .8
    N_KNN = 40  # number of edge from one sampled point
    MAX_EDGE_LEN = 2.5 # [m] Maximum edge length
    show_path = False



    # Create a channel to connect to the broker
    channel = Channel("amqp://10.10.2.23:30000")
    # Create a subscription 
    subscription = Subscription(channel)

    # Subscribe to the following topic:
    # - To get the position of the ArUco marker used as a calling signal for the robot

    topicGetArUcoLocation = "FrameTransformation."+str(arucoID+100)+"."+str(worldFrame)
    
    subscription.subscribe(topicGetArUcoLocation)

    # Localise the marker for calling the robot
    
    markerLocalized = False
    notSeen = 0
    count = 0

    while not markerLocalized:
        # Source must be the current position of the robot in the world frame
        message = channel.consume()
        
        notSeen = notSeen + 1

        if (message.topic == topicGetArUcoLocation):
            print("Found ArUco")
            # get the frame transformation betweeb the ArUco marker on the robot's back and the world
            arUcoToWorld = unpackFrameTransformation (message)
            
            goalX = arUcoToWorld[0,3]
            goalY = arUcoToWorld[1,3]
            print("x= ",goalX," y= ",goalY)
            markerLocalized = True
            notSeen = 0

        
        
        if notSeen > 30:
            notSeen = 0
            count = count + 1
            print("Try to turn the marker or show to other camera.")


            if count > 4:
                print("I can't localize the marker in the Intelligent Space.")
                sys.exit(0)
            

            
    # unsubscribe to not accumulate messages
    subscription.unsubscribe(topicGetArUcoLocation)
    
    

    # call the robot
    navigate(goalX,goalY,robotArUco, worldFrame, mapFile,step,robotRadius,N_KNN,MAX_EDGE_LEN,show_path)


def main(args):
    print(__file__ + " start!!")

    arucoID = args["aruco"]
    call_via_aruco(arucoID)


if __name__ == '__main__':
    # construct the argument parse and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-a","--aruco",type=int, default=4,
        help="ID of the ArUco used to call the robot")
    
    args = vars(ap.parse_args())

    main(args)
