

from is_wire.core import Channel, Message, Subscription
from is_msgs.common_pb2 import Tensor
import numpy as np
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D
import cv2
import sys



def create_world_frame():
    # Create the world frame
    u_vectors = np.array([1,0,0])
    v_vectors = np.array([0,1,0])
    w_vectors = np.array([0,0,1])
    posx = np.array([0,0,0])
    posy = np.array([0,0,0])
    posz = np.array([0,0,0])

    return posx, posy, posz, u_vectors, v_vectors, w_vectors#import math

def get_robot_pose(a):

    # Separate the position and orientation of the camera frame and axis
    u_robot_vec=np.array(a[0,0:3])
    v_robot_vec=np.array(a[1,0:3])
    w_robot_vec=np.array(a[2,0:3])
    robot_posx=np.array([a[0,3], a[0,3], a[0,3]])
    robot_posy=np.array([a[1,3], a[1,3], a[1,3]])
    robot_posz=np.array([a[2,3], a[2,3], a[2,3]])

    return robot_posx, robot_posy, robot_posz, u_robot_vec, v_robot_vec, w_robot_vec

def update_axis (ax,posx,posy):
    # clear axis
    ax.cla()
    # plot text for world axis
    ax.text(1,0,0,"X")
    ax.text(0,1,0,"Y")
    ax.text(0,0,1,"Z")
    ax.text(0,0,0,"World")
    # set plot limits according to the current robot's position
    ax.set_xlim(-10,posx+10)
    ax.set_ylim(-10,posy+10)
    ax.set_zlim(-0.5,2.5) 
    # plot axis labels
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')


def get_keyPressed(event):
    keyPressed["letter"] = event.key
 
##### Main Program
## Plot world frame and robot frame in the same graphic
#######################

# Create a channel to connect to the broker
channel = Channel("amqp://10.10.2.20:30000")
# Create a subscription 
subscription = Subscription(channel)
# Subscribe to get the relation from the initial reference frame (2001) where the robot was
# turned on and its current reference frame (2000). This Frame Transforamtion corressponds
# to the dead reckoning odometry
subscription.subscribe("NewRobotPose")


# Create a 3D figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

update_axis (ax,0,0)
# world reference frame
posx,posy,posz,u_vectors,v_vectors,w_vectors = create_world_frame()
# plot frame
ax.quiver(posx,posy,posz,u_vectors,v_vectors,w_vectors)
plt.draw()

keyPressed = {"letter":None} 

while True:
    # listen the channel
    message = channel.consume()

    if (message.topic == "NewRobotPose"):
        
        # unpack the message according to its format
        tensor = message.unpack(Tensor)
        # Extract the transformation matrix from the tensor
        robotPose = np.matrix(tensor.doubles).reshape(tensor.shape.dims[0].size,tensor.shape.dims[1].size)

           
        # get new pose of the robot
        robot_posx, robot_posy, robot_posz, u_robot_vec, v_robot_vec, w_robot_vec = get_robot_pose(robotPose)
        update_axis (ax,robot_posx[0],robot_posy[0])
        # plot world frame and robot framesubscription.unsubscribe("NewRobotPose")
        ax.quiver(posx,posy,posz,u_vectors,v_vectors,w_vectors)
        ax.quiver(robot_posx,robot_posy,robot_posz,u_robot_vec,v_robot_vec,w_robot_vec)
        # add label to robot frame
        ax.text(robot_posx[0],robot_posy[0],robot_posz[0],"Robot")        
        
        plt.draw()
        plt.pause(0.01)
    
    # Get key pressed
    cid = plt.gcf().canvas.mpl_connect('key_press_event', get_keyPressed)
    key = keyPressed["letter"]

    if  key == 'q':
        break

        
plt.close(fig)        