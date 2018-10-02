
import numpy as np
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D





def create_world_frame():
    # Create the world frame
    u_vectors = np.array([1,0,0])
    v_vectors = np.array([0,1,0])
    w_vectors = np.array([0,0,1])
    posx = np.array([0,0,0])
    posy = np.array([0,0,0])
    posz = np.array([0,0,0])

    return posx, posy, posz, u_vectors, v_vectors, w_vectors

def get_robot_pose(a):

    # Separate the position and orientation of the camera  frame and axis
    u_robot_vec=np.array(a[0,0:3])
    v_robot_vec=np.array(a[1,0:3])
    w_robot_vec=np.array(a[2,0:3])
    robot_posx=np.array([a[0,3], a[0,3], a[0,3]])
    robot_posy=np.array([a[1,3], a[1,3], a[1,3]])
    robot_posz=np.array([a[2,3], a[2,3], a[2,3]])

    return robot_posx, robot_posy, robot_posz, u_robot_vec, v_robot_vec, w_robot_vec

def update_axis (ax,posx,posy):
    ax.cla()
    ax.text(1,0,0,"X")
    ax.text(0,1,0,"Y")
    ax.text(0,0,1,"Z")
    ax.text(0,0,0,"World")
    ax.set_xlim(-10,posx+10)
    ax.set_ylim(-10,posy+10)
    ax.set_zlim(-0.5,2.5) 
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')



#plt.ion()
# Create a 3D figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
update_axis (ax,0,0)
# world reference frame
posx,posy,posz,u_vectors,v_vectors,w_vectors = create_world_frame()
# plot frame
ax.quiver(posx,posy,posz,u_vectors,v_vectors,w_vectors)
plt.draw()
plt.pause(0.5)

test = np.identity(4)


i=0
while i<15 :

    test[0,3] = test[0,3] + 1
    test[1,3] = test[1,3] + 2
    # get new pose of the robot
    robot_posx, robot_posy, robot_posz, u_robot_vec, v_robot_vec, w_robot_vec = get_robot_pose(test)
    update_axis (ax,robot_posx[0],robot_posy[0])
    # plot world frame and robot frame
    ax.quiver(posx,posy,posz,u_vectors,v_vectors,w_vectors)
    ax.quiver(robot_posx,robot_posy,robot_posz,u_robot_vec,v_robot_vec,w_robot_vec)
    # add label to robot frame
    ax.text(test[0,3],test[1,3],test[2,3],"Robot")
    
    plt.draw()
    plt.pause(0.5)
    
    i = i + 1
    #print i


plt.show()