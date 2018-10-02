

import numpy as np 
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D



def plot_pose(a) :

#a=np.matrix('0.858799636364 -0.498833626509 0.116739630699 0.0692247077823; 0.498248010874 0.866276144981 0.036255273968 0.047551561147; -0.119214117527 0.0270292758942 0.992500603199 1.14261198044; 0.0 0.0 0.0 1.0')
   
    
    #'0.807126879692 -0.520674049854 -0.278289377689 0.00147371739149; 0.482435017824 0.853386104107 -0.19745542109 0.00950167234987;0.340298175812 0.0251150391996 0.939982235432 1.17362141609; 0 0 0 1')




    # Create the world frame
    u_vectors = np.array([1,0,0])
    v_vectors = np.array([0,1,0])
    w_vectors = np.array([0,0,1])
    posx = np.array([0,0,0])
    posy = np.array([0,0,0])
    posz = np.array([0,0,0])


    # Separate the position and orientation of the camera  frame and axis
    u_robot_vec=np.array(a[0,0:3])
    v_robot_vec=np.array(a[1,0:3])
    w_robot_vec=np.array(a[2,0:3])
    robot_posx=np.array([a[0,3], a[0,3], a[0,3]])
    robot_posy=np.array([a[1,3], a[1,3], a[1,3]])
    robot_posz=np.array([a[2,3], a[2,3], a[2,3]])

    # Create a 3D figure
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')


    # plot the robot base frame and label the axis 
    ax.quiver(posx,posy,posz,u_vectors,v_vectors,w_vectors)
    ax.text(1,0,0,"X")
    ax.text(0,1,0,"Y")
    ax.text(0,0,1,"Z")
    ax.text(0,0,0,"World")


    # plot the camera frame and label the axis
    ax.quiver(robot_posx,robot_posy,robot_posz,u_robot_vec,v_robot_vec,w_robot_vec)
    ax.text((a[0,3]+a[0,0]),(a[1,3]+a[1,0]),(a[2,3]+a[2,0]),"X")
    ax.text((a[0,3]+a[0,1]),(a[1,3]+a[1,1]),(a[2,3]+a[2,1]),"Y")
    ax.text((a[0,3]+a[0,2]),(a[1,3]+a[1,2]),(a[2,3]+a[2,2]),"Z")
    ax.text(a[0,3],a[1,3],a[2,3],"Robot")

    #ax.axis('square')
    ax.set_xlim(-10,10)
    ax.set_ylim(-10,10)
    ax.set_zlim(-0.5,2.5) 
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')



    #plt.axis('equal')
    plt.show()