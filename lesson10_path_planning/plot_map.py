

import matplotlib.pyplot as plt
import numpy as np
import argparse


# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-m", "--mapfile", default="../lesson09_mapping_with_ArUco/map.dat",
	help="Name of the file that will contain the map")
ap.add_argument("-g", "--grain", type=int, default=2,
	help="Granularity for plotting the map")
args = vars(ap.parse_args())

mapFile = args["mapfile"]
step = args["grain"]

with open(mapFile, 'r') as f:
    
    # Read first line
    line = f.readline()
    header = []
    x = []
    y = []
    # Loop for reading lines and collecting map coordinates
    while line:
        if line.startswith('#'):
            header.append(line)
        else:
            # Split the line when a space is found
            data = line.split()
            # Get the map coordinates
            x.append(float(data[0]))
            y.append(float(data[1]))
        line = f.readline()

    # Reduce map density to plot
    lowRes_x = []
    lowRes_y = []
    for i in range(0,len(x),step):
        lowRes_x.append(x[i])
        lowRes_y.append(y[i])
        
        




fig=plt.figure()
#plt.axis([-5,20,-5,20])
plt.plot(lowRes_x,lowRes_y,'b.')
plt.grid(True)
plt.axis('equal')
plt.show()
