import numpy as np
import numpy.matlib





def read_frameTransformation(fileName):
# Function to read the file that contains the map
    with open(fileName, 'r') as f:
        # read lines
        line = f.readline()
        header = []
        frameMatrix = np.matlib.zeros((4,4))
        # Go through the lines to get the (x,y) coordinates of the map
        i=0
        while line:
            if line.startswith('#'):
                header.append(line)
            else:
                # split the line when find a space
                data = line.split()
                frameMatrix[i,0] = float(data[0])
                frameMatrix[i,1] = float(data[1])
                frameMatrix[i,2] = float(data[2])
                frameMatrix[i,3] = float(data[3])
                i=i+1
            line = f.readline()
        
        
    return frameMatrix




fileName = "frameArUcoRobot.dat"

frameArUcoRobot = read_frameTransformation(fileName)

print(frameArUcoRobot)