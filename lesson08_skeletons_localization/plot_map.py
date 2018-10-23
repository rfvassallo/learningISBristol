

import matplotlib.pyplot as plt
import numpy as np


with open('map.dat', 'r') as f:
    line = f.readline()
    header = []
    x = []
    y = []
    while line:
        if line.startswith('#'):
            header.append(line)
        else:
            data = line.split()
            x.append(float(data[0]))
            y.append(float(data[1]))
        line = f.readline()
    lowRes_x = []
    lowRes_y = []
    for i in range(0,len(x),20):
        lowRes_x.append(x[i])
        lowRes_y.append(y[i])
        
        




fig=plt.figure()
#plt.axis([-5,20,-5,20])
plt.plot(lowRes_x,lowRes_y,'b.')
plt.grid(True)
plt.axis('equal')
plt.show()
