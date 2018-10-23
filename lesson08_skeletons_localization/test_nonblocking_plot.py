
import random
import matplotlib.pyplot as plt




fig=plt.figure()
plt.axis([-5,20,-5,20])
plt.ion()


for i in range(200):
    x = random.random()*4
    y = random.random()*7
    plt.plot(x, y,'r.')
    plt.draw()
    plt.pause(0.001)
    #print x
    #print y
    #print ('Ok. Funciona')
