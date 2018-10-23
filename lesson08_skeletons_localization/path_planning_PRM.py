"""

Probablistic Road Map (PRM) Planner

author: Atsushi Sakai (@Atsushi_twi)

"""

import random
import math
import numpy as np
import scipy.spatial
import matplotlib.pyplot as plt

# parameter
N_SAMPLE = 600  # number of sample_points
N_KNN = 15  # number of edge from one sampled point
MAX_EDGE_LEN = 7 # [m] Maximum edge length

show_animation = True


class Node:
    """
    Node class for dijkstra search
    """

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


class KDTree:
    """
    Nearest neighbor search class with KDTree
    """

    def __init__(self, data):
        # store kd-tree
        self.tree = scipy.spatial.cKDTree(data)

    def search(self, inp, k=1):
        u"""
        Search NN

        inp: input data, single frame or multi frame

        """

        if len(inp.shape) >= 2:  # multi input
            index = []
            dist = []

            for i in inp.T:
                idist, iindex = self.tree.query(i, k=k)
                index.append(iindex)
                dist.append(idist)

            return index, dist
        else:
            dist, index = self.tree.query(inp, k=k)
            return index, dist

    def search_in_distance(self, inp, r):
        u"""
        find points with in a distance r
        """

        index = self.tree.query_ball_point(inp, r)
        return index


def PRM_planning(sx, sy, gx, gy, ox, oy, rr):

    obkdtree = KDTree(np.vstack((ox, oy)).T)

    #sample_x, sample_y = sample_points(sx, sy, gx, gy, rr, ox, oy, obkdtree)
    
    sample_x, sample_y = read_map('map.dat')
    sample_x.append(sx)
    sample_y.append(sy)
    sample_x.append(gx)
    sample_y.append(gy)
    

    #if show_animation:
    #    plt.plot(sample_x, sample_y, ".b")

    road_map = generate_roadmap(sample_x, sample_y, rr, obkdtree)

    rx, ry = dijkstra_planning(sx, sy, gx, gy, ox, oy, rr, road_map, sample_x, sample_y)

    return rx, ry


def is_collision(sx, sy, gx, gy, rr, okdtree):
    x = sx
    y = sy
    dx = gx - sx
    dy = gy - sy
    yaw = math.atan2(gy - sy, gx - sx)
    d = math.sqrt(dx**2 + dy**2)

    if d >= MAX_EDGE_LEN:
        #print('entrei')
        return True

    D = rr
    nstep = round(d / D)

    for i in range(nstep):
        idxs, dist = okdtree.search(np.matrix([x, y]).T)
        if dist[0] <= rr:
            return True  # collision
        x += D * math.cos(yaw)
        y += D * math.sin(yaw)

    # goal point check
    idxs, dist = okdtree.search(np.matrix([gx, gy]).T)
    if dist[0] <= rr:
        return True  # collision

    return False  # OK


def generate_roadmap(sample_x, sample_y, rr, obkdtree):
    """
    Road map generation

    sample_x: [m] x positions of sampled points
    sample_y: [m] y positions of sampled points
    rr: Robot Radius[m]
    obkdtree: KDTree object of obstacles
    """

    road_map = []
    nsample = len(sample_x)
    skdtree = KDTree(np.vstack((sample_x, sample_y)).T)

    for (i, ix, iy) in zip(range(nsample), sample_x, sample_y):

        index, dists = skdtree.search(
            np.matrix([ix, iy]).T, k=nsample)
        inds = index[0][0]
        edge_id = []
        #  print(index)

        for ii in range(1, len(inds)):
            nx = sample_x[inds[ii]]
            ny = sample_y[inds[ii]]

            if not is_collision(ix, iy, nx, ny, rr, obkdtree):
                edge_id.append(inds[ii])

            if len(edge_id) >= N_KNN:
                break

        road_map.append(edge_id)

    #plot_road_map(road_map, sample_x, sample_y)

    return road_map


def dijkstra_planning(sx, sy, gx, gy, ox, oy, rr, road_map, sample_x, sample_y):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    nstart = Node(sx, sy, 0.0, -1)
    ngoal = Node(gx, gy, 0.0, -1)

    openset, closedset = dict(), dict()
    openset[len(road_map) - 2] = nstart

    while True:
        if len(openset) == 0:
            print("Cannot find path")
            break

        c_id = min(openset, key=lambda o: openset[o].cost)
        current = openset[c_id]

        # show graph
        '''
        if show_animation and len(closedset.keys()) % 2 == 0:
            plt.plot(current.x, current.y, "xg")
            plt.pause(0.001)
        '''

        if c_id == (len(road_map) - 1):
            print("goal is found!")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i in range(len(road_map[c_id])):
            n_id = road_map[c_id][i]
            dx = sample_x[n_id] - current.x
            dy = sample_y[n_id] - current.y
            d = math.sqrt(dx**2 + dy**2)
            node = Node(sample_x[n_id], sample_y[n_id],current.cost + d, c_id)

            if n_id in closedset:
                continue
            # Otherwise if it is already in the open set
            if n_id in openset:
                if openset[n_id].cost > node.cost:
                    openset[n_id].cost = node.cost
                    openset[n_id].pind = c_id
            else:
                openset[n_id] = node

    # generate final course
    rx, ry = [ngoal.x], [ngoal.y]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x)
        ry.append(n.y)
        pind = n.pind

    return rx, ry


def plot_road_map(road_map, sample_x, sample_y):

    for i in range(len(road_map)):
        for ii in range(len(road_map[i])):
            ind = road_map[i][ii]

            plt.plot([sample_x[i], sample_x[ind]],
                     [sample_y[i], sample_y[ind]], "-k")

'''
def sample_points(sx, sy, gx, gy, rr, ox, oy, obkdtree):
    maxx = max(ox)
    maxy = max(oy)
    minx = min(ox)
    miny = min(oy)

    print ('maxx = ',maxx)
    print ('maxy = ',maxy)
    print ('minx = ',minx)
    print ('miny = ',miny)

    sample_x, sample_y = [], []

    while len(sample_x) <= N_SAMPLE:
        tx = (random.random()) * (maxx - minx) + minx
        ty = (random.random()) * (maxy - miny) + miny
        #tx = (random.random() - minx) * (maxx - minx)
        #ty = (random.random() - miny) * (maxy - miny)


        index, dist = obkdtree.search(np.matrix([tx, ty]).T)

        if dist[0] >= rr:
            sample_x.append(tx)
            sample_y.append(ty)

    sample_x.append(sx)
    sample_y.append(sy)
    sample_x.append(gx)
    sample_y.append(gy)

    return sample_x, sample_y
'''

def create_virtualObstacles() :
    ox = []
    oy = []
    for i in frange(-3,7,0.1):
        ox.append(i)
        oy.append(-4)
    for i in frange(-4,22,0.1):
        ox.append(7)
        oy.append(i)
    for i in frange(-1,7,0.1):
        ox.append(i)
        oy.append(22)
    for i in frange(13,22,0.1):
        ox.append(-1)
        oy.append(i)
    for i in frange(5,13,0.1):
        ox.append(2)
        oy.append(i)
    for i in frange(-3,2,0.1):
        ox.append(i)
        oy.append(5)
    for i in frange(-4,5,0.1):
        ox.append(-3)
        oy.append(i)
    for i in frange(-1,2,0.1):
        ox.append(i)
        oy.append(13)

    if (False):
        ox=[]
        oy=[]
        for i in frange(-20,40,0.1):
            ox.append(i)
            oy.append(-20)
        for i in frange(-20,40,0.1):
            ox.append(40)
            oy.append(i)
        for i in frange(-20,40,0.1):
            ox.append(i)
            oy.append(40)
        for i in frange(-20,40,0.1):
            ox.append(-20)
            oy.append(i)
                        

    return ox,oy

def frange(start, stop, step):
    i = start
    while i < stop:
        yield i
        i += step

def read_map(fileName):
    with open(fileName, 'r') as f:
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
        x = (np.array(x)).tolist()
        y = (np.array(y)).tolist()
        lowRes_x = []
        lowRes_y = []
        for i in range(0,len(x),5):
            lowRes_x.append(x[i])
            lowRes_y.append(y[i])
    return lowRes_x,lowRes_y




def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 0.0  # [m]
    sy = 0.0 # [m]
    gx = 4.50  # [m]
    gy = 16.0  # [m]
    robot_size = 0.6  # [m]

    ox, oy = create_virtualObstacles()
    sample_x, sample_y = read_map('map.dat')

    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "^r")
        plt.plot(gx, gy, "^c")
        plt.plot(sample_x, sample_y, ".b")
        plt.grid(True)
        plt.axis("equal")

    rx, ry = PRM_planning(sx, sy, gx, gy, ox, oy, robot_size)

    assert len(rx) != 0, 'Cannot found path'

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.show()


if __name__ == '__main__':
    main()