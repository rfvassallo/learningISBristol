"""

Probablistic Road Map (PRM) Planner

original author: Atsushi Sakai (@Atsushi_twi)

modified by: Raquel Frizera Vassallo
"""

import random
import math
import numpy as np
import scipy.spatial
import matplotlib.pyplot as plt
import argparse




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
        """
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
        """
        find points with in a distance r
        """

        index = self.tree.query_ball_point(inp, r)
        return index


def PRM_planning(mapFile, sx, sy, gx, gy, ox, oy, rr, step, N_KNN, MAX_EDGE_LEN):
    """
    sx: source x position [m] 
    sy: source y position [m] 
    gx: goal x position [m]
    gx: goal x position [m]
    ox: obstacles x position [m]
    oy: obstacles y position [m]
    rr: Robot Radius[m]
    """

    # Create KDTree containing obstacles
    obkdtree = KDTree(np.vstack((ox, oy)).T)

    # Read map file    
    map_x, map_y = read_map(mapFile, step)
    # Include source and goal positions to the map, so they can be used to 
    # find the path
    map_x.append(sx)
    map_y.append(sy)
    map_x.append(gx)
    map_y.append(gy)
    
    # Create the roadmap, with the links between 
    road_map = generate_roadmap(map_x, map_y, rr, obkdtree,N_KNN, MAX_EDGE_LEN)
    # Find the path using Dijkstra
    rx, ry = dijkstra_planning(sx, sy, gx, gy, ox, oy, rr, road_map, map_x, map_y)

    return rx, ry


def is_collision(sx, sy, gx, gy, rr, okdtree, MAX_EDGE_LEN):
    # Check if there is collision with the obstacles
    """
    sx: source x position [m] 
    sy: source y position [m] 
    gx: goal x position [m]
    gx: goal x position [m]
    rr: Robot Radius[m]
    obkdtree: KDTree object of obstacles
    """
    x = sx
    y = sy
    dx = gx - sx
    dy = gy - sy
    yaw = math.atan2(gy - sy, gx - sx)
    d = math.sqrt(dx**2 + dy**2)

    if d >= MAX_EDGE_LEN:
        return True
    
    # Use robot radius to define step
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


def generate_roadmap(map_x, map_y, rr, obkdtree, N_KNN, MAX_EDGE_LEN):
    """
    Road map generation

    sample_x: [m] x positions of sampled points
    sample_y: [m] y positions of sampled points
    rr: Robot Radius[m]
    obkdtree: KDTree object of obstacles
    """

    road_map = []
    nsample = len(map_x)
    skdtree = KDTree(np.vstack((map_x, map_y)).T)

    for (i, ix, iy) in zip(range(nsample), map_x, map_y):

        index, dists = skdtree.search(
            np.matrix([ix, iy]).T, k=nsample)
        inds = index[0][0]
        edge_id = []
        #  print(index)

        for ii in range(1, len(inds)):
            nx = map_x[inds[ii]]
            ny = map_y[inds[ii]]

            if not is_collision(ix, iy, nx, ny, rr, obkdtree, MAX_EDGE_LEN):
                edge_id.append(inds[ii])

            if len(edge_id) >= N_KNN:
                break

        road_map.append(edge_id)

    #plot_road_map(road_map, map_x, map_y)

    return road_map


def dijkstra_planning(sx, sy, gx, gy, ox, oy, rr, road_map, map_x, map_y):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
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
            dx = map_x[n_id] - current.x
            dy = map_y[n_id] - current.y
            d = math.sqrt(dx**2 + dy**2)
            node = Node(map_x[n_id], map_y[n_id],current.cost + d, c_id)

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


def plot_road_map(road_map, map_x, map_y):

    for i in range(len(road_map)):
        for ii in range(len(road_map[i])):
            ind = road_map[i][ii]

            plt.plot([map_x[i], map_x[ind]],
                     [map_y[i], map_y[ind]], "-k")



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

    """ # Create a square as obstacle
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
    """                

    return ox,oy


def frange(start, stop, step):
# Function range for float 
    i = start
    while i < stop:
        yield i
        i += step

def read_map(fileName, step):
# Function to read the file that contains the map
    with open(fileName, 'r') as f:
        # read lines
        line = f.readline()
        header = []
        x = []
        y = []
        # Go through the lines to get the (x,y) coordinates of the map
        while line:
            if line.startswith('#'):
                header.append(line)
            else:
                # split the line when find a space
                data = line.split()
                x.append(float(data[0]))
                y.append(float(data[1]))
            line = f.readline()
        
        x = (np.array(x)).tolist()
        y = (np.array(y)).tolist()
        
        # Reduce the map density to speed up the path-planning 
        lowRes_x = []
        lowRes_y = []
        
        for i in range(0,len(x),step):
            lowRes_x.append(x[i])
            lowRes_y.append(y[i])
        
    return lowRes_x,lowRes_y




def main(args):
    print(__file__ + " start!!")
    # parameters
    mapFile = args["mapfile"]
    step = args["grain"]
    source = args["origin"]
    sx = source[0]
    sy = source[1]
    goal = args["target"]
    gx = goal[0]
    gy = goal[1]
    robot_size = args["robotradius"]
    N_KNN = args["nknn"]  # number of edge from one sampled point
    MAX_EDGE_LEN = args["maxedge"] # [m] Maximum edge length
    show_animation = args["show"]

    
    # Create obstacles
    ox, oy = create_virtualObstacles()

    # Read map just to plot 
    map_x, map_y = read_map(mapFile,step)

    # Plot map points and obstacles
    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "^r")
        plt.plot(gx, gy, "^c")
        plt.plot(map_x, map_y, ".b")
        plt.grid(True)
        plt.axis("equal")
    
    # Perform path-planning
    rx, ry = PRM_planning(mapFile,sx, sy, gx, gy, ox, oy, robot_size, step, N_KNN, MAX_EDGE_LEN)

    # Check if a path was found
    assert len(rx) != 0, 'Cannot found path'

    # SHow the result
    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.show()


if __name__ == '__main__':
    # construct the argument parse and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-m", "--mapfile", default="../lesson09_mapping_with_ArUco/map1311.dat",
        help="Name of the file that will contain the map")
    ap.add_argument("-g", "--grain", type=int, default=2,
        help="Granularity for creating the roadmap from the points saved in the map file")
    ap.add_argument("-o", "--origin", nargs='+', type=float,
        help="x and y coordinates of the origin for the path planning")
    ap.add_argument("-t", "--target", nargs='+', type=float,
        help="x and y coordinates of the target for the path planning")
    ap.add_argument("-r", "--robotradius", type=float, default=0.6,
        help="Robot radius size")
    ap.add_argument("-n", "--nknn", type=int, default= 20,
        help="Number of edges (KNN) from one sampled point used for building the roadmap")
    ap.add_argument("-e", "--maxedge", type=int, default= 3,
        help="Maximum edge length [m] used for avoiding collisions")
    ap.add_argument("-s", "--show", type=bool, default= True,
        help="Show animation")

    args = vars(ap.parse_args())

    main(args)
