import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from itertools import product
import math

class PathFinder:
    def __init__(self, grid):
        self.grid = grid
        self.xdim, self.ydim, self.zdim = grid.shape
        # allow all dimensions
        self.delta = [list(move) for move in product([-1, 0, 1], repeat=3) if move != (0, 0, 0)]
        self.cost = [round(math.sqrt(sum([i**2 for i in move])), 2) for move in self.delta]

        self.maxp = 0.1

    def calcheuristic(self, goal):
        x_index, y_index, z_index = np.indices(self.grid.shape)
        heuristic = np.sqrt((x_index - goal[0]) ** 2 + (y_index - goal[1]) ** 2 + (z_index - goal[2]) ** 2)
        return heuristic
    
    def update_grid(self,grid):
        self.grid = grid

    def search(self, init, goal):
        heuristic = self.calcheuristic(goal)
        
        x = init[0]
        y = init[1]
        z = init[2]
        
        closed = np.empty((self.xdim,self.ydim,self.zdim), dtype=np.int8)
        closed[:] = 0
        closed[x,y,z] = 1

        expand = np.empty((self.xdim,self.ydim,self.zdim), dtype=np.int8)
        expand[:] = -1
        action = np.empty((self.xdim,self.ydim,self.zdim), dtype=np.int8)
        action[:] = -1


        g = 0
        h = heuristic[x,y,z]
        f = g+h

        openl = [[f, g, x, y, z]]

        found = False  # flag that is set when search is complete
        resign = False # flag set if we can't find expand
        count = 0
    
        while not found and not resign and count < 1e6:
            if len(openl) == 0:
                resign = True
                return "Fail: Open List is empty"
            else:
                openl.sort()
                openl.reverse()
                nextl = openl.pop()
                
            
                x = nextl[2]
                y = nextl[3]
                z = nextl[4]
                g = nextl[1]
                f = nextl[0]
                expand[x,y,z] = count
                count += 1

                if x == goal[0] and y == goal[1] and z == goal[2]:
                    found = True
                else:
                    for i in range(len(self.delta)):
                        x2 = x + self.delta[i][0]
                        y2 = y + self.delta[i][1]
                        z2 = z + self.delta[i][2]
                        
                        if z2 >= 0 and z2 < self.zdim and \
                            y2 >=0 and y2 < self.ydim and \
                            x2 >=0 and x2 < self.xdim:
                                
                                if closed[x2,y2,z2] == 0 and self.grid[x2,y2,z2] < self.maxp:

                                    g2 = g + self.cost[i]
                                    f2 = g2 + heuristic[x2,y2,z2]
                                    openl.append([f2, g2, x2, y2, z2])
                                    closed[x2,y2,z2] = 1
                                    
                                    # Memorize the sucessfull action for path planning
                                    action[x2,y2,z2] = i
                        else:
                            pass

        path=[]

        path.append([goal[0], goal[1], goal[2]])
        
        while x != init[0] or y != init[1] or z != init[2]:
            x2 = x-self.delta[action[x,y,z]][0]
            y2 = y-self.delta[action[x,y,z]][1]
            z2 = z-self.delta[action[x,y,z]][2]
            #policy[x2][y2][z2]=delta_name[action[x][y][z]]
            x = x2
            y = y2
            z = z2
            # Path
            path.append([x2, y2, z2])
        
        path.reverse()

        return path

    # def plot3Dgrid(self, az, el):
    #     # ... plot the grid using self.grid ...
    #     return plt3d

# Usage:
# grid should be the 3D numpy array representing the occupancy grid
# grid = np.zeros((50,50,50))
# pathfinder = PathFinder(grid)
# start = [20, 20, 20]
# goal = [40, 40, 40]
# path = pathfinder.search(start, goal)

# print(path)

# path = np.array(path)

# locations = np.array(locations)

# grid[locations[:, 0], locations[:, 1], locations[:, 2]] = 1

# grid = np.array(grid)

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.plot(path[:, 0], path[:, 1], path[:, 2])
# plt.show()

