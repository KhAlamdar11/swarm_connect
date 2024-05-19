import numpy as np
import math
from copy import deepcopy

class OccupancyGrid3D:
    def __init__(self, ranges, origins, resolution):
        self.range_x = math.ceil(ranges[0]/resolution)
        self.range_y = math.ceil(ranges[1]/resolution)
        self.range_z = math.ceil(ranges[2]/resolution)
        self.resolution = resolution
        self.origin_x = origins[0]
        self.origin_y = origins[1] 
        self.origin_z = origins[2] 
        self.grid = np.zeros((self.range_x, self.range_y, self.range_z), dtype=int)  # 0 represents free space

    def mark_occupied(self, pos):
        real_x, real_y, real_z = pos
        x, y, z = self.real_to_grid(real_x, real_y, real_z)
        if 0 <= x < self.range_x and 0 <= y < self.range_y and 0 <= z < self.range_z:
            self.grid[y, x, z] = 1  # 1 represents an occupied cell
        else:
            print("UAV OUT OF RANGE!")

    def get_grid_marked(self, pos, n=3):
        self.grid.fill(0)
        # print('-----------------------------')
        for p in pos:
            # print(p)
            x, y, z = self.real_to_grid(p)
            # Iterate over the neighborhood defined by n
            for dx in range(-n, n + 1):
                for dy in range(-n, n + 1):
                    for dz in range(-n, n + 1):
                        nx, ny, nz = x + dx, y + dy, z + dz
                        # Check if the neighbor is within the grid bounds
                        if 0 <= nx < self.range_x and 0 <= ny < self.range_y and 0 <= nz < self.range_z:
                            self.grid[ny, nx, nz] = 1
        return self.grid
        


    def is_occupied(self, real_x, real_y, real_z):
        x, y, z = self.real_to_grid(real_x, real_y, real_z)
        return self.grid[y, x, z] == 1

    def real_to_grid(self, p):
        real_x, real_y, real_z = p
        """Convert real-world coordinates to grid coordinates"""
        grid_x = int((real_x - self.origin_x) / self.resolution)
        grid_y = int((real_y - self.origin_y) / self.resolution)
        grid_z = int((real_z - self.origin_z) / self.resolution)
        return grid_y, grid_x, grid_z

    def grid_to_real(self, grid_x, grid_y, grid_z):
        """Convert grid coordinates to real-world coordinates"""
        real_x = (grid_x * self.resolution) + self.origin_x
        real_y = (grid_y * self.resolution) + self.origin_y
        real_z = (grid_z * self.resolution) + self.origin_z
        return real_x, real_y, real_z
    
    def all_grid_to_real(self, grid_coords):
        """Convert grid coordinates to real-world coordinates"""
        real_coords = []
        for p in grid_coords:
            grid_x, grid_y, grid_z = p
            real_x = (grid_x * self.resolution) + self.origin_x
            real_y = (grid_y * self.resolution) + self.origin_y
            real_z = (grid_z * self.resolution) + self.origin_z
            real_coords.append([real_y, real_x, real_z])
        return np.array(real_coords)

    def display_grid(self):
        print(self.grid)

    def get_grid(self):
        return self.grid
    