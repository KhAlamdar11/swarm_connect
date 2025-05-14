import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import binary_fill_holes, binary_dilation
from scipy.spatial import ConvexHull
import cv2

class InspectionViewpoint:
    def __init__(self, occupancy_grid, z_min=0.0, z_max=5.0, step_height=1.0,
                 dilation_radius=10, offset=5, spiral_rotations=1):
        self.occupancy_grid = occupancy_grid
        self.z_min = z_min
        self.z_max = z_max
        self.step_height = step_height
        self.dilation_radius = dilation_radius
        self.offset = offset
        self.spiral_rotations = spiral_rotations

    def fill_hollow_obstacles(self):
        self.occupancy_grid = binary_fill_holes(self.occupancy_grid).astype(np.uint8)

    def circular_structure(self, radius):
        y, x = np.ogrid[-radius:radius+1, -radius:radius+1]
        return (x**2 + y**2 <= radius**2).astype(bool)

    def dilate_obstacles(self):
        structure = self.circular_structure(self.dilation_radius)
        self.occupancy_grid = binary_dilation(self.occupancy_grid, structure=structure).astype(np.uint8)

    def compute_viewpoints(self):
        grad_x = cv2.Sobel(self.occupancy_grid.astype(np.float32), cv2.CV_64F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(self.occupancy_grid.astype(np.float32), cv2.CV_64F, 0, 1, ksize=3)
        grad_magnitude = np.sqrt(grad_x**2 + grad_y**2)
        grad_orientation = np.arctan2(grad_y, grad_x)

        edge_positions = np.argwhere(grad_magnitude > 0.5)

        waypoints = []
        for y, x in edge_positions:
            if self.occupancy_grid[y, x] == 0:
                angle = grad_orientation[y, x]
                x_new = int(x - self.offset * np.cos(angle))
                y_new = int(y - self.offset * np.sin(angle))
                if (0 <= x_new < self.occupancy_grid.shape[1] and 
                    0 <= y_new < self.occupancy_grid.shape[0] and 
                    self.occupancy_grid[y_new, x_new] == 0):
                    waypoints.append(((x_new, y_new), angle))
        return waypoints

    def order_boundary_points(self, waypoints):
        if len(waypoints) < 3:
            return waypoints
        points = np.array([pt for pt, _ in waypoints])
        angles = np.array([a for _, a in waypoints])
        hull = ConvexHull(points)
        return [((int(points[i][0]), int(points[i][1])), float(angles[i])) for i in hull.vertices]

    def generate_spiral_trajectory(self, waypoints):
        num_levels = int((self.z_max - self.z_min) / self.step_height)
        spiral_waypoints = []
        for i in range(num_levels):
            z = self.z_min + i * self.step_height
            angle_offset = (i / num_levels) * (2 * np.pi * self.spiral_rotations)
            for (x, y), angle in waypoints:
                x_new = x + 2 * np.cos(angle_offset)
                y_new = y + 2 * np.sin(angle_offset)
                spiral_waypoints.append(((x_new, y_new, z), angle))
        return spiral_waypoints



    def generate(self, visualize=False):
        self.fill_hollow_obstacles()
        self.dilate_obstacles()
        waypoints_2d = self.compute_viewpoints()
        ordered_2d = self.order_boundary_points(waypoints_2d)
        if visualize:
            self.visualize_2d(ordered_2d)
        return self.generate_spiral_trajectory(ordered_2d)

    def visualize_2d(self, waypoints):
        plt.figure(figsize=(8, 8))
        plt.imshow(self.occupancy_grid, cmap='gray_r')
        for (x, y), angle in waypoints:
            plt.arrow(x, y, np.cos(angle), np.sin(angle), color='red', head_width=2, head_length=2)
        plt.title("2D Waypoints")
        plt.show()