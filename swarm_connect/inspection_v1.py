import rclpy
from rclpy.node import Node
from icuas25_eval.srv import GetOctomapSlice
from icuas25_eval.srv import CheckLine, GetOctomapBounds
import numpy as np
import os
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
import tf_transformations


from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from visualization_msgs.msg import Marker, MarkerArray

from swarm_connect.modules.inspection_viewpoint import InspectionViewpoint

class InspectionBeacons(Node):
    def __init__(self):
        super().__init__('inspection_v1')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )


        # Parameters
        self.slice_height = 1.5

        # Get map bounds
        self.bounds_client = self.create_client(GetOctomapBounds, '/get_octomap_bounds')
        self.get_logger().info("Waiting for /get_octomap_bounds service...")
        self.bounds_client.wait_for_service()
        self.octomap_bounds = self.call_get_octomap_bounds()
        self.get_logger().info(f'Octomap Bounds: {self.octomap_bounds}...')

        # Subscribe to the occupancy grid to get map origin and resolution
        self.map_resolution, self.map_origin = None, None
        self.occupancy_grid_sub = self.create_subscription(OccupancyGrid,'/projected_map',self.occupancy_grid_cb,qos)

        # Get octomap slice as binary occupancy grid
        self.get_occupancy_grid_slice_client = self.create_client(GetOctomapSlice, 'get_occupancy_grid_slice')
        while not self.get_occupancy_grid_slice_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service (get_occupancy_grid_slice) not available, waiting...')
        self.req = GetOctomapSlice.Request()
        response = self.send_request()
        if response is not None:
            self.occupancy_grid = np.array(response.occupancy_grid, dtype=np.uint8)
            self.occupancy_grid = self.occupancy_grid.reshape((response.height, response.width))

    
        # Initialize the generator
        ivp = InspectionViewpoint(occupancy_grid=self.occupancy_grid,
                                  z_min=1.0,
                                  z_max=self.octomap_bounds[2][1]+0.2,
                                  step_height=0.4,
                                  dilation_radius=13,
                                  offset=5,
                                  spiral_rotations=2)

        # Generate 3D spiral waypoints
        self.waypoints = ivp.generate(visualize=False)

        self.waypoints_world = []

        # # Visualize
        self.frame_id = 'world'
        self.path_pub = self.create_publisher(Path, 'spiral_path', 10)

        self.create_timer(1.0, self.run_)


    def run_(self):
        if self.map_origin is not None and self.map_resolution is not None:
            print("IN RUN")
            for (gx, gy, z), angle in self.waypoints:
                wx, wy = self.grid_to_world(gx, gy)
                self.waypoints_world.append(((wx, wy, z), angle))
            np.save('/root/CrazySim/ros2_ws/src/meta_packages_hero/swarm_connect/data/waypoints_box_1.npy', self.waypoints_world)
            self.vis_path()

    def vis_path(self):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.frame_id

        for (x, y, z), angle in self.waypoints_world:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)

            # Convert yaw (angle) to quaternion
            q = tf_transformations.quaternion_from_euler(0, 0, angle)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published spiral path with {len(path_msg.poses)} poses.")
        self.waypoints_world = []



    def send_request(self):
        self.req.height = self.slice_height
        future = self.get_occupancy_grid_slice_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def call_get_octomap_bounds(self):
        """ Calls the get_octomap_bounds service synchronously """
        request = GetOctomapBounds.Request()
        future = self.bounds_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            response = future.result()
            return ((response.min_x, response.max_x),(response.min_y, response.max_y),(response.min_z, response.max_z))
        else:
            self.get_logger().error("Service call failed!")
            return None

    def occupancy_grid_cb(self, msg: OccupancyGrid):
        print("📡 RECEIVED MAP MESSAGE!")
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.get_logger().info(f"Map origin: {self.map_origin}, resolution: {self.map_resolution}")


    def grid_to_world(self, gx, gy):
        if self.map_resolution is None or self.map_origin is None:
            raise RuntimeError("Map origin/resolution not yet received.")
        
        wx = self.map_origin[0] + (gx + 0.5) * self.map_resolution
        wy = self.map_origin[1] + (gy + 0.5) * self.map_resolution
        return wx, wy


def main():

    rclpy.init()

    node = InspectionBeacons()

    rclpy.spin(node)

    node.get_logger().info('Keyboard interrupt, shutting down.\n')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
