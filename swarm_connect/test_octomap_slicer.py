import rclpy
from rclpy.node import Node
from icuas25_eval.srv import GetOctomapSlice
import numpy as np
import os

class OctomapSliceClient(Node):
    def __init__(self):
        super().__init__('octomap_slice_client')
        self.cli = self.create_client(GetOctomapSlice, 'get_occupancy_grid_slice')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req = GetOctomapSlice.Request()

    def send_request(self, height):
        self.req.height = height
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main():
    rclpy.init()
    client_node = OctomapSliceClient()

    # Set height here
    height_value = 1.5
    response = client_node.send_request(height_value)

    if response is not None:
        occupancy_np = np.array(response.occupancy_grid, dtype=np.uint8)
        occupancy_np = occupancy_np.reshape((response.height, response.width))

        save_path = f"/root/CrazySim/ros2_ws/src/meta_packages_hero/icuas25_eval/results/octomap_slice_z{height_value}.npy"
        np.save(save_path, occupancy_np)
        client_node.get_logger().info(f"Occupancy grid saved as '{save_path}' with shape {occupancy_np.shape}")
    else:
        client_node.get_logger().error("Service call failed or returned None")

    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
