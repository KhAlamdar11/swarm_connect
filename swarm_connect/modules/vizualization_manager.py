import rclpy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Quaternion, Point, Vector3
from nav_msgs.msg import Path
import tf_transformations
import numpy as np

class VisualizationManager:
    def __init__(self, node):
        self.node = node
        self.viz_goals_pub = node.create_publisher(MarkerArray, f'{node.get_name()}/viz_goals', 10)
        self.viz_pins_pub = node.create_publisher(MarkerArray, f'{node.get_name()}/viz_pins', 10)
        self.viz_edges_pub = node.create_publisher(Marker, f'{node.get_name()}/viz_edges', 10)
        self.viz_map_pub = node.create_publisher(MarkerArray, f'{node.get_name()}/viz_map', 10)
        self.viz_anon_pt_pub = node.create_publisher(MarkerArray, f'{node.get_name()}/viz_anon_pt', 10)
        self.viz_path_pub = node.create_publisher(Marker, f'{node.get_name()}/viz_path', 10)
        self.viz_charging_stations_pub = node.create_publisher(MarkerArray, f'{node.get_name()}/viz_charging_stations', 10)

    def create_marker(self, marker_id, marker_type, frame_id="world", scale=(0.15, 0.15, 0.15), color=(1.0, 0.0, 0.0, 1.0), lifetime=0.1):
        """Helper function to create and initialize a marker."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = marker_type
        marker.id = marker_id
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        marker.lifetime = rclpy.duration.Duration(seconds=lifetime).to_msg()
        marker.pose.orientation = self.get_default_orientation()
        return marker

    def get_default_orientation(self):
        """Return a default orientation (identity quaternion)."""
        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        return Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

    def viz_pins(self, time, pin_agents):
        """Visualize pinned agents as cylinders."""
        marker_array = MarkerArray()
        for i in range(2):
            marker = self.create_marker(i, Marker.CYLINDER, scale=(0.15, 0.15, pin_agents[i, 2] * 1.2), color=(1.0, 0.0, 0.0, 1.0))
            marker.header.stamp = time
            marker.pose.position.x = pin_agents[i, 0]
            marker.pose.position.y = pin_agents[i, 1]
            marker.pose.position.z = pin_agents[i, 2] / 2
            marker_array.markers.append(marker)
        self.viz_pins_pub.publish(marker_array)

    def viz_goals(self, time, goal_pos, battery):
        """Visualize UAV goal positions."""
        self.viz_goals_pub.publish(MarkerArray(markers=[self.create_marker(0, Marker.DELETEALL)]))  # Clear old markers
        marker_array = MarkerArray()
        for i in range(goal_pos.shape[0]):
            marker = self.create_marker(i, Marker.SPHERE, color=(0.0, 0.0, battery[i], 1.0))
            marker.header.stamp = time
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = goal_pos[i]
            marker_array.markers.append(marker)
        self.viz_goals_pub.publish(marker_array)

    def viz_map(self, occupancy_grid):
        """Visualize the occupancy grid (obstacle positions)."""
        grid = occupancy_grid.get_grid()
        res = occupancy_grid.resolution
        marker_array = MarkerArray()
        id = 0
        for z in range(grid.shape[2]):
            for y in range(grid.shape[1]):
                for x in range(grid.shape[0]):
                    if grid[x, y, z] != 0:  # Only show occupied cells
                        marker = self.create_marker(id, Marker.CUBE, scale=(0.95 * res, 0.95 * res, 0.95 * res), color=(0.0, 0.0, 0.0, 0.8))
                        real_x, real_y, real_z = occupancy_grid.grid_to_real(x, y, z)
                        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = real_x, real_y, real_z
                        marker_array.markers.append(marker)
                        id += 1
        self.viz_map_pub.publish(marker_array)

    def viz_edges(self, all_positions, edges):
        """Visualize edges between positions."""
        edge_marker = self.create_marker(0, Marker.LINE_LIST, scale=(0.03, 0.03, 0.03), color=(0.5, 0.5, 0.5, 1.0))
        pts = [Point(x=all_positions[u][0], y=all_positions[u][1], z=all_positions[u][2]) for e in edges for u in e]
        edge_marker.points = pts
        self.viz_edges_pub.publish(edge_marker)

    def viz_path(self, all_paths):
        """Visualize UAV paths."""
        edge_marker = self.create_marker(0, Marker.LINE_LIST, scale=(0.03, 0.03, 0.03), color=(0.7, 0.0, 0.0, 1.0))
        pts = [Point(x=u[0], y=u[1], z=u[2]) for path in all_paths if path is not None for u, v in zip(path[:-1], path[1:])]
        edge_marker.points = pts
        self.viz_path_pub.publish(edge_marker)

    def viz_anon_pt(self, x, y, z):
        """Visualize an anonymous point (Purple sphere)."""
        marker = self.create_marker(0, Marker.SPHERE, scale=(0.1, 0.1, 0.1), color=(0.627, 0.125, 0.941, 1.0))
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, z
        self.viz_anon_pt_pub.publish(MarkerArray(markers=[marker]))

    def viz_charging_stations(self, charging_stations):
        """Visualize charging stations."""
        marker_array = MarkerArray()
        for i, pos in enumerate(charging_stations):
            color = (0.0, 1.0, 0.0, 1.0) if charging_stations[pos] is None else (1.0, 1.0, 0.0, 1.0)
            marker = self.create_marker(i, Marker.CUBE, scale=(0.25, 0.25, 0.05), color=color)
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = pos[0], pos[1], 0.025
            marker_array.markers.append(marker)
        self.viz_charging_stations_pub.publish(marker_array)
