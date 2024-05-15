import rclpy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion, Point, Vector3, Twist
from nav_msgs.msg import Odometry, Path
import tf_transformations

import numpy as np

# TODO: Reduce reduncancy of creation of markers every time. Create a single function for that.

'''
Base class for visualization management
'''

class VisualizationManager:
    def __init__(self, node):
    
        self.viz_goals_pub = node.create_publisher(MarkerArray, f'{node.get_name()}/viz_goals', 10)
        self.viz_pins_pub = node.create_publisher(MarkerArray, f'{node.get_name()}/viz_pins', 10)
        self.viz_edges_pub = node.create_publisher(Marker, f'{node.get_name()}/viz_edges', 10)
        self.viz_map_pub = node.create_publisher(MarkerArray, f'{node.get_name()}/viz_map', 10)
        self.viz_anon_pt_pub = node.create_publisher(MarkerArray, f'{node.get_name()}/viz_anon_pt', 10)
        self.viz_path_pub = node.create_publisher(Marker, f'{node.get_name()}/viz_path', 10)


    def viz_pins(self, time, pin_agents):
        '''
        for pinned agents as cylinders
        Color: Yellow
        '''
        marker_array = MarkerArray()
        for i in range(2):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.type = Marker.CYLINDER
            marker.id = i  
            marker.header.stamp = time #self.get_clock().now().to_msg()
            marker.pose.position.x = pin_agents[i,0]
            marker.pose.position.y = pin_agents[i,1]
            marker.pose.position.z = pin_agents[i,2]/2 
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = pin_agents[i,2]*1.2
            marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
            marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()

            # Replace with the actual angle
            quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
            marker.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

            marker_array.markers.append(marker)

        self.viz_pins_pub.publish(marker_array)


    def viz_goals(self, time, goal_pos, battery):
        '''
        Show position of goals
        Color: Blue
        '''
        # clear all old markers
        # Clear all markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        self.viz_goals_pub.publish(MarkerArray(markers=[clear_marker]))

        marker_array = MarkerArray()
        for i in range(goal_pos.shape[0]):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.type = Marker.SPHERE
            marker.id = i  # Assuming ns is unique for each UAV
            marker.header.stamp = time
            marker.pose.position.x = goal_pos[i,0]
            marker.pose.position.y = goal_pos[i,1]
            marker.pose.position.z = goal_pos[i,2]
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.color = ColorRGBA(r=0.0, g=0.0, b=battery[i], a=1.0)
            marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()

            # Replace with the actual angle
            quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
            marker.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

            marker_array.markers.append(marker)

        self.viz_goals_pub.publish(marker_array)


    def viz_map(self,occupancy_grid):
        '''
        shows map. Just ibstacle positions!
        Color: Black
        '''

        grid = occupancy_grid.get_grid()
        res = occupancy_grid.resolution

        marker_array = MarkerArray()
        id = 0
        for z in range(grid.shape[2]):
            for y in range(grid.shape[1]):
                for x in range(grid.shape[0]):
                    if grid[x, y, z] != 0:  # only show occupied cells!
                        marker = Marker()
                        marker.header.frame_id = "world"
                        marker.type = marker.CUBE
                        marker.action = marker.ADD
                        marker.id = id
                        id += 1
                        marker.scale.x = 0.95 * res
                        marker.scale.y = 0.95 * res
                        marker.scale.z = 0.95 * res
                        marker.color.a = 0.01  if grid[x, y, z] == 0 else 0.8
                        marker.color.r = 1.0 if grid[x, y, z] == 0 else 0.0
                        marker.color.g = 1.0 if grid[x, y, z] == 0 else 0.0
                        marker.color.b = 1.0 if grid[x, y, z] == 0 else 0.0
                        real_x, real_y, real_z = occupancy_grid.grid_to_real(x, y, z)
                        marker.pose.position.x = real_x
                        marker.pose.position.y = real_y
                        marker.pose.position.z = real_z
                        marker.pose.orientation.w = 1.0
                        marker_array.markers.append(marker)

        self.viz_map_pub.publish(marker_array)


    def viz_edges(self, all_positions, edges):
        '''
        visualize the edges between given positions!
        color: grey
        '''
        edge_marker = Marker(
                header=Header(frame_id="world"),
                type=Marker.LINE_LIST,
                action=Marker.ADD,
                color=ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0),
                scale=Vector3(x=0.07, y=0.07, z=0.07),
                lifetime=rclpy.duration.Duration(seconds=0.1).to_msg(),
                id = 0,
                ns = "lines"
            )
        pts = []
        for e in edges:
            u, v = all_positions[e[0]], all_positions[e[1]]
            # print(u,v)
            pts.append(Point(x=u[0], y=u[1], z=u[2]))
            pts.append(Point(x=v[0], y=v[1], z=u[2]))
        edge_marker.points = pts

        self.viz_edges_pub.publish(edge_marker)

    
    def viz_path(self, all_paths):
        edge_marker = Marker(
                header=Header(frame_id="world"),
                type=Marker.LINE_LIST,
                action=Marker.ADD,
                color=ColorRGBA(r=0.7, g=0.0, b=0.0, a=1.0),
                scale=Vector3(x=0.03, y=0.03, z=0.03),
                lifetime=rclpy.duration.Duration(seconds=0.1).to_msg(),
                id = 0,
                ns = "lines"
            )
        pts = []
        for path in all_paths:
            if path is not None:
                for i in range(len(path[:-1])):
                    u, v = path[i], path[i+1]
                    # print(u,v)
                    pts.append(Point(x=u[0], y=u[1], z=u[2]))
                    pts.append(Point(x=v[0], y=v[1], z=u[2]))
        edge_marker.points = pts

        self.viz_path_pub.publish(edge_marker)


    def viz_anon_pt(self,x,y,z):
        '''
        utils function to create a marker at the spcified position.
        color: purple
        '''
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = Marker.SPHERE
        marker.id = 0  # Assuming ns is unique for each UAV
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z 
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color = ColorRGBA(r=0.627, g=0.125, b=0.941, a=1.0)
        marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()

        # Replace with the actual angle
        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        marker.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

        marker_array.markers.append(marker)

        self.viz_anon_pt_pub.publish(marker_array)