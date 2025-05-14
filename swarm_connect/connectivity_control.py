import numpy as np
import math
import argparse
import configparser
from os import path
import os

import tf_transformations

import rclpy
import rclpy.node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

from .modules.connectivity_class import ConnectivityClass
from .modules.occupancy_grid import OccupancyGrid3D
from .utils.utils import *
from .modules.Astar import PathFinder
from .modules.vizualization_manager import VisualizationManager
from .modules.uav import UAV
from .modules.config_manager import load_config_params

class Cons1(rclpy.node.Node):
    """
    Main class to handle UAV formation, battery management,
    and connectivity.
    """

    def __init__(self, node_name: str):
        super().__init__(node_name)

        # Load config file parameters
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').value
        if not config_file:
            self.get_logger().error("No config file provided! Exiting.")
            return
        self.get_logger().info(f"Using config file: {config_file}")
        config = configparser.ConfigParser()
        config.read(config_file)
        config=config[config.default_section]
        load_config_params(self, config)

        # load comm range from setup file
        self.comm_radius = float(os.environ.get('COMM_RANGE', 1))
        config['comm_radius'] = str(self.comm_radius)
        self.get_logger().info(f"Communication range: {self.comm_radius}")

        self.uav_ns = [f'cf_{i+1}' for i in range(self.n_agents)]

        # Create UAV objects
        self.active_uavs = []
        self.grounded_uavs = []

        for i in range(self.n_init_agents):
            self.active_uavs.append(UAV(self, self.uav_ns[i],
                                        self.rate,
                                        self.battery_decay_rate,
                                        self.battery_recharge_rate,
                                        self.battery[i]))

        for i in range(self.n_init_agents, self.n_agents):
            self.grounded_uavs.append(UAV(self, self.uav_ns[i],
                                          self.rate,
                                          self.battery_decay_rate,
                                          self.battery_recharge_rate,
                                          0.8))

        # Initialize charging stations
        spacing = 0.3
        denom = math.ceil(math.sqrt(self.n_agents))
        indices = np.arange(self.n_agents)
        x_coords = (indices % denom) * spacing
        y_coords = (indices // denom) * spacing
        z_coords = np.full(self.n_agents, 0.1)
        self.charging_stations = {(x_coords[i], y_coords[i], z_coords[i]): 
                                  self.uav_ns[i] for i in range(self.n_agents)}

        self.critical_uavs_idx = []
        self.goal_pos = None
        self.pin_agents = None

        # Time handling
        self.clock = Clock(clock_type=ClockType.ROS_TIME)
        self.start_time = self.clock.now()

        self.get_logger().info('Initialization completed...')

        # Flags
        self.is_init_called = False
        self.is_form_called = False
        self.is_flag = False
        self.is_paths_created = False
        self.is_pt_reached = True

        # Configuration values
        self.takeoff_delay = 5  # Replace hardcoded 5
        self.controller_start_delay = 12  # Replace hardcoded 12
        self.critical_battery_level = 0.2  # Adjust this as per the system's need
        self.dead_battery_level = 0.1  # Adjust based on when UAV should land
        self.num_takeoff_retries = 20  # Replace hardcoded retries (previously 5)

        # Changed variables initialization
        self.fiedler_eigenvalue = []  # Changed from self.fiedler
        self.num_active_uavs_list = []  # Changed from self.n_array_list

        # Set up occupancy grid and pathfinder
        self.occupancy_grid = OccupancyGrid3D(self.map_range,
                                              self.map_origin, self.map_res)
        self.path_finder = PathFinder(self.occupancy_grid.get_grid())
        self.all_paths = []

        # ROS 2 Subscribers
        # self.key_vel = None
        # self.create_subscription(Twist, '/cmd_vel', self._set_key_vel, 10)

        # Controllers
        self.connectivity_controller = ConnectivityClass()
        self.connectivity_controller.params_from_cfg(config, 1/self.rate)

        # Set base and robot
        self.base = None
        self.create_subscription(PoseStamped, 'cf_6/pose', self._base_pose_cb,10)  
        self.robot = None
        self.create_subscription(PoseStamped, 'cf_5/pose', self._robot_pose_cb,10)  

        # Visualization manager
        self.viz_manager = VisualizationManager(self)

        # Timers
        self.init_form = self.create_timer(1/self.rate, self.initialize_formation)
        self.run_c = self.create_timer(1/self.rate, self.run)
        self.viz = self.create_timer(0.1, self.viz)

    def _robot_pose_cb(self, msg: PoseStamped):
    
        position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
        orientation_q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        attitude = tf_transformations.euler_from_quaternion(orientation_q)
        # wrap yaw
        if attitude[2] > np.pi:
            attitude[2] -= 2 * np.pi
        elif attitude[2] < -np.pi:
            attitude[2] += 2 * np.pi

        self.robot = position

        self.get_logger().info(f'Robot position: {self.robot}')

        self.connectivity_controller.set_pin_nodes(end=self.robot[:2])
        
    def _base_pose_cb(self, msg: PoseStamped):
    
        position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
        orientation_q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        attitude = tf_transformations.euler_from_quaternion(orientation_q)
        # wrap yaw
        if attitude[2] > np.pi:
            attitude[2] -= 2 * np.pi
        elif attitude[2] < -np.pi:
            attitude[2] += 2 * np.pi

        self.base = position

        self.get_logger().info(f'base position: {self.base}')

        self.connectivity_controller.set_pin_nodes(start=self.base[:2])
        

    def initialize_formation(self):
        """
        Initializes UAV formation. Handles UAV takeoff and checks
        if all UAVs are airborne.
        """
        if not self.is_init_called:
            pose_count = 0
            for uav in self.active_uavs:
                if uav.get_pose() is not None:
                    print(f'takeoff altitude {self.takeoff_alt}')
                    uav.takeoff(self.takeoff_alt)
                    self.free_charging_station(uav)
                    pose_count += 1
            if pose_count == self.n_init_agents:
                self.is_init_called = True
                self.get_logger().info('Takeoffs completed...')

        if self.is_init_called and self.check_time() > self.takeoff_delay:
            battery = np.array([uav.get_battery() for uav in self.active_uavs])
            self.connectivity_controller.reset(battery=battery,
                                               is_gen_lattice=False)

            uav_pos = np.array([uav.get_pose() for uav in self.active_uavs])
            self.goal_pos = uav_pos

            pin_agents = self.connectivity_controller.get_pin_nodes()
            self.pin_agents = np.concatenate(
                (pin_agents,
                 np.full((pin_agents.shape[0], 1), self.takeoff_alt,
                         dtype=pin_agents.dtype)), axis=1)

            self.connectivity_controller.set_positions_uav(uav_pos[:, :2])
            self.is_form_called = True
            self.init_form.cancel()

    def run(self):
        """
        Main function running the connectivity controller,
        UAV battery management, and task scheduling.
        """
        if self.is_form_called and self.check_time() > self.controller_start_delay:
            self.run_controller()
            self.manage_batteries()
            self.handle_landing()


    def run_controller(self):
        """
        Runs the connectivity controller and updates the goal positions
        for all UAVs based on the controller output.
        """
        try:
            battery = np.array([uav.get_battery() for uav in self.active_uavs])
            control_vector = self.connectivity_controller.controller(battery)
            positions, done = self.connectivity_controller.step(control_vector)

            self.pin_agents[:, :2] = positions[:2]
            self.goal_pos[:, :2] = positions[2:]

            # Set z height
            self.goal_pos[:, 2] = self.takeoff_alt

            self.fiedler_eigenvalue.append(self.connectivity_controller.get_fiedler())
            np.save('hardware_fiedler.npy', np.array(self.fiedler_eigenvalue))

            self.num_active_uavs_list.append(self.connectivity_controller.get_n_agents())
            np.save('hardware_n_agents.npy', np.array(self.num_active_uavs_list))

            self.get_logger().debug('Controller updated positions and goals')

        except Exception as e:
            self.get_logger().error(f"Error in running controller: {str(e)}")


    def manage_batteries(self):
        """
        Handles battery checks, UAV deployment, and critical battery-level decisions.
        """
        for i, uav in enumerate(self.active_uavs):
            uav.go_to(self.goal_pos[i])

            if i in self.battery_decay_select or 1000 in self.battery_decay_select:
                uav.decrease_battery()

                if self.is_critical_battery(i, uav):
                    self.critical_uavs_idx.append(i)
                    self.deploy_new_uav(i)


    def is_critical_battery(self, uav_idx, uav):
        """
        Checks if a UAV is in a critical battery condition and needs attention.
        
        Args:
            uav_idx (int): The index of the UAV.
            uav (UAV): The UAV object to check battery level for.

        Returns:
            bool: True if the battery is critical, False otherwise.
        """
        return (uav_idx not in self.critical_uavs_idx and
                uav.get_battery() <= self.critical_battery_level and
                (self.n_init_agents <= int(self.add_uav_limit[0]) or
                self.connectivity_controller.get_fiedler() <= self.add_uav_limit[1]))


    def deploy_new_uav(self, critical_uav_idx):
        """
        Adds a new UAV when the critical battery level is reached.
        """
        try:
            self.connectivity_controller.add_agent(critical_uav_idx)

            self.get_logger().info(f'Adding new agent at {self.goal_pos[-1]}...')

            battery = np.array([uav.get_battery() for uav in self.active_uavs])
            control_vector = self.connectivity_controller.controller(battery)
            positions, done = self.connectivity_controller.step(control_vector)
            
            self.pin_agents[:, :2] = positions[:2]
            self.goal_pos = np.append(self.goal_pos, [self.goal_pos[-1]], axis=0)
            self.goal_pos[:, :2] = positions[2:]

            uav_dist_to_goal = [np.linalg.norm(self.goal_pos[-1] - 
                                uav_g.get_pose()) for uav_g in 
                                self.grounded_uavs if 
                                uav_g.get_battery() > 0.8]

            uav_to_add_idx = np.argmin(uav_dist_to_goal)
            uav_to_add = self.grounded_uavs.pop(uav_to_add_idx)
            path = self.create_paths(uav_to_add, self.goal_pos[-1])

            if path is None:
                self.get_logger().error("Failed to create a valid path for UAV deployment.")
                return

            uav_to_add.set_trajectory(path, mode='trajectory')
            self.active_uavs.append(uav_to_add)

            if self.num_takeoff_retries > 0:
                self.get_logger().info(f'Taking off UAV {uav_to_add.get_name()}')

            for _ in range(self.num_takeoff_retries):  
                self.active_uavs[-1].takeoff(self.takeoff_alt,
                                            mode_change=False)
                self.free_charging_station(self.active_uavs[-1])

            self.n_init_agents += 1

        except Exception as e:
            self.get_logger().error(f"Error during UAV deployment: {str(e)}")


    def handle_landing(self):
        """
        Handles landing UAVs when the battery reaches the critical level
        and checks for free charging stations.
        """
        uavs_to_land = []
        for i, uav in enumerate(self.active_uavs):
            if uav.get_battery() <= self.dead_battery_level:
                uavs_to_land.append(i)

        for i in uavs_to_land:
            uav_to_remove = self.active_uavs.pop(i)
            self.grounded_uavs.append(uav_to_remove)
            self.connectivity_controller.kill_node_i(i)
            self.goal_pos = np.delete(self.goal_pos, i, axis=0)
            self.n_init_agents -= 1
            self.critical_uavs_idx.remove(i)

            station = self.check_free_station(uav_to_remove)
            self.get_logger().info(f'Free charging station: {station}')

            path = self.create_paths(uav_to_remove, station)
            uav_to_remove.set_trajectory(path, 'landing')

        for uav in self.grounded_uavs:
            if uav.mode == 'landing':
                uav.go_to_land()
            else:
                uav.recharge_battery()


    def _set_key_vel(self, msg):
        """
        Callback for handling key velocity commands.
        """
        key_vel = np.array([msg.linear.x, msg.linear.y])
        self.connectivity_controller.update_pin_nodes(u2=key_vel)

    

    def check_time(self):
        """
        Checks the elapsed time since node start.
        """
        elapsed_time = self.clock.now() - self.start_time
        elapsed_seconds = elapsed_time.nanoseconds / 1e9
        return elapsed_seconds

    def create_paths(self, uav, goal):
        """
        Creates paths for UAV to a specific goal using the occupancy grid.
        Handles exceptions that may occur during pathfinding.
        """
        try:
            self.get_logger().debug('Creating path...')

            pos = [uav_i.get_pose() for uav_i in self.active_uavs if uav_i != uav]
            grid = self.occupancy_grid.get_grid_marked(pos)

            self.viz_manager.viz_map(self.occupancy_grid)
            self.path_finder.update_grid(self.occupancy_grid.get_grid())

            uav_pos = uav.get_pose()
            uav_pos[-1] = self.takeoff_alt

            start = self.occupancy_grid.real_to_grid(uav_pos)
            goal = self.occupancy_grid.real_to_grid(goal)

            self.get_logger().debug(f"Searching path from {start} to {goal}")
            path_grid = self.path_finder.search(start, goal)
            path = self.occupancy_grid.all_grid_to_real(path_grid)

            self.get_logger().debug('Path created successfully')
            return path

        except Exception as e:
            self.get_logger().error(f"Error in creating path: {str(e)}")
            return None

    def free_charging_station(self, uav):
        """
        Frees a charging station after a UAV takes off.
        """
        name = uav.get_name()
        for key in self.charging_stations:
            if self.charging_stations[key] == name:
                self.charging_stations[key] = None

    def check_free_station(self, uav):
        """
        Finds the nearest free charging station for a UAV.
        """
        for pos in self.charging_stations:
            if self.charging_stations[pos] is None:
                self.charging_stations[pos] = uav.get_name()
                return pos

    def viz(self):
        """
        Visualizes UAV goals, charging stations, and connectivity edges.
        """
        if self.goal_pos is not None:
            battery = [uav.get_battery() for uav in self.active_uavs]
            self.viz_manager.viz_goals(self.get_clock().now().to_msg(),
                                       self.goal_pos, battery)

            self.viz_manager.viz_charging_stations(self.charging_stations)

        if self.pin_agents is not None:
            self.viz_manager.viz_pins(self.get_clock().now().to_msg(),
                                      self.pin_agents)

        if self.goal_pos is not None and self.pin_agents is not None:
            all_positions = np.vstack([self.pin_agents, self.goal_pos])
            edges = get_edges(all_positions, self.comm_radius)
            self.viz_manager.viz_edges(all_positions, edges)

        all_paths = [uav.get_trajectory() for uav in self.active_uavs if 
                     len(uav.get_trajectory()) != 0]
        self.viz_manager.viz_path(all_paths)
        

def main():
    """
    Main entry point of the script to start the ROS 2 node.
    """
    rclpy.init()

    # Initialize the actual node (it will read config_file itself)
    node = Cons1("basic2")

    rclpy.spin(node)

    node.get_logger().info('Keyboard interrupt, shutting down.\n')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
