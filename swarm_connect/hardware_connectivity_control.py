import numpy as np
import math

import argparse
import configparser
from os import path

from crazyflie_py import *
import rclpy 
import rclpy.node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist

from .modules.connectivity_class import ConnectivityClass
from .modules.occupancy_grid import OccupancyGrid3D
from .utils.utils import *
from .modules.Astar import PathFinder
from .modules.vizualization_manager import VisualizationManager
from .modules.uav_hardware import UAV
from .modules.config_manager import load_config_params

import time

class Cons1(rclpy.node.Node):
    def __init__(self, node_name: str, config):
        super().__init__(node_name)
        
        load_config_params(self,config)

        self.uav_ns = []
        for i in range(self.n_agents):
            self.uav_ns.append(f'cf{i+1}')

        '''
        # create UAV objects
        each uav is created and stored as an object in this ordered list. This list ensures ordering is constant especially for the algorithm to work.
        '''
        self.active_uavs = []
        self.grounded_uavs = []
        for i in range(self.n_init_agents):
            self.active_uavs.append(UAV(self, self.uav_ns[i],
                                              self.rate,
                                              self.battery_decay_rate,
                                              self.battery_recharge_rate,
                                              self.battery[i]))
        
        # set full battery for grounded uavs
        for i in range(self.n_init_agents,self.n_agents):
            self.grounded_uavs.append(UAV(self, self.uav_ns[i],
                                                self.rate,
                                                self.battery_decay_rate,
                                                self.battery_recharge_rate,
                                                0.8))
            
        self.charging_stations = None

        # uav index that are below a thresh2 and above thresh1 ie. in new deployment agent range
        self.critical_uavs_idx = [] 

        self.goal_pos = None
        self.pin_agents = None

        # Time
        self.clock = Clock(clock_type=ClockType.ROS_TIME)
        self.start_time = self.clock.now()  # Record the start time when the node is initialized

        self.get_logger().info('Initialization completed...')

        # init flags
        self.is_init_called = False        
        self.is_form_called = False
        self.is_flag = False

        self.is_paths_created = False
        self.is_pt_reached = True

        self.fiedler = []
        self.n_array_list = []
        
        self.occupancy_grid = OccupancyGrid3D(self.map_range,self.map_origin,self.map_res)

        self.path_finder = PathFinder(self.occupancy_grid.get_grid())
        self.all_paths = []  # self.all_paths = [None for i in self.uav_namespaces]
        
        #_________________________  Subs  _________________________
        self.key_vel = None
        self.create_subscription(Twist,'/cmd_vel',self._set_key_vel,10)

        #_________________________ Controllers  _____________________
        self.connectivity_controller = ConnectivityClass()
        self.connectivity_controller.params_from_cfg(config,1/self.rate)

        #_________________________ Vizualizations  _____________________
        self.viz_manager = VisualizationManager(self)

        #_________________________ Timers  _____________________
        # initialize formation
        self.init_form = self.create_timer(1/self.rate, self.initialize_formation)
        
        # run controller
        self.run_c = self.create_timer(1/self.rate, self.run)

        # safety landing
        # self.safety_landing_c = self.create_timer(1/self.rate, self.safety_landing)

        # self.test_trajectory_t = self.create_timer(0.5, self.test_trajectory)
        # self.is_traj_test = False

        # visualizers
        self.viz = self.create_timer(0.1, self.viz)


    def initialize_formation(self):

        if not self.is_init_called:
            self.initialize_charging_stations()
            pose_count = 0
            for uav in self.active_uavs:
                if uav.get_pose() is not None:
                    uav.takeoff(self.takeoff_alt)
                    self.free_charging_station(uav)
                    pose_count+=1
            if pose_count==self.n_init_agents:
                self.is_init_called = True
                self.get_logger().info('Takeoffs completed...')

        if self.is_init_called and self.check_time() > 5:

            # get batteries
            battery = []
            for i in range(len(self.active_uavs)):
                battery.append(self.active_uavs[i].get_battery())
            battery = np.array(battery)
            self.connectivity_controller.reset(battery=battery,is_gen_lattice = False)

            # Get all UAV poses
            uav_pos = np.array([uav.get_pose() for uav in self.active_uavs])

            self.goal_pos = uav_pos

            # Get positions of pinned agents
            pin_agents = self.connectivity_controller.get_pin_nodes()
            self.pin_agents = np.concatenate((pin_agents, np.full((pin_agents.shape[0], 1), self.takeoff_alt, dtype=pin_agents.dtype)), axis=1)

            # update position ordering in controller
            self.connectivity_controller.set_positions_uav(uav_pos[:,:2])

            # time.sleep(3)
            self.get_logger().info('Connectivity controller initialized...')

            self.is_form_called = True
            self.init_form.cancel()

    def safety_landing(self):
        if self.check_time() > 150:
            self.get_logger().info('Safety landing initiated...')
            for uav in self.active_uavs:
                if uav.get_pose() is not None:
                    uav.land()
                    self.free_charging_station(uav)

                    self.init_form.cancel()
                    self.safety_landing_c.cancel()
                    self.run_c.cancel()


    def run(self):
        '''
        runs connectivity controller
        '''
        if self.is_form_called and self.check_time() > 10:
            # run controller and get new positions


            # get batteries
            battery = []
            for i in range(len(self.active_uavs)):
                battery.append(self.active_uavs[i].get_battery())
            battery = np.array(battery)

            v = self.connectivity_controller.controller(battery)
            p, done = self.connectivity_controller.step(v)
            self.pin_agents[:,:2] = p[:2]
            self.goal_pos[:,:2] = p[2:]

            # save fiedler value
            self.fiedler.append(self.connectivity_controller.get_fiedler())
            np.save('hardware_fiedler.npy', np.array(self.fiedler))  
            self.n_array_list.append(self.connectivity_controller.get_n_agents())
            np.save('hardware_n_agents.npy', np.array(self.n_array_list))  

            # call go to to get UAVs to new positions and update batteries
            uavs_to_land = []
            for i in range(len(self.active_uavs)):
                self.active_uavs[i].go_to(self.goal_pos[i])
                
                # battery management below
                if i in self.battery_decay_select or 1000 in self.battery_decay_select:
                    self.active_uavs[i].decrease_battery()

                    if self.active_uavs[i].get_battery() <= self.dead_battery_level:
                        uavs_to_land.append(i)
                        print('2. ',self.critical_uavs_idx)
                        self.get_logger().info(f'UAVs to land: {uavs_to_land}')


            for i in uavs_to_land:
                uav_to_remove = self.active_uavs.pop(i)
                self.grounded_uavs.append(uav_to_remove)
                self.connectivity_controller.kill_node_i(i)
                self.goal_pos = np.delete(self.goal_pos, i, axis=0)
                self.n_init_agents-=1
                # TODO: Fix bug of why land must be called twice
                uav_to_remove.land()


                # pass
        # print(self.charging_stations)
        # print('--------------------------------------------')
        # for uav in self.grounded_uavs:
        #     print(uav.mode)
        # for uav in self.active_uavs:
        #     print(uav.mode)
        # print('--------------------------------------------')



    #_________________________  Subs  _________________________
                 
    # subscribes to the keyboard cmd, and moves the pinned node.
    def _set_key_vel(self,msg):
        key_vel = np.array([msg.linear.x, msg.linear.y])
        self.connectivity_controller.update_pin_nodes(u2=key_vel)

    #__________________________  Utils  _______________________

    def check_time(self):
        elapsed_time = self.clock.now() - self.start_time
        elapsed_seconds = elapsed_time.nanoseconds / 1e9
        return elapsed_seconds  

    def create_paths(self,uav,goal):

        self.get_logger().info('Creating Path...')

        pos = [uav_i.get_pose() for uav_i in self.active_uavs if uav_i!=uav]
        grid = self.occupancy_grid.get_grid_marked(pos)

        self.viz_manager.viz_map(self.occupancy_grid)
        
        self.path_finder.update_grid(self.occupancy_grid.get_grid())

        uav_pos = uav.get_pose()
        # start path from a bit of heigh in case UAV hasnt fully taken off
        uav_pos[-1] = self.takeoff_alt/2

        start = self.occupancy_grid.real_to_grid(uav_pos)
        goal = self.occupancy_grid.real_to_grid(goal)

        self.get_logger().info(f"Searching path from {uav.get_pose()} to {self.goal_pos[-1]}")
        path_grid = self.path_finder.search(start, goal)
        path = self.occupancy_grid.all_grid_to_real(path_grid)
        
        # self.all_paths.append(path)

        return path
    

    def free_charging_station(self,uav):
        name = uav.get_name()
        for key in self.charging_stations:
            if self.charging_stations[key] == name:
                self.charging_stations[key] = None

    def initialize_charging_stations(self):
        
        self.charging_stations = {}
        is_charging_stations_initialized = False
        all_uavs = np.concatenate((self.active_uavs, self.grounded_uavs))

        while not(is_charging_stations_initialized):
            is_charging_stations_initialized = True
            for uav in all_uavs:
                pos = uav.get_pose()
                if pos is not None:
                   self.charging_stations[(pos[0],pos[1],0.05)] = uav.get_name()
                else:
                    is_charging_stations_initialized = False


    def check_free_station(self,uav):
        uav_pos = uav.get_pose()

        distances = []
        for pos in self.charging_stations:                
            if self.charging_stations[pos] is None:
                self.charging_stations[pos] = uav.get_name()
                return pos

        # TODO: Add functionality to go to the nearest charging startion
        #             distances.append(np.linalg.norm(station - uav_pos))
        #         else:
        #             distances.append(math.inf)
        
        # for i in 

    #_________________________  Viz  _________________________

    def viz(self):

        if self.goal_pos is not None:
            battery = []
            for i in range(len(self.active_uavs)):
                battery.append(self.active_uavs[i].get_battery())
            self.viz_manager.viz_goals(self.get_clock().now().to_msg(),
                                      self.goal_pos,
                                      battery)

        if self.charging_stations is not None:     
            self.viz_manager.viz_charging_stations(self.charging_stations)

        if self.pin_agents is not None:
            self.viz_manager.viz_pins(self.get_clock().now().to_msg(),
                                      self.pin_agents)

        if self.goal_pos is not None and self.pin_agents is not None:
            
            all_positions = np.vstack([self.pin_agents,self.goal_pos])
            edges = get_edges(all_positions,self.comm_radius*1.05)
            
            self.viz_manager.viz_edges(all_positions,edges)
        
        all_paths = []
        for uav in self.active_uavs:
            traj = uav.get_trajectory()
            if len(traj) != 0: 
                all_paths.append(traj)
        self.viz_manager.viz_path(all_paths)


def main():
    
    # Load config file from arguments
    parser = argparse.ArgumentParser(description='Start a Basic1 node with configuration.')
    parser.add_argument('config', type=str, help='Path to the configuration file.')
    args = parser.parse_args()
    config_file = path.abspath(args.config)  # Ensure the path is absolute
    print(f"Using config file: {config_file}")
    config = configparser.ConfigParser()
    config.read(config_file)

    rclpy.init()
    
    node = Cons1("basic2", config=config[config.default_section])

    rclpy.spin(node)    
        
    node.get_logger().info('Keyboard interrupt, shutting down.\n')

    rclpy.shutdown()

if __name__ == '__main__':
   main()
