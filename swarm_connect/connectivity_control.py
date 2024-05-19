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
from .modules.uav import UAV
from .modules.config_manager import load_config_params

import time

class Cons1(rclpy.node.Node):
    def __init__(self, node_name: str, config):
        super().__init__(node_name)
        
        load_config_params(self,config)

        self.uav_ns = []
        for i in range(self.n_agents):
            self.uav_ns.append(f'cf_{i+1}')

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
            
        # initialize charging stations
        spacing = 0.3
        denom = math.ceil(math.sqrt(self.n_agents))
        indices = np.arange(self.n_agents)
        x_coords = (indices % denom) * spacing
        y_coords = (indices // denom) * spacing
        self.charging_pos = np.column_stack((x_coords, y_coords, np.full(self.n_agents, 0.1)))

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

        # self.test_trajectory_t = self.create_timer(0.5, self.test_trajectory)
        # self.is_traj_test = False

        # visualizers
        self.viz = self.create_timer(0.5, self.viz)


    def initialize_formation(self):

        if not self.is_init_called:
            pose_count = 0
            for uav in self.active_uavs:
                if uav.get_pose() is not None:
                    uav.takeoff(self.takeoff_alt)
                    pose_count+=1
            if pose_count==self.n_init_agents:
                self.is_init_called = True
                self.get_logger().info('Takeoffs completed...')
                # time.sleep(5)
            

        if self.is_init_called and self.check_time() > 5:

            self.connectivity_controller.reset(is_gen_lattice = False)

            # Get all UAV poses
            uav_pos = np.array([uav.get_pose() for uav in self.active_uavs])

            self.goal_pos = uav_pos

            # Get positions of pinned agents
            pin_agents = self.connectivity_controller.get_pin_nodes()
            self.pin_agents = np.concatenate((pin_agents, np.full((pin_agents.shape[0], 1), self.takeoff_alt, dtype=pin_agents.dtype)), axis=1)

            # update position ordering in controller
            self.connectivity_controller.set_positions_uav(uav_pos[:,:2])

            # time.sleep(3)

            self.is_form_called = True
            self.init_form.cancel()


    def run(self):
        '''
        runs connectivity controller
        '''
        if self.is_form_called and self.check_time() > 12:
            # run controller and get new positions
            v = self.connectivity_controller.controller()
            p, done = self.connectivity_controller.step(v)
            self.pin_agents[:,:2] = p[:2]
            self.goal_pos[:,:2] = p[2:]

            # call go to to get UAVs to new positions and update batteries
            uavs_to_land = []
            for i in range(len(self.active_uavs)):
                self.active_uavs[i].go_to(self.goal_pos[i])
                
                # battery management below
                if i in self.battery_decay_select or 1000 in self.battery_decay_select:
                    self.active_uavs[i].decrease_battery()

                    # Check if new agent has not already been deployed for the new agent   
                    # And check for critical conditions                 
                    if i not in self.critical_uavs_idx and \
                                self.active_uavs[i].get_battery() <= self.critical_battery_level and \
                                (self.n_init_agents <= int(self.add_uav_limit[0]) or \
                                self.connectivity_controller.get_fiedler() <= self.add_uav_limit[1]):
                        
                        self.critical_uavs_idx.append(i)

                        # add a new agent 'for' this agent
                        self.connectivity_controller.add_agent(i)  
                            
                        self.get_logger().info(f'Ading new agent at {self.goal_pos[-1]}...')
                        v = self.connectivity_controller.controller()
                        p, done = self.connectivity_controller.step(v)
                        self.pin_agents[:,:2] = p[:2]
                        # duplicate last row to increase size of goal positions and keep heigh var
                        self.goal_pos = np.append(self.goal_pos, [self.goal_pos[-1]], axis=0)
                        self.goal_pos[:,:2] = p[2:]

                        # find the closet available uav on ground to the goal
                        uav_dist_to_goal = []
                        for uav in self.grounded_uavs:
                            # print(uav.get_name(), uav.get_battery())
                            if uav.get_battery() > 0.8: 
                                uav_dist_to_goal.append(np.linalg.norm(self.goal_pos[-1] - 
                                                                        uav.get_pose()))
                            
                        uav_to_add_idx = np.argmin(uav_dist_to_goal)

                        # pop the uav from grounded list and add to active
                        uav_to_add = self.grounded_uavs.pop(uav_to_add_idx)
                        path = self.create_paths(uav_to_add,self.goal_pos[-1])
                        uav_to_add.set_trajectory(path)
                        
                        self.active_uavs.append(uav_to_add)
                        for i in range(5): # client dropping msg issue!!!
                            self.active_uavs[-1].takeoff(self.takeoff_alt/2)
                        self.n_init_agents+=1
                        

                    if self.active_uavs[i].get_battery() <= self.dead_battery_level:
                        uavs_to_land.append(i)
                        self.get_logger().info(f'UAVs to land: {uavs_to_land}')


            for i in uavs_to_land:
                uav_to_remove = self.active_uavs.pop(i)
                self.grounded_uavs.append(uav_to_remove)
                self.connectivity_controller.kill_node_i(i)
                self.goal_pos = np.delete(self.goal_pos, i, axis=0)
                self.n_init_agents-=1
                self.critical_uavs_idx.remove(i)
                # TODO: Fix bug of why land must be called twice
                # uav_to_remove.land()
                # uav_to_remove.land()

                station = self.check_free_station(uav_to_remove)
                self.get_logger().info(f'Free charging station: {station}')
                
                # plan path to charging station
                path = self.create_paths(uav_to_remove,station)
                uav_to_remove.set_trajectory(path,'landing')

            # recharge battery of UAVs on ground!
            for uav in self.grounded_uavs:
                # keep landing if not yet landed...
                if uav.mode == 'landing':
                    uav.go_to_land()
                else:
                    uav.recharge_battery()


    #_________________________  Tests  _________________________
    def test_trajectory(self):

        if self.check_time() > 5 and not(self.is_traj_test):

            uav = self.active_uavs[0]

            self.get_logger().info('Creating Test Path...')

            pos = [uav_i.get_pose() for uav_i in self.active_uavs if uav_i!=uav]

            grid = self.occupancy_grid.get_grid_marked(pos)

            self.viz_manager.viz_map(self.occupancy_grid)
            
            self.path_finder.update_grid(grid)

            start = uav.get_pose()
            goal = np.array([0,0, self.takeoff_alt])

            start = self.occupancy_grid.real_to_grid(start)
            goal = self.occupancy_grid.real_to_grid(goal)

            self.get_logger().info(f"Searching path from {start} to {goal}")
            path_grid = self.path_finder.search(start, goal)
            path = self.occupancy_grid.all_grid_to_real(path_grid)
            
            # # Just for displaying path. Comment the run timer while running this test!
            self.active_uavs[0].set_trajectory(path)

            self.is_traj_test = True

        if self.is_traj_test:
            self.active_uavs[0].go_to(None)            


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
    
    def check_free_station(self,uav):
        uav_pos = uav.get_pose()
        uav_poses = [uav_i.get_pose() for uav_i in self.active_uavs if uav_i!=uav]
        
        distances = []
        for station in self.charging_pos:
            for uav_i_pose in uav_poses:
                if np.linalg.norm(station - uav_i_pose) > 0.15:
                    return station

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

        if self.pin_agents is not None:
            self.viz_manager.viz_pins(self.get_clock().now().to_msg(),
                                      self.pin_agents)

        if self.goal_pos is not None and self.pin_agents is not None:
            
            all_positions = np.vstack([self.pin_agents,self.goal_pos])
            edges = get_edges(all_positions,self.comm_radius)
            
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
