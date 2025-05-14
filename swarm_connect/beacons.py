import numpy as np
import math
import argparse
import configparser
from os import path
import os
import copy

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

        self.uav_ns = "cf_6"
        self.explore_robot = UAV(node=self, name=self.uav_ns, rate=0.1, battery_decay_rate=0.0, battery_recharge_rate=0.0)
        self.explore_robot.speed = 0.12
        self.explore_robot.takeoff(altitude=0.5)

        self.uav_ns = "cf_7"
        self.explore_uav = UAV(node=self, name=self.uav_ns, rate=0.1, battery_decay_rate=0.0, battery_recharge_rate=0.0)
        self.explore_uav.speed = 0.12
        self.explore_uav.takeoff(altitude=2.0)

        self.uav_ns = "cf_8"
        self.base_uav = UAV(node=self, name=self.uav_ns, rate=0.1, battery_decay_rate=0.0, battery_recharge_rate=0.0)
        self.base_uav.speed = 0.12
        self.base_uav.takeoff(altitude=2.0)


        # self.follow_trajectory()
        self.goals = np.array([[0,4,0.5],[3.5,4,0.5],[3.5,0,0.5],[8.0,0,0.5],[8.0,7.5,0.5],[0,7.5,0.5]])
        self.goal_idx = 0

        self.path_completed = False

        self.init_form = self.create_timer(0.5, self.run_)

    def run_(self):
        if self.explore_robot.position is not None and self.explore_uav.position is not None and self.goals is not None:        
            if self.explore_robot.is_takeoff_completed() and self.explore_uav.is_takeoff_completed():
                if self.explore_robot.is_reached() and self.explore_uav.is_reached(): 
                    
                    print(f"goal is {self.goals[self.goal_idx]}")

                    goal = copy.deepcopy(self.goals[self.goal_idx])

                    self.explore_robot.go_to(goal=goal)

                    goal[2] = 2.0
                    self.explore_uav.go_to(goal=goal)

                    self.base_uav.go_to(goal=np.array([0.0,0.0,2.0]))

                    self.goal_idx+=1
                    if self.goal_idx == self.goals.shape[0]:
                        if self.path_completed:
                            self.goals = None
                        else:
                            self.goals = self.goals[::-1]
                            self.goal_idx=0
                            self.path_completed = True


def main():

    rclpy.init()

    node = Cons1("beacons")

    rclpy.spin(node)

    node.get_logger().info('Keyboard interrupt, shutting down.\n')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
