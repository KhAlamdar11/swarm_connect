import sys
import logging 
import numpy as np
import rowan

from crazyflie_py import *
import rclpy 
import rclpy.node

from rclpy.clock import Clock, ClockType
from builtin_interfaces.msg import Duration

from geometry_msgs.msg import Quaternion, Point, Vector3, Twist

from crazyflie_interfaces.msg import LogDataGeneric, AttitudeSetpoint, Hover
from crazyflie_interfaces.srv import Takeoff, GoTo, Land

import pathlib

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty

from time import time
from rclpy import executors
from rclpy.qos import qos_profile_sensor_data

import argparse

from ament_index_python.packages import get_package_share_directory

import tf_transformations
from geometry_msgs.msg import Twist


from enum import Enum
from copy import copy
import time
from collections import deque
from threading import Thread

class Test1(rclpy.node.Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        
        self.rate = 5

        self.position = []
        self.velocity = []
        self.attitude = []

        self.trajectory_changed = True

        self.flight_mode = 'idle'

        self.get_logger().info('Initialization completed...')

        self.is_flying = False

        # init flags
        self.pose_received = False
        self.pose = None
        self.goal1 = [0.5,0.0,0.15]
        self.goal2 = [0.5,-0.25,0.15]
        self.goal3 = [0.25,-0.25,0.15]
        self.goal4 = [0.25,0.0,0.15]

        self.is_goal1 = False
        self.is_goal2 = False
        self.is_goal3 = False
        self.is_goal4 = False

        self.is_init = False
        self.is_takeoff = False
        self.is_land = False

        # Time
        self.clock = Clock(clock_type=ClockType.ROS_TIME)
        self.start_time = self.clock.now()  # Record the start time when the node is initialized
        
        #_________________________  Subs  _________________________
        # self.create_subscription(PoseStamped,'cf_1/pose',self._pose_msg_callback,10)
        # self.create_subscription(LogDataGeneric,'cf_1/velocity',self._velocity_msg_callback,10)

        self.takeoff_client = self.create_client(Takeoff, '/cf1/takeoff')

        self.land_client = self.create_client(Land, f'/cf1/land')
        
        self.publisher_ = self.create_publisher(Hover, '/cf1/cmd_hover', 10)
        # self.publisher_ = self.create_publisher(Twist, '/cf1/cmd_vel_legacy', 10)

        self.pose_list = []

        self.subscription = self.create_subscription(PoseStamped,'/cf1/pose', self.pose_cb,10)

        self.create_subscription(PoseStamped,'/true_cf1_pose', self.true_pose_cb,10)
        
        self.go_to_client = self.create_client(GoTo, f'/cf1/go_to')

        self.run_c = self.create_timer(1/self.rate, self.run)

    def run(self):
        print(self.check_time())
        if not(self.is_takeoff) and self.check_time() > 4:
            print('Taking off')
            self.is_takeoff=True
            self.takeoff(0.15)

        if not(self.is_goal1) and self.is_takeoff and self.check_time() > 6:
            self.is_goal1 = True     
            self.go_to(self.goal1)

        if not(self.is_goal2) and self.is_goal1 and self.check_time() > 12:
            self.is_goal2 = True 
            self.go_to(self.goal2)

        if not(self.is_goal3) and self.is_goal2 and self.check_time() > 14:
            self.is_goal3 = True 
            self.go_to(self.goal3)

        if not(self.is_goal4) and self.goal3 and self.check_time() > 18:
            self.is_goal4 = True 
            self.go_to(self.goal4)           

        if not(self.is_land) and self.check_time() > 24:
            print('Landing')
            self.is_land=True
            self.land()


    def go_to(self, goal, mode = None):
    
        if self.pose is not None:
            # current_distance = np.linalg.norm(self.pose[:3] - pos)

            # t = current_distance / self.speed
            # t_s, t_ns = self.get_sec_nanosec(t)
            
            while not self.go_to_client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info(f'Goto for {self.name} not available, waiting...') if self.is_log_status else None
            request = GoTo.Request(group_mask=0,
                                relative=False,
                                goal=Point(x=goal[0], y=goal[1], z=goal[2]),
                                yaw=self.pose[-1],
                                duration=Duration(sec=2, nanosec=0))
            
            future = self.go_to_client.call_async(request)

    def check_time(self):
        elapsed_time = self.clock.now() - self.start_time
        elapsed_seconds = elapsed_time.nanoseconds / 1e9
        return elapsed_seconds
    
    def takeoff(self,altitude):
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Takeoff not available, waiting...')
        future = self.takeoff_client.call_async(Takeoff.Request(height=altitude))


    def land(self):
        while not self.land_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Land not available, waiting...')
        future = self.land_client.call_async(Land.Request())

    def true_pose_cb(self,msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])

        self.get_logger().info(f"Position: ({x}, {y}, {z})")
        # self.get_logger().info(f"Orientation in RPY: (Roll: {roll}, Pitch: {pitch}, Yaw: {yaw})")


    def pose_cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])

        # Print the extracted values
        # self.get_logger().info(f"Position: ({x}, {y}, {z})")
        # self.get_logger().info(f"Orientation in RPY: (Roll: {roll}, Pitch: {pitch}, Yaw: {yaw})")

        self.pose = [x,y,z,roll,pitch,yaw]
            
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-n","--n_agents",help ="Number of agents",default=1,type=int)
    args = parser.parse_args()
    N_AGENTS = args.n_agents

    rclpy.init()

    node = Test1("Test1")

    rclpy.spin(node)    
        
    node.get_logger().info('Keyboard interrupt, shutting down.\n')

    rclpy.shutdown()

if __name__ == '__main__':
   main()
