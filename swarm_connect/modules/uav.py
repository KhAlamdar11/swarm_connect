import rclpy

from geometry_msgs.msg import Quaternion, Point, Vector3, Twist
from nav_msgs.msg import Odometry, Path
import tf_transformations
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.msg import LogDataGeneric, AttitudeSetpoint
from crazyflie_interfaces.srv import Takeoff, GoTo, Land
from crazyflie_interfaces.srv import UploadTrajectory
from crazyflie_interfaces.msg import TrajectoryPolynomialPiece
from builtin_interfaces.msg import Duration

import math

import copy

import numpy as np

# TODO: Reduce reduncancy of creation of markers every time. Create a single function for that.

'''
Base class for UAV related data handling and processing
'''

class UAV:
    def __init__(self, node, name, rate, battery_decay_rate, battery_recharge_rate, battery=1.0):

        self.node = node
        self.name = name

        self.speed = 0.2

        # defines if the uav is ready for go to or requires to follow its current traj first!
        self.mode = 'go_to'
        self.trajectory = []

        self.old_goal = np.array([1000.0,1000.0])

        self.battery = battery
        self.battery_decay_rate = battery_decay_rate
        self.battery_recharge_rate = battery_recharge_rate
        self.rate = rate

        # state variables
        self.position = None
        self.attitude = None

        self.goal = None

        self.takeoff_alt = 0.2

        self.is_log_status = False

        # subscribers
        node.create_subscription(PoseStamped, f'{self.name}/pose', lambda msg, ns=name: self._pose_msg_callback(msg),10) 

        # clients for flight handling
        self.takeoff_client = node.create_client(Takeoff, f'/{self.name}/takeoff')

        self.land_client = node.create_client(Land, f'/{self.name}/land')
        
        self.go_to_client = node.create_client(GoTo, f'/{self.name}/go_to')

    #_________________________  Features  _________________________

    def decrease_battery(self):
        self.battery -= self.battery_decay_rate/self.rate

    def recharge_battery(self):
        self.battery = min(1.0, self.battery + self.battery_recharge_rate/self.rate)

    #_________________________  Flight Handlers  _________________________

    def takeoff(self,altitude):
        self.node.get_logger().info(f'Takeoff for uav {self.name} called...') if self.is_log_status else None
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Takeoff for {self.name} not available, waiting...') if self.is_log_status else None
        future = self.takeoff_client.call_async(Takeoff.Request(height=altitude))
        self.takeoff_alt = altitude


    def land(self):
        self.node.get_logger().info(f'Land for uav {self.name} called...') if self.is_log_status else None
        while not self.land_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(f'Land for {self.name} not available, waiting...') if self.is_log_status else None
        future = self.land_client.call_async(Land.Request())


    def go_to(self, goal):

        if self.mode == 'go_to':
            pos = copy.deepcopy(goal)
            # only proceed if new pt is further form last point to avoid repetitive commands!
            if np.linalg.norm(pos - self.goal) < 0.05:  # Define some acceptable distance within which new commands are suppressed
                return  # Skip sending new command if close enough
            self.goal = pos

        elif self.mode == 'trajectory':
            # extract and delete current waypoint
            pos = self.trajectory[0]

            # only proceed if new pt is further form last point to avoid repetitive commands!
            if np.linalg.norm(pos - self.goal) < 0.05:  # Define some acceptable distance within which new commands are suppressed
                return  # Skip sending new command if close enough

            self.goal = pos
            if len(self.trajectory) > 1:
                self.trajectory = self.trajectory[1:]
            else:
                self.mode = 'go_to'
                self.trajectory = []

        current_distance = np.linalg.norm(self.position - pos)

        t = current_distance / self.speed
        t_s, t_ns = self.get_sec_nanosec(t)
        
        while not self.go_to_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(f'Goto for {self.name} not available, waiting...') if self.is_log_status else None
        request = GoTo.Request(group_mask=0,
                            relative=False,
                            goal=Point(x=pos[0], y=pos[1], z=pos[2]),
                            yaw=self.attitude[2],
                            duration=Duration(sec=t_s, nanosec=t_ns))
        
        future = self.go_to_client.call_async(request)


    # def go_to(self, goal):

    #     if self.mode == 'go_to':
    #         pos = copy.deepcopy(goal)
    #     elif self.mode == 'trajectory':
    #         # extract and delete current waypoint
    #         pos = self.trajectory[0]
    #         if len(self.trajectory) > 1:
    #             self.trajectory = self.trajectory[1:]
    #         else:
    #             self.mode = 'go_to'
    #             self.trajectory = []

    #     current_distance = np.linalg.norm(self.position - pos)
    #     if current_distance < 0.05:  # Define some acceptable distance within which new commands are suppressed
    #         return  # Skip sending new command if close enough

    #     t = current_distance / self.speed
    #     t_s, t_ns = self.get_sec_nanosec(t)
        
    #     while not self.go_to_client.wait_for_service(timeout_sec=1.0):
    #         self.node.get_logger().info(f'Goto for {self.name} not available, waiting...') if self.is_log_status else None
    #     request = GoTo.Request(group_mask=0,
    #                         relative=False,
    #                         goal=Point(x=pos[0], y=pos[1], z=pos[2]),
    #                         yaw=self.attitude[2],
    #                         duration=Duration(sec=t_s, nanosec=t_ns))
        
    #     future = self.go_to_client.call_async(request)

    # def go_to(self, goal):
    #     pos = copy.deepcopy(goal)

    #     old_new_goal_dist = np.linalg.norm(self.position-pos)

    #     if old_new_goal_dist > 0.2:
    #         self.node.get_logger().info(f'Goto for uav {self.name} called...') if self.is_log_status else None   
            
    #         # compute duration for constant speed, t = d/v
    #         # contributed by Dr. Nada
    #         t = np.linalg.norm(self.position - pos) / self.speed 
    #         t_s, t_ns = self.get_sec_nanosec(t)
            
    #         while not self.go_to_client.wait_for_service(timeout_sec=1.0):
    #             self.node.get_logger().info(f'Goto for {self.name} not available, waiting...') if self.is_log_status else None
    #         request = GoTo.Request(group_mask=0,
    #                             relative=False,
    #                             goal=Point(x=pos[0], y=pos[1], z=pos[2]),
    #                             yaw=self.attitude[2],
    #                             duration=Duration(sec=t_s, nanosec=t_ns))
            
    #         future = self.go_to_client.call_async(request)


    #_________________________  Callbacks  _________________________
    
    def _pose_msg_callback(self, msg: PoseStamped):
    
        self.position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
        orientation_q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        attitude = tf_transformations.euler_from_quaternion(orientation_q)
        # wrap yaw
        if attitude[2] > np.pi:
            attitude[2] -= 2 * np.pi
        elif attitude[2] < -np.pi:
            attitude[2] += 2 * np.pi
        self.attitude = attitude

        if self.goal is None:
            self.goal = self.position

    #_________________________  Setters/Getters  _________________________

    def set_trajectory(self,traj):
        self.trajectory = traj
        self.mode = 'trajectory'
        print(self.trajectory)

    
    def get_trajectory(self):
        return self.trajectory
    

    def get_pose(self):
        return self.position
    
    def get_name(self):
        return self.name

    def get_battery(self):
        return self.battery
    

    #_________________________  Utils  _________________________

    def get_sec_nanosec(self,sec):
        '''
        Converts seconds in float to seconds(int) and nanseconds(int) for the duration based services
        '''
        seconds = int(math.floor(sec))
        fractional_part = sec - seconds
        nanoseconds = int(fractional_part * 1e9)  # Convert fractional seconds to nanoseconds
        return seconds, nanoseconds
