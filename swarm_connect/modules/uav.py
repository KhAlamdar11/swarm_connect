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

from uav_trajectory import generate_trajectory

'''
Base class for UAV related data handling and processing.
TODO: Refine the logic for cleaner code.
'''

class UAV:
    def __init__(self, node, name, rate, battery_decay_rate, battery_recharge_rate, battery=1.0):

        self.node = node
        self.name = name

        self.speed = 0.07

        # defines if the uav is ready for go to or requires to follow its current traj first!
        self.mode = 'land'
        self.prev_mode = 'go_to'
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

        self.is_takeoff_completed_var = False

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

    def takeoff(self,altitude,mode_change=True):

        # t = abs(self.position[2] - altitude) / self.speed
        t = 5.0
        t_s, t_ns = self.get_sec_nanosec(t)
        
        self.node.get_logger().info(f'Takeoff for uav {self.name} called...') if self.is_log_status else None
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Takeoff for {self.name} not available, waiting...') if self.is_log_status else None
        
        for i in range(20):
            future = self.takeoff_client.call_async(Takeoff.Request(height=altitude,
                                                                    duration=Duration(sec=t_s, nanosec=t_ns)))

        self.takeoff_alt = altitude
        if mode_change:
            self.mode = 'go_to'


    def land(self, duration):

        self.node.get_logger().info(f'Land for uav {self.name} called...') if self.is_log_status else None
        while not self.land_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(f'Land for {self.name} not available, waiting...') if self.is_log_status else None
        for i in range(10):
            future = self.land_client.call_async(Land.Request(duration=Duration(sec=duration_sec, 
                                                                               nanosec=duration_nanosec)))


    def go_to(self, goal, mode = None):
        
        if self.mode == 'go_to':
            pos = copy.deepcopy(goal)
            # only proceed if new pt is further form last point to avoid repetitive commands!
            if self.goal is not None:
                if np.linalg.norm(pos - self.goal) < 0.05:  # Define some acceptable distance within which new commands are suppressed
                    return  # Skip sending new command if close enough
            self.goal = pos

        elif self.mode == 'trajectory':
            pos = None
            if len(self.trajectory) > 1:
                if np.linalg.norm(self.trajectory[0] - self.position) < 0.1 or self.prev_mode=='go_to':
                    pos = self.trajectory[0]
                    self.trajectory = self.trajectory[1:]
                    self.prev_mode = 'trajectory'
            else:
                self.mode = 'go_to'
                self.trajectory = []

        if pos is not None:
            current_distance = np.linalg.norm(self.position - pos)

            if current_distance<3.0:
                t = current_distance / self.speed
            else:            
                t = current_distance / 0.05

            t_s, t_ns = self.get_sec_nanosec(t)

            # print('---------------------')
            # print(f'Distance: {current_distance}, Speed: {self.speed}, Time: {t_s},{t_ns}')

            while not self.go_to_client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info(f'Goto for {self.name} not available, waiting...') if self.is_log_status else None
            request = GoTo.Request(group_mask=0,
                                relative=False,
                                goal=Point(x=pos[0], y=pos[1], z=pos[2]),
                                yaw=self.attitude[2],
                                duration=Duration(sec=t_s, nanosec=t_ns))
            
            future = self.go_to_client.call_async(request)

    def go_to_land(self):

        # TODO: Remove this redundant check!
        
        if self.mode == 'landing':
            pos = None
            if len(self.trajectory) > 1:
                if np.linalg.norm(self.trajectory[0] - self.position) < 0.1:
                    pos = self.trajectory[0]
                    self.trajectory = self.trajectory[1:]
            else:
                self.mode = 'land'
                self.trajectory = []
                self.land()

        if pos is not None:
            current_distance = np.linalg.norm(self.position - pos)

            t = current_distance / self.speed
            t_s, t_ns = self.get_sec_nanosec(t)
            
            print('---------------------')
            print(f'Distance: {current_distance}, Speed: {self.speed}, Time: {t_s},{t_ns}')

            while not self.go_to_client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info(f'Goto for {self.name} not available, waiting...') if self.is_log_status else None
            request = GoTo.Request(group_mask=0,
                                relative=False,
                                goal=Point(x=pos[0], y=pos[1], z=pos[2]),
                                yaw=self.attitude[2],
                                duration=Duration(sec=t_s, nanosec=t_ns))
            
            future = self.go_to_client.call_async(request)


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

        # if self.goal is None:
        #     self.goal = self.position

    #_________________________  Setters/Getters  _________________________

    def set_trajectory(self,traj,mode='trajectory'):
        self.trajectory = traj
        self.mode = mode
        # print(self.mode)

    def set_mode(self,mode):
        self.mode = mode

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
        """Converts seconds in float to seconds(int) and nanseconds(int) for the duration based services"""
        seconds = int(math.floor(sec))
        fractional_part = sec - seconds
        nanoseconds = int(fractional_part * 1e9)  # Convert fractional seconds to nanoseconds
        return seconds, nanoseconds

    def is_reached(self):
        if self.goal is None:
            # print("Is reached: Goal is none")
            return True
        if np.linalg.norm(self.position - self.goal) > 0.1:  
            # print(f"Is reached: Position is far from goal {self.position}, {self.goal}")
            return False
        # print(f"Is reached: Position reached {np.linalg.norm(self.position - self.goal)}")
        return True

    def is_takeoff_completed(self):
        # print(f"Is takeoff completed: {self.position[2]}, {self.takeoff_alt}")
        if self.is_takeoff_completed_var == True:
            return True
        if abs(self.position[2] - self.takeoff_alt) > 0.1: 
            return False
        self.is_takeoff_completed_var = True
        return True 
