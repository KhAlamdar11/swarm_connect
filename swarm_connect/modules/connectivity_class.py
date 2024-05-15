import numpy as np
import configparser
from os import path
import matplotlib.pyplot as plt
from matplotlib.pyplot import gca
import json
from matplotlib.colors import Normalize
import math
import random

from swarm_connect_ros.utils.connectivity_controller import ConnectivityController
from swarm_connect_ros.utils.lattice import *


font = {'family': 'sans-serif',
        'weight': 'bold',
        'size': 14}


class ConnectivityClass():

    def __init__(self):

        self.mean_pooling = True  # normalize the adjacency matrix by the number of neighbors or not

        # number states per agent
        self.nx_system = 4
        # number of observations per agent
        self.n_features = 8
        # number of actions per agent
        self.nu = 2 

        # default problem parameters
        self.n_agents = 10  # int(config['network_size'])
        self.comm_radius = 2.5  # float(config['comm_radius'])
        self.dt = 0.01  # #float(config['system_dt'])
        self.v_max = 3.0  #  float(config['max_vel_init'])

        # intitialize state matrices
        self.x = None
        self.u = None
        self.mean_vel = None
        self.init_vel = None

        # pin specific params
        self.robot_speed = 1
        self.pin_speed_vec = [0.0,0.0]
        self.robot_init_dist = 5.0
        self.robot_speed = 1 

        # UAV params
        self.battery = None
        self.in_motion = np.zeros(self.n_agents-2, dtype=bool)
        self.uav_speed = 0.85

        self.max_accel = 1
      
        self.fig = None
        self.points = None
        self.points2 = None
        # self.line_objects =

        # pinned node positions
        self.start_node = np.array([0.0,0.0])
        self.end_node = None
        self.path_xy = None
        self.t = 0
        self.robot_trajectory_argument = None
        self.robot_trajectory = None

        # controller params
        self.controller_params = None

        self.connectivity_controller = None
        
        self.fiedler_value = None
        self.fiedler_vector = None
        self.fiedler_value_list = []

        self.path_plot = None

        self.seed()


    def params_from_cfg(self, args, rate):

        self.n_agents = args.getint('n_init_agents') + 2

        self.comm_radius = args.getfloat('comm_radius')

        # UAV control params
        self.uav_speed = args.getfloat('uav_speed')
        self.uav_v_max = args.getfloat('uav_v_max')

        # controller
        delta = args.getfloat('delta')
        sigma = math.sqrt(-self.comm_radius/(2*math.log(delta)))
        self.controller_params = {'sigma':sigma,
                                  'range':self.comm_radius,
                                  'normalized': args.getint('normalized'),
                                  'epsilon': args.getfloat('epsilon'),
                                  'gainConnectivity': args.getfloat('gainConnectivity'),
                                  'gainRepel': args.getfloat('gainRepel'),
                                  'repelThreshold': self.comm_radius*args.getfloat('repelThreshold'),
                                  'unweighted': args.getint('unweighted'),
                                  'v_max': args.getfloat('uav_v_max')}

        # states
        # number of observations per agent
        self.n_features = args.getint('n_states')
        # number of actions per agent
        self.nu = args.getint('n_actions')
        self.dt = rate


        # mobile robot params!
        self.robot_speed = args.getfloat('robot_speed')
        self.robot_init_dist = args.getfloat('robot_init_dist')
        self.robot_trajectory_argument = args.get('robot_trajectory')
        self.end_node = np.array(json.loads(args.get('end_node_init')), dtype=float)
        self.start_node = np.array(json.loads(args.get('start_node_init')), dtype=float)

        # UAV addition 
        self.add_uav_limit = np.array(json.loads(args.get('add_uav_limit')), dtype=float)
        self.add_uav_criterion = args.get('add_uav_criterion')

        # viz params
        self.plot_lim = np.array(json.loads(args.get('plot_lim')), dtype=float)



    def reset(self,is_gen_lattice=True,i=0):

        self.t = 0

        x = np.zeros((self.n_agents, self.nx_system))
        # self.battery = np.ones(self.n_agents)
       
        # set pinned nodes
        # th = np.radians(np.random.uniform(0, 360))
        
        if self.robot_trajectory_argument == 'circle':
            self.robot_trajectory = 'circle'
        elif self.robot_trajectory_argument == 'elipse':
            self.robot_trajectory = 'elipse'
        elif self.robot_trajectory_argument == 'line':
            self.robot_trajectory = 'line'
        elif self.robot_trajectory_argument == 'random':
            # chose randomly between circle, elipse and line!
            self.robot_trajectory =random.choice(['circle','elipse'])
        elif self.robot_trajectory_argument == 'do_not_move':
            self.robot_trajectory = 'do_not_move'

        _, _, self.end_node = self.get_curve()
        
        vector_se = self.end_node - self.start_node
        magnitude_se = np.linalg.norm(vector_se)
        self.pin_speed_vec = self.robot_speed * vector_se / magnitude_se

        # keep good initialization
        self.mean_vel = np.mean(x[:, 2:4], axis=0)
        self.init_vel = x[:, 2:4]
        self.x = x

        # print(self.end_node)

        # self.update_pin_nodes(u2=self.pin_speed_vec)

        self.x[0,:2] = self.start_node
        self.x[1,:2] = self.end_node

        # create controller
        self.connectivity_controller = ConnectivityController(self.controller_params)

        self.controller()

        # return None

        # set initial topo
        if is_gen_lattice:
            p = gen_lattice(self.n_agents-2, self.comm_radius*0.8, self.start_node, self.end_node)
            p = np.vstack((self.start_node, self.end_node, p))
            for i, (x, y) in enumerate(p):
                self.x[i, 0] = x
                self.x[i, 1] = y

        # update state network
        self.update_state_network()

        return self.x[2:,:2]


    def step(self, v):

        '''
        takes desired velocity command of each node and adjusts
        '''

        assert v.shape == (self.n_agents, self.nu)
       
        v =  self.uav_speed * v
        
        # update velocities
        # x velocity
        self.x[:, 2] = v[:, 0]
        # y velocity
        self.x[:, 3] = v[:, 1]

        # x position
        self.x[:, 0] = self.x[:, 0] + self.x[:, 2] * self.dt 
        # y position
        self.x[:, 1] = self.x[:, 1] + self.x[:, 3] * self.dt 

        # self.update_pin_nodes(u2=self.pin_speed_vec)
        # self.end_node = np.array(self.path_xy[self.t])


        self.x[0,:2] = self.start_node
        self.x[1,:2] = self.end_node

        # print(self.end_node)

        # self.compute_helpers()

        # print(self.fiedler_value)
        done = False #True if self.fiedler_value<=0.1 or self.t > len(self.path_xy) - 2 else False

        # print('----------------')
        # print(f'distance: {distance(self.start_node,self.end_node)}')
        # print(f'max d: {(self.n_agents - 3)*self.comm_radius}')

        # return (None, None), self.instant_cost(), done, {}

        # self.decrease_battery()
        # update adj matrix
        self.update_state_network()

        return self.get_positions(), done


    #____________________  Controller  ________________________


    def controller(self):
        return self.connectivity_controller(self.get_positions())


    #____________________  Utils  ________________________


    def kill_node_i(self,i):
        '''
        kills node i
        TODO: Modify for allowing for multiple pinned nodes
        '''
    
        self.n_agents -= 1
    
        self.x = np.delete(self.x, i+2, axis=0)
        # self.in_motion = np.delete(self.in_motion, i, axis=0)

    def update_pin_nodes(self, u1=[0,0], u2=[0,0]):

        # if self.t < len(self.path_xy)-1:
        #     self.t+=1
        # else:
        #     self.t = 0
        self.t +=1

        self.start_node += np.array(u1)
        self.end_node += np.array(u2*self.dt*self.robot_speed)
        # self.end_node = np.array(self.path_xy[self.t])

    def get_curve(self):

        t = np.linspace(0, 2 * np.pi, int(1000/self.robot_speed))
        R = 2*self.robot_init_dist/2
        r = 2*self.robot_init_dist

        if self.robot_trajectory == 'circle':
            x, y = R*np.cos(t), R*np.sin(t)
        elif self.robot_trajectory == 'elipse':
            x, y = R*np.cos(t), r * np.sin(t)
        elif self.robot_trajectory == 'do_not_move':
            x, y = R*np.ones_like(t), np.zeros_like(t) 
            

        self.path_xy = np.array([[x, y] for x, y in zip(x, y)])

        if self.robot_trajectory == 'circle' or self.robot_trajectory == 'elipse':
            self.flip_shift()

        return x, y, self.path_xy[0]
    

    #____________________  Add agents  _____________________


    def add_agent_i(self,agent,n):
        t = np.linspace(0, 2 * np.pi, 9)   
        # print(t) 
        a, b = 0.4*self.comm_radius*np.cos(t), 0.4*self.comm_radius*np.sin(t)
        a += self.x[n,0]
        b += self.x[n,1]
        arc = np.array([[a, b] for a, b in zip(a, b)])

        # shortlist pts
        possible_pts = []
        possible_connections = []
        for j in range(arc.shape[0]):
            allow = True
            cons = 0
            for i in range(self.n_agents):
                if i != n and distance(arc[j],self.x[i]) < self.comm_radius*0.3:
                    allow = False
                    break
                elif i != n and i!=agent and distance(arc[j],self.x[i]) < self.comm_radius :
                    cons += 1
            if allow:
                possible_pts.append(arc[j])
                possible_connections.append(cons)

        return np.array(possible_pts), np.array(possible_connections)


    def add_agent(self,agent):
        neighbors = np.nonzero(self.state_network[agent, :])[0]
        arc = np.array([])
        poss = np.array([])
        for neigh in neighbors:
            a, pos = self.add_agent_i(agent,neigh)
            arc = svstack([arc, a])
            poss = np.concatenate([poss,pos])
        m = np.argmax(poss)
        
        new_agent = np.array([arc[m,0],arc[m,1],0,0])

        self.x = np.vstack([self.x,new_agent])
        self.n_agents += 1
        self.battery = np.append(self.battery, 1.0)
        self.in_motion = np.append(self.in_motion, False)


    def seed(self, seed=None):
        return [seed]
    
    def relu(self,a):
        return a * (a>0)


    #____________________  Features  ________________________

    def update_state_network(self):
        self.diff = self.x.reshape((self.n_agents, 1, self.nx_system)) - self.x.reshape((1, self.n_agents, self.nx_system))
        self.r2 =  np.multiply(self.diff[:, :, 0], self.diff[:, :, 0]) + np.multiply(self.diff[:, :, 1], self.diff[:, :, 1])
        self.adj_mat = (self.r2 < self.comm_radius).astype(float)
        self.state_network = self.adj_mat
    

    #____________________  Getters  ________________________

    def get_n_agents(self):
        return self.n_agents
    
    def get_positions(self):
        return self.x[:,:2]
    
    def get_adj_mat(self):
        return self.state_network

    def get_params(self):
        return self.n_agents, self.comm_radius, self.start_node, self.end_node

    def get_fiedler_list(self):
        return self.fiedler_value_list
    
    def get_fiedler(self):
        return self.connectivity_controller.get_fiedler()[0]
    
    def get_pin_nodes(self):
        return self.x[:2,:2]

    
    #____________________  Setters  ________________________

    def set_battery(self,i,batt_level):
        '''
        sets battery of the ith node to a certain value
        '''
        self.battery[i] = batt_level


    def set_all_batteries(self,arr):
        '''
        sets the battery levels of all uavs
        be careful that battery of pinned nodes is kept const 1
        '''
        arr = np.array(arr)
        assert arr.shape == (self.n_agents-2,) 
        self.battery = arr

        # print(self.battery)


    def set_batteries_random(self,lower_bound,n_lower_bound):
        '''
        sets the battery levels of all uavs
        be careful that battery of pinned nodes is kept const 1
        '''
        pass

    def set_positions(self,p):
        for pos in range(p.shape[0]):
            self.x[pos,0] = p[pos,0]
            self.x[pos,1] = p[pos,1]

        self.start_node = p[0]
        self.end_node = p[1]

        vector_se = self.end_node - self.start_node
        magnitude_se = np.linalg.norm(vector_se)
        self.pin_speed_vec = self.robot_speed * vector_se / magnitude_se

    def set_positions_uav(self,p):
        # print(p)
        for pos in range(p.shape[0]):
            self.x[pos+2,0] = p[pos,0]
            self.x[pos+2,1] = p[pos,1]


    def set_pin_speed(self,speed):
        self.robot_speed *= speed

    def set_speed(self,speed):
        self.uav_speed = speed

    def set_params(self, n_agents, comm_radius, start_node, end_node):
        self.n_agents = n_agents 
        self.comm_radius = comm_radius


