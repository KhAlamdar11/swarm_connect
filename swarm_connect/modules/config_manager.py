import json
import math
import numpy as np


def load_config_params(self, config):
    # map params
    self.map_range = np.array(json.loads(config.get('map_range')), dtype=float)
    self.map_origin = np.array(json.loads(config.get('map_origin')), dtype=float)
    self.map_res = config.getfloat('map_res')


    # UAV addition 
    self.add_uav_limit = np.array(json.loads(config.get('add_uav_limit')), dtype=float)
    self.add_uav_criterion = config.get('add_uav_criterion')

    self.rate = 6 # Hz

    # basic setup params
    self.n_agents = config.getint('n_agents')
    self.n_init_agents = config.getint('n_init_agents')
    self.comm_radius = config.getfloat('comm_radius')

    # UAV battery params
    initial_batteries_str = config.get('initial_batteries')
    if initial_batteries_str:
        try:
            self.battery = np.array(json.loads(initial_batteries_str), dtype=float)
            if self.battery.shape[0] != self.n_init_agents:
                self.get_logger().info(f"Error: initial_batteries must be an array of size {self.n_init_agents}. Using default values between 0.5 and 1.0")
                self.battery = np.linspace(0.5, 1.0, self.n_init_agents)
        except json.JSONDecodeError:
            self.get_logger().info(f"Invalid value for 'initial_batteries'. Using default values between 0.5 and 1.0")
            self.battery = np.linspace(0.5, 1.0, self.n_agents-2)
    self.battery_decay_rate = config.getfloat('battery_decay_rate')
    self.battery_decay_select = np.array(json.loads(config.get('battery_decay_select')), dtype=int)
    self.battery_recharge_rate = config.getfloat('battery_recharge_rate')
    self.critical_battery_level = config.getfloat('critical_battery_level')
    self.dead_battery_level = config.getfloat('dead_battery_level')

    self.takeoff_alt = config.getfloat('takeoff_alt')