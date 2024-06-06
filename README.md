## TODO

### Init

**connectivity_control:**

```bash
bash ~/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh -n 7 -m crazyflie
```

after some time (10s)

```bash
ros2 launch crazyflie launch.py backend:=cflib
```

```bash
ros2 run swarm_connect connectivity_control /home/anton-superior/ros2_ws/src/swarm_connect/cfg/cfg1.cfg
```


**hardware_connectivity_control:**

Add updates to land uavs after a certain time and detach all controllers

Note: Remember to comment uncomment number of crazyflies in the crazyflies config file every time you change it in sim...

```bash
bash ~/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh -n 2 -m crazyflie
```

```bash
ros2 launch crazyflie launch.py backend:=cflib
```

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- cfg_2

2 UAVs, no battery decay just simple formation control, diamond formation

```bash
ros2 run swarm_connect hardware_connectivity_control /home/anton-superior/ros2_ws/src/swarm_connect/cfg/cfg2.cfg
```

- cfg_3

2 UAVs, no battery decay just simple formation control, line formation

```bash
ros2 run swarm_connect hardware_connectivity_control /home/anton-superior/ros2_ws/src/swarm_connect/cfg/cfg3.cfg
```

<!-- - cfg_4

3 UAVs, 2 in air, battery management re-enacted.

```bash
ros2 run swarm_connect hardware_connectivity_control /home/anton-superior/ros2_ws/src/swarm_connect/cfg/cfg3.cfg
``` -->



## build

```bash
colcon build --symlink-install
```

## How to run

```bash
bash ~/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh -n 10 -m crazyflie
```

after some time (10s)

```bash
ros2 launch crazyflie launch.py backend:=cflib
```

`ros2 run swarm_connect_ros basic1 /home/anton-superior/ros2_ws/src/swarm_connect_ros/cfg/cros_v12.cfg`

run keyboard:

`ros2 run teleop_twist_keyboard teleop_twist_keyboard`

The bash file for running is located in: /home/anton-superior/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/launch

Thw rold file is located in:
/home/anton-superior/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/worlds

crazysim_default

Bugs:

Msg dropping!!! have multiple takoffs and land commands<!!>