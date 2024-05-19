## TODO

### Init

**connectivity_control:**

```bash
ros2 run swarm_connect connectivity_control /home/anton-superior/ros2_ws/src/swarm_connect/cfg/cfg1.cfg
```

Todo:

- Ensure that path planning via A* makes sense
- Make the trajectory tracker better (use trajectory upload options)
- Allow charging stations (?)

May 18, 1.22 -> At some point, we must stop the development of the thesis and start the documentation...

May 18, 22.11 -> What did I mean by 'stop the development'?




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

## Versions

### Basic 1

basic connectivity maintainence. 

`ros2 run swarm_connect_ros basic1 /home/anton-superior/ros2_ws/src/swarm_connect_ros/cfg/cros_basic1.cfg`

### Basic 2

Connectivity mainanence with fleet management. Uses out of box go-to function for trajectory tracking. Highly unstable! Overshoots!

`ros2 run swarm_connect_ros basic2 /home/anton-superior/ros2_ws/src/swarm_connect_ros/cfg/cros_basic2.cfg`

updates:

- source code for 3D astar: https://github.com/balzer82/3D-OccupancyGrid-Python/blob/master/3D-AStar-PathPlanning.py

    - modified for 26 dims
    
    - modifued for eucledian cost

### Basic 3

Create custom go_to_pt controller using velocities command!! Tune for PID if need be!

`ros2 run swarm_connect_ros basic2 /home/anton-superior/ros2_ws/src/swarm_connect_ros/cfg/cros_basic2.cfg`

--------------------------------------------------------------------------

### Batt 1

direct go, no trajectory shit!

`ros2 run swarm_connect_ros batt1 /home/anton-superior/ros2_ws/src/swarm_connect_ros/cfg/batt1.cfg`

### Batt 2

Update with UAV states and inactive UAV list and recharging

`ros2 run swarm_connect_ros batt2 /home/anton-superior/ros2_ws/src/swarm_connect_ros/cfg/batt2.cfg`

### Traj 1

Add trjectory planning now! Add states

`ros2 run swarm_connect_ros traj1 /home/anton-superior/ros2_ws/src/swarm_connect_ros/cfg/traj1.cfg`

--------------------------------------------------------------------------

### Cons1

Allows consensus based initial takeoff avoiding the need for initial configuration 

```bash
ros2 run swarm_connect_ros cons1 /home/anton-superior/ros2_ws/src/swarm_connect_ros/cfg/cons1.cfg
```