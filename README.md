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

Bugs:

Msg dropping!!! have multiple takoffs and land commands<!!>