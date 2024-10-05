# swarm_connect

This repository contains part of material developed for Master's thesis in [<a href="#ref1">1</a>].

The package contains the ROS 2 for battery aware connectivity maintainance algorithm tested on Crazyflie UAVs. It implements connectivity maintainance algorihtm from [<a href="#ref2">2</a>] and battery aware connectivity maintainance of multi agent systems from [<a href="#ref2">1</a>]. This package extends the work of [gym_connect](https://github.com/KhAlamdar11/gym-connect) to Crazyflie UAVs. For theoretical and technical details please consider reading [<a href="#ref2">1</a>].

## Setup

1. **Install CrazySim**

    Follow the instructions [here](https://github.com/gtfactslab/CrazySim) to install Crazysim, which is a software-in-the-loop (SITL) simulator for the Crazyflie UAVs.

2. **Install dependencies**

    Install dependencies via the requirements file.

    ```bash
    pip install -r /path/to/requirements.txt
    ```


3. **Setup the package:**

    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/KhAlamdar11/swarm_connect.git
    cd .. && colcon build --symlink-install
    source install/setup.bash
    ```

## How to run

### 1. Run Crazysim

Start the CrazySim simulator by specifying the number of UAVs (cfs).

```bash
bash ~/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh -n 7 -m crazyflie
```

The associated files can be modified to spawn the cfs with different spacing or formations.

Next, navigate to the ```crazyflies.yaml``` file in the ```~/ros2_ws/src/crazyswarm2/crazyflie/config``` directory. Uncomment the number of UAVs that you have spawned in the first step.

⚠️ In case of mismatch in number of UAVs spawned and the number of UAVs defined in crazyflie.yaml, the server may fail to initialize.

Then run:

```bash
ros2 launch crazyflie launch.py backend:=cflib
```

⚠️ After running the simulator, give a few seconds before you run the crazyflie server. If all crazyflies do not connect, close other processing on your computer and start again, or reduce the number of Crazyflies (large number of UAVs may result in reduction in Gazebo real time factor causing async issues with the server). This can also result in a scenerio where sometimes new UAVs fail to take off.

### 2. Run the connectivity controller

Use the configuration files in /cfg directory to initialize the controller.

```bash
ros2 run swarm_connect connectivity_control /home/anton-superior/ros2_ws/src/swarm_connect/cfg/cfg1_battery.cfg
```

The virtual nodes can be controlled by keyboard commands:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Hardware migration

The exact same scripts are used with Crazyflies on hardware as well. Be careful about the namespaces of Crazyflies as on hardware they may look like ```cf1/``` instead of ```cf_1/```.