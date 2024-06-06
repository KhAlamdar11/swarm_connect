# Hardware Migration

Docker container is used to connect the crazyflies and initiate the client.

**Basic Run Setup**

```bash
# Start the bebop_cont:
docker start -i crazyswarm2_cont2

# Stop the conatainer
docker stop crazyswarm2_cont2

# connect another terminal to docker
docker exec -it crazyswarm2_cont2 bash
```

When inside docker you can use cflib to connect and use to find out the URIs

```bash
cfclient
```

Crazyswarm2 client can be launched via the following command. The crazyflie must however know its own position for it to work!

```bash
ros2 launch crazyflie launch.py
```

In case of not having localization available and just for testing, you can use the following to connect to the crazyflie and initiate useful topics. first checkout keyboard_velmux_launch.py and make sure that the ‘robot_prefix’ of vel_mux matches your crazyflie ID in crazyfies.yaml (‘cf231’). You dont need to do this if youre only using this for launching the client!

```bash
ros2 launch crazyflie_examples keyboard_velmux_launch.py
```

```bash
ros2 launch motion_capture_tracking launch.py
```

:warning: **Operating w/o Localization!**

After successive takeoffs and landings, the stability of the UAV worsens. It is recommended to relaunch the application after each takeoff! Ideally, use the localization system!

To connect to correct cfs and/or changing the number of cfs, go to dir `/home/developer/ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml` and modify it with required params.




**One time**

```bash
# Build a Dockerfile
docker build -t crazyswarm2_img2 . 

# Run the crazyswarm2_img2 container for the fist time
./first_run.sh

# This will create docker container crazyswarm_container and position you into the container

# Delete the container
docker rm crazyswarm2_cont2
```