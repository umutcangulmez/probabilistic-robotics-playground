# Probabilistic Robotics Playground

This repository contains experimental implementations and simulations related to probabilistic robotics, including Gazebo simulations, sensor models.

---

## Gazebo Experiments – Node Bringup

Follow the steps below to start the Gazebo simulation environment using Docker.

```bash
cd dockerfiles
docker compose up -d
docker exec -it ros2_gazebo bash 
```


# follower-leader node bringup
Use the following steps to build and launch the follower–leader setup:

```bash
colcon build --packages-select follower_robot

source install/setup.bash

ros2 launch follower_robot leader_follower.launch.py 

```

# turtle experiments
Use the following steps to build and launch the turtle-experiments setup:

```bash
colcon build --packages-select turtle_experiments
source install/setup.bash
ros2 launch turtle_experiments bringup.launch.py
```
