colcon build --packages-select visual_inertial_nav_es_ekf

ros2 launch visual_inertial_nav_es_ekf visual_inertial_nav_es_ekf.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/vehicle_blue/cmd_vel

ign gazebo -r project_world.sdf

ros2 launch foxglove_bridge foxglove_bridge_launch.xml





colcon build --packages-select ekf_experiment
ros2 launch ekf_experiment experiment.launch.py scenarios:="['high_imu_noise']" num_runs:=1



# square movement

[//]: # (ros2 run visual_inertial_nav_es_ekf trajectory_publisher --ros-args     -p trajectory:=square -p loops:=2)

```bash 
ros2 run visual_inertial_nav_es_ekf trajectory_publisher --ros-args \
-p trajectory:=square \
-p side_length:=6.0 \
-p linear_speed:=0.3 \
-p loops:=2
```

# circular movement

```bash 
ros2 run visual_inertial_nav_es_ekf trajectory_publisher --ros-args \
-p trajectory:=circle \
-p radius:=3.0 \
-p linear_speed:=0.3 \
-p loops:=2
```

# 8-shape

```bash 
ros2 run visual_inertial_nav_es_ekf trajectory_publisher --ros-args \
-p trajectory:=figure8 \
-p scale:=4.0 \
-p linear_speed:=0.3 \
-p loops:=1
```




# Helpers

ros2 launch visual_inertial_nav_es_ekf visual_inertial_nav_es_ekf.py
ros2 run visual_inertial_nav_es_ekf data_logger --ros-args -p experiment_name:=imu_vision_10lm_square_1
ros2 run visual_inertial_nav_es_ekf trajectory_publisher --ros-args     -p trajectory:=square -p loops:=2
~/ros2_ws/src/visual_inertial_nav_es_ekf/scripts# ign gazebo -r project_world.sdf
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
ros2 run rqt_image_view rqt_image_view /visual_detector/debug_image
