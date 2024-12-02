# CS5478 Project - VLM Mobile Manipulation

Kevin Ma, Oscar Jiang

## Set up

First, set up this demo environment following the [Original README](Original_README.md).

Set up extra dependencies

```bash
git clone https://github.com/leonhartyao/gazebo_models_worlds_collection.git
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<current path>/gazebo_models_worlds_collection/models
cd ros2_ws/src
git clone https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.git
```

Build packages

```bash
cd ros2_ws
colcon build
source install/setup.bash
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:<path to ros2_ws>/install/ros2_linkattacher/lib
```

## Launching

Launch the gazebo environment

```bash
ros2 launch mir_gazebo mobile_manipulator.launch.py world:=office_simplified rviz_config_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/rviz/mir_nav.rviz
```

Launch localization and navigation

```bash
ros2 launch mir_navigation mir_nav_launch.py use_sim_time:=true map:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/maps/office_simplified.yaml
```

Launch move-it & relay node

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true prefix:=ur_ use_fake_hardware:=true use_sim_time:=true
ros2 launch hello_moveit_ur hello_moveit_ur_launch.py 
```

Running python robot interface

```bash
ros2 run vlm_ros_interface robot_interface 
```
