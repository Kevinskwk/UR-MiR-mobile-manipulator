# CS5478 Project - VLM Mobile Manipulation

Kevin Ma, Oscar Jiang

## Setting up

### Set up ROS Gazebo Environment

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

### Set up GPT

```bash
pip install openai
```

### Set up RoboPoint

Follow the instructions from [RoboPoint](https://github.com/wentaoyuan/RoboPoint) repo to set up RoboPoint.

> Note: Limited by computation resources, RoboPoint and Gazebo are deployed in separated devices. Instructions from RoboPoint are manually carried out in the ROS side through the python robot interface. FAST API can be used for communication between ROS and RoboPoint across devices or to resolve conflict between ROS and Conda environment.

## Running

### Launching Robot

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

### Task Planning

Add your OpenAI API key at `openai.api_key`, and Modify the `task_description` and `image_path` in [gpt_task_planning.py](gpt_task_planning.py)

And then run:

```bash
python gpt_task_planning.py
```

### Using RoboPoint

Follow the instructions in [robopoint.ipynb](robopoint.ipynb) to specify input image file and querry. Run through all cells for the output.

The output of RoboPoint can then be entered in the python robot interface.

## Acknowledgement

This repo is based on [UR-MiR-mobile-manipulator](https://github.com/Spartan-Velanjeri/UR-MiR-mobile-manipulator)

Borrowing packages from:

- [IFRA_LinkAttacher](https://github.com/IFRA-Cranfield/IFRA_LinkAttacher) for vacuum gripper link attacher
- [gazebo_models_worlds_collection](https://github.com/leonhartyao/gazebo_models_worlds_collection) for office world and gazebo assets

VLMs: GPT-4o, [RoboPoint](https://github.com/wentaoyuan/RoboPoint)