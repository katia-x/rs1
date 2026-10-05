# RS1 Robot for Bushland Preservation
Robot for identification and removal of invasive plant species from struggling Australian bushland regions.

## Overview
ROS 2 husky robot which navigates through a Gazebo simulation environment of bushland, identifying and removing invasive and harmful weed species to assist with environmental regeneration.

## Package setup
You will need to install Foxglove to use GUI

```bash
sudo apt update
sudo apt install ros-humble-foxglove-bridge
```

## Navigation
Using the Nav2 framework -Which needs to be installed into the ROS2 workspace-.

## RS1 simulation

The package launches the supplied bushland environment with Husky and Parrot
models on ROS 2 Humble / Gazebo Fortress. From this repository directory:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rs1 --symlink-install
source install/setup.bash
ros2 launch rs1 rs1_demo.launch.py
```

To run the GUI, in another window run
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml   port:=8765   use_sim_time:=true
```

See [simulation setup](docs/simulation_setup.md) for dependencies, RViz,
teleoperation, spawn settings and RGB-D topics. Mission scaffold code is not
compiled or launched yet.

## Plant Detection
The pipeline receives images from the Husky camera, converts them to OpenCV format, and runs RTMDet Tiny object detection.

### Plant Detection Installation:
Assuming you have a ROS workspace set up for this project and have cloned this RS1 repo, follow these instructions to setup the plant detection pipeline on your machine.

See the [Plant Detection documentation](src/plant_perception/PLANT_DETECTION_README.md).
