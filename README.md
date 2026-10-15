# RS1 Robot for Bushland Preservation
Robot for identification and removal of invasive plant species from struggling Australian bushland regions.

## Overview
ROS 2 husky robot which navigates through a Gazebo simulation environment of bushland, identifying and removing invasive and harmful weed species to assist with environmental regeneration.

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

See [simulation setup](docs/simulation_setup.md) for dependencies, RViz,
teleoperation, spawn settings and RGB-D topics. Mission scaffold code is not
compiled or launched yet.

## Plant Detection
The pipeline receives images from the Husky camera, converts them to OpenCV format, and runs RTMDet Tiny object detection.

The demo now includes the C++ ONNX detector and an annotated-image RViz display.
See [OpenCV perception setup](docs/opencv_perception.md) for building and auto/CPU/CUDA modes.

For NVIDIA GPU inference, follow the [GPU dependency setup guide](docs/gpu_inference_setup.md).
The verified GPU overlay supports cpu, cuda and auto modes without the training venv.
