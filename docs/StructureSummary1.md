# RS1 Project Structure

> **Status:** Work in progress  
> This document describes the repository as it currently exists. It is intended
> to be updated as the simulation, autonomy, perception, and mission logic grow.

## 1. Project purpose

RS1 is a ROS 2 package for a simulated Husky ground robot and Parrot platform
operating in an Australian bushland environment. The longer-term goal is to
identify and remove invasive plant species. The current repository contains a
working Gazebo simulation, navigation configuration, and a C++ plant-detection
pipeline. Mission orchestration and several platform/sensor classes remain
partly scaffolded.

The main target environment is:

- ROS 2 Humble
- Ubuntu 22.04
- Gazebo Fortress / `ros_ign_gazebo`
- C++17
- OpenCV DNN with an RTMDet Tiny ONNX model

## 2. Repository overview

```text
rs1/
├── CMakeLists.txt              # ROS 2 build targets, dependencies, installation
├── package.xml                 # Package metadata and ROS dependencies
├── README.md                   # Setup, launch, perception, and operator guidance
├── setup.sh                    # Repository setup helper
├── src/                        # C++ ROS nodes and processing code
├── launch/                     # Python ROS 2 launch descriptions
├── config/                     # Gazebo, bridge, robot, SLAM, Nav2, and RViz config
├── worlds/                     # Gazebo world, terrain, meshes, and plant assets
├── urdf_husky/                 # Husky robot description and meshes
├── urdf_parrot/                # Parrot robot description and meshes
├── machine_learning/models/    # Runtime ONNX model
├── test/                       # C++ unit tests
├── docs/                       # Setup, perception, GPU, GUI, and dataset documentation
├── tools/                      # GPU/OpenCV and cuDNN setup utilities
├── Environment_ML/             # Environment mesh assets
└── log/                        # Local ROS/colcon logs; not part of the runtime design
```

## 3. Build and package layer

`CMakeLists.txt` defines the `rs1` ament package and builds:

- `mission_node`
  - Sources: `src/main.cpp`, `src/mission_node.cpp`,
    `src/mission_processing.cpp`
  - Depends on `rclcpp`, `geometry_msgs`, and `sensor_msgs`
- `plant_detector`
  - Sources: `src/plant_perception/plant_detector.cpp`,
    `src/plant_perception/plant_detector_main.cpp`, and
    `src/plant_perception/rtmdet.cpp`
  - Depends on ROS image/detection messages, `cv_bridge`, and OpenCV DNN
- `mission_processing_test`
  - Built with `ament_cmake_gtest`
  - Tests the current plant-target velocity behaviour

The build installs the executables, launch files, configuration, world files,
robot descriptions, and ONNX models into the ROS package share/install space.
`package.xml` declares the ROS 2, Gazebo, navigation, SLAM, RViz, image, and
OpenCV dependencies.

## 4. C++ source structure

### 4.1 Mission and orchestration

Files:

- `src/main.cpp`
- `src/mission_node.h`
- `src/mission_node.cpp`
- `src/mission_processing.h`
- `src/mission_processing.cpp`

`main.cpp` initializes ROS 2, creates `MissionNode`, spins it, and shuts ROS
down. `MissionNode` currently publishes `geometry_msgs/msg/Twist` commands to
`/husky1/cmd_vel` through a temporary testing interface.

`MissionProcessing` subscribes to `/husky1/scan` and retains the latest laser
scan. Its `pinpointPlant` method currently accepts a 2D image bounding box and
laser scan, estimates the target bearing/range, and returns a bounded velocity
command:

1. Validate the image bounds.
2. Convert the image-centre offset to a camera bearing.
3. Match valid laser returns to the camera ray.
4. Rotate toward an off-centre target.
5. Drive forward while the target is farther than the stop distance.

This is the first control-oriented piece of the mission layer, but it is not
yet connected to plant-detector messages or a complete mission state machine.
The command publisher and several comments are explicitly temporary.

### 4.2 Platform and sensor scaffolds

The following files establish the intended separation between robot platforms,
shared processing, and sensors:

- `src/platform.h`, `src/platform.cpp`
- `src/glider/glider_node.h`, `src/glider/glider_node.cpp`
- `src/glider/glider_processing.h`, `src/glider/glider_processing.cpp`
- `src/wombat/wombat_node.h`, `src/wombat/wombat_node.cpp`
- `src/wombat/wombat_processing.h`, `src/wombat/wombat_processing.cpp`
- `src/sensing/laser.h`, `src/sensing/laser.cpp`
- `src/sensing/ranger.h`, `src/sensing/ranger.cpp`
- `src/sensing/sonar.h`, `src/sensing/sonar.cpp`

These files currently contain declarations/comments that describe planned base
classes and processing responsibilities. They are not part of the current
`CMakeLists.txt` executable targets. The repository uses the names **Wombat**
for the ground platform and **Glider** for the aerial platform in this
application layer, while the simulation assets are currently named Husky and
Parrot.

### 4.3 Plant perception

Files:

- `src/plant_perception/plant_detector.h`
- `src/plant_perception/plant_detector.cpp`
- `src/plant_perception/plant_detector_main.cpp`
- `src/plant_perception/rtmdet.h`
- `src/plant_perception/rtmdet.cpp`

`plant_detector_main.cpp` runs `PlantDetector` with a two-thread
`MultiThreadedExecutor`. `PlantDetector`:

- Subscribes to the latest camera image, normally
  `/husky1/camera/image`.
- Converts ROS images to BGR OpenCV matrices with `cv_bridge`.
- Runs inference at a configurable timer rate.
- Publishes an annotated image on `/husky1/detections/image`.
- Publishes `vision_msgs/msg/Detection2DArray` on `/husky1/detections`.
- Preserves the input timestamp and camera frame ID.
- Uses separate callback groups so image acquisition can continue while
  inference is running.

`RtmDet` is ROS-independent OpenCV inference code. It loads the ONNX model,
prepares a 640 x 640 padded/normalized input, decodes the six RTMDet outputs at
strides 8, 16, and 32, performs per-class non-maximum suppression, and returns
pixel-space detections. The configured default classes are `Weed` and `Plant`.

The selected backend can be `cpu`, `cuda`, or `auto`. In auto mode the detector
checks CUDA availability, performs a warm-up inference, and falls back to CPU
if CUDA is unavailable or fails during inference.

## 5. Simulation and robot description

### Launch files

- `launch/rs1_demo.launch.py` is the primary integrated demo launch. It starts
  Gazebo, spawns enabled robots, publishes robot state, starts localization and
  bridges, optionally starts SLAM/Nav2 and RViz, and can launch the C++ plant
  detector for the Husky.
- `launch/rs1_environmenttest.launch.py` provides an environment-focused test
  launch.
- `launch/rs1_navigation.launch.py` exposes the reusable namespace-aware SLAM
  and Nav2 launch configuration.

The demo supports Husky and Parrot enable flags, spawn poses, GUI/headless
rendering, simulation time, RViz, SLAM, Nav2, and perception/device settings.

### World and models

- `worlds/rs1_world.sdf` and `worlds/rs1_terrain.sdf` define the simulation
  world and terrain.
- `worlds/model.sdf` and `worlds/meshes/` provide the environment geometry,
  materials, textures, and plant meshes.
- `urdf_husky/` contains the Husky Xacro description and meshes.
- `urdf_parrot/` contains the Parrot Xacro/SDF description, meshes, and
  thumbnails.
- `Environment_ML/` contains additional environment mesh assets.

The launch configuration adds the package share directory to Gazebo resource
paths so the world and model URIs resolve after installation.

## 6. Configuration

The `config/` directory separates runtime configuration from source code:

- `ignition_server.config`: Gazebo server physics, scene, and sensor setup.
- `gazebo_bridge_*.yaml`: ROS/Gazebo topic bridge definitions.
- `robot_localization_*.yaml`: robot EKF/localization parameters.
- `slam_params_*.yaml`: SLAM Toolbox parameters.
- `nav2_params_*.yaml`: Nav2 parameters.
- `rs1_*.rviz`: RViz layouts for the simulated robots.
- `rs1_demo.config`: Gazebo model metadata/configuration.

The supplied documentation describes camera RGB, depth, camera-info,
point-cloud, odometry, TF, and simulation-clock topics. The default camera
stream is 720 x 480 and is used by the plant detector.

## 7. Tests

`test/basic_tests.cpp` currently covers `MissionProcessing::pinpointPlant` for:

- A centred target that produces forward motion.
- A target to the left that produces a positive turn.
- A target to the right that produces a negative turn.
- An invalid bounding box that produces a zero command.

The perception documentation also records integration verification for the
detector against reference images and demo output. These checks depend on a
ROS 2 environment and the required model/reference assets.

## 8. Documentation and operational tooling

The `docs/` directory currently includes:

- `simulation_setup.md`: ROS/Gazebo dependencies, launch options, topics,
  world layout, and teleoperation.
- `opencv_perception.md`: detector build, runtime parameters, topics, decoder
  details, and verification notes.
- `gpu_inference_setup.md`: optional CUDA/cuDNN-enabled OpenCV overlay.
- `dataset_preparation.md`: ROS bag frame extraction and annotation preparation.
- `gui-implementation-plan.md` and `gui-structure.md`: GUI planning material.
- `handy_commands.md`: common commands.
- `layout1.json`, `layout2.json`, `layout3.json`: Foxglove layouts.

The `tools/` directory contains scripts for installing local cuDNN assets and
building/activating the optional OpenCV CUDA overlay. Runtime inference does
not require the training environment; the committed ONNX model is the runtime
artifact.

## 9. Current implementation boundaries

Implemented or substantially wired:

- Gazebo bushland environment and robot spawning.
- Husky/Parrot robot descriptions and sensor topics.
- Optional localization, SLAM, Nav2, RViz, and teleoperation support.
- C++ RTMDet ONNX plant detection with CPU/auto/CUDA backend selection.
- Annotated image and 2D detection outputs.
- Initial laser/image target alignment and velocity calculation.
- Unit tests for the current mission-processing method.

Still evolving:

- Connecting detector outputs to mission processing.
- Complete mission orchestration/state management.
- Platform and sensor class implementations.
- Plant approach, removal, verification, and recovery behaviours.
- A validated autonomous navigation workflow.
- Final GUI and operator workflow.
- Broader test coverage and stable interfaces between perception, mission,
  navigation, and platform layers.

These boundaries are expected to change. When adding a subsystem, update the
relevant section above and the repository overview so this document remains a
useful map of the project.
