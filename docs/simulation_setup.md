# RS1 simulation setup

This bringup targets ROS 2 Humble and Gazebo Fortress on Ubuntu 22.04.
The robot descriptions and simulation configuration were adapted from the
subject's `41068_ignition_bringup` package (version 1.0.5, declared Apache 2.0).
Their original asset licenses/attributions still apply. That package is not
required at runtime. The supplied environment mesh and materials are unchanged.

## Build and launch

Run from the `rs1` repository in a regular system terminal:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src --rosdistro humble -r -y
colcon build --packages-select rs1 --symlink-install
source install/setup.bash
ros2 launch rs1 rs1_demo.launch.py
```

Alternatively, build from the outer ROS workspace and source that workspace's
`install/setup.bash`. Use the same build location consistently.

Both robots start by default. The demo also runs the C++ mission, Wombat and
plant detector nodes. Weed target estimation and display run inside the mission
node; navigation continues following search goals. See
[weed estimation](weed_estimation.md) for inputs, tuning and validation.

Useful launch options:

| Option | Default | Purpose |
|---|---|---|
| `husky` / `parrot` | `true` / `true` | Enable each robot |
| `world` | `rs1_world` | SDF basename under `worlds/`, without extension |
| `gui` | `true` | Gazebo GUI |
| `rviz` | `false` | RViz window for each enabled robot |
| `slam` / `nav2` | `false` / `false` | Optional subject navigation configuration; Nav2 enables SLAM |
| `use_sim_time` | `true` | Use the bridged simulation clock |
| `headless_rendering` | `false` | EGL camera rendering without a display; use with `gui:=false` |
| `husky_x/y/z/yaw` | `0 / 0 / 1 / 0` | Ground robot spawn pose, metres/radians |
| `parrot_x/y/z/yaw` | `2 / 0 / 1.5 / 0` | Drone spawn pose, metres/radians |

For Husky-only capture with RViz:

```bash
ros2 launch rs1 rs1_demo.launch.py parrot:=false rviz:=true
```

RViz uses each robot's odometry frame, so it works without SLAM. Map and
navigation displays need the corresponding optional nodes. The inherited
navigation parameters are starting points; autonomous navigation is not part
of the validated manual-driving setup. The Parrot retains the demo's simplified
velocity-controlled model, rather than a flight controller.

## Manual driving and camera topics

In another terminal, source ROS and the same install setup, then run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/husky1/cmd_vel
```

Keep the teleop terminal focused and stop the robot before exiting it.

| Topic | Message type |
|---|---|
| `/husky1/camera/image` | `sensor_msgs/msg/Image`, RGB8, 720 x 480 |
| `/husky1/camera/depth/image` | `sensor_msgs/msg/Image`, 32FC1 depth in metres |
| `/husky1/camera/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/husky1/camera/depth/points` | `sensor_msgs/msg/PointCloud2` |
| `/husky1/odom` | `nav_msgs/msg/Odometry`, filtered |
| `/husky1/tf`, `/husky1/tf_static` | Robot transforms |
| `/clock` | Simulation time |

The bridge publishes the depth cloud directly on `/husky1/camera/depth/points`
in `husky1_camera_link`, using X forward, Y left, and Z up.

RGB, depth and calibration also use `husky1_camera_link`, located at the
simulated camera origin. The camera is configured for 3 Hz in simulation time;
wall-clock throughput depends on simulation performance. The demo runs plant
inference and weed estimation, but does not start rosbag recording.

## Autonomous goal flow

Mission supplies one normal search goal at a time through `wombat_next_goal`.
Wombat keeps the lane-end goal active while Nav2 follows it. When Mission
selects a weed, it publishes a complete `rs1/msg/WombatWeedGoal` on
`/wombat/weed_goal`; Wombat cancels the lane goal and sends the weed goal to
Nav2. After the weed is handled or unreachable, Wombat reports the outcome
through the service and Mission resumes the search plan.

Mission publishes `rs1/msg/WombatStatus` on `/wombat/status`. A new region
increments the mission generation and causes Wombat to cancel its current goal
and request a fresh search goal. See [navigation architecture](navigation_architecture.md)
for the ownership boundary between Mission, Wombat, and Nav2. Nav2's active
behavior tree only computes and follows paths; it has no recovery nodes.

## World layout

`worlds/rs1_world.sdf` includes `worlds/model.sdf`. The existing mesh URI
`model://worlds/meshes/Environment_Combined.obj` resolves through the package
share directory, which the launch adds to Gazebo's resource paths. The world
rotates the Y-up OBJ by +90 degrees about X to match Gazebo's Z-up convention.
Physics, scene, contact and sensor systems load from
`config/ignition_server.config`.

## RViz from Snap-installed VS Code

If RViz reports a `/snap/core20/.../libpthread.so.0` symbol lookup error, launch
from a regular system terminal. For an affected VS Code terminal, the following
clears the inherited GTK/GIO overrides for this launch only:

```bash
env -u GTK_PATH -u GTK_EXE_PREFIX -u GIO_MODULE_DIR -u GTK_IM_MODULE_FILE -u GTK_MODULES \
  ros2 launch rs1 rs1_demo.launch.py rviz:=true
```
