# C++ OpenCV plant perception

The demo launches `rs1/plant_detector` by default. Runtime inference requires
OpenCV and ROS 2, without Python ML dependencies. The ONNX export remains a training-side tool.

Build from the repository root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rs1 --symlink-install
source install/setup.bash
ros2 launch rs1 rs1_demo.launch.py parrot:=false rviz:=true device:=auto
```

Launch arguments: `perception` (default true), `device` (auto/cpu/cuda),
`model_path`, `camera_topic`, `confidence_threshold` (0.3), `nms_threshold` (0.65),
`inference_rate_hz` (2.0). Disable detection with `perception:=false`.
The model resolves from the installed rs1 package share directory, not a user's home directory.

`auto` checks OpenCV's CUDA DNN target and performs a warm-up forward pass;
if unavailable or unsuccessful it reloads on CPU. A CUDA runtime exception in auto mode
also triggers CPU reload and retry. `cpu` always uses CPU. `cuda` requires a working CUDA
target and exits with a clear error if unavailable. The Ubuntu OpenCV 4.5.4 build on this
laptop has no CUDA DNN target, so auto selects CPU even though the laptop has an NVIDIA GPU.
GPU inference requires a CUDA/cuDNN-enabled OpenCV build; installing PyTorch CUDA does not enable it.

Published topics:

- `/husky1/detections/image`: BGR8 annotated image, including frames without detections.
- `/husky1/detections`: `vision_msgs/Detection2DArray` with pixel-space boxes,
  class names (Weed/Plant), and confidence scores. These are 2D detections, not 3D positions.

Both outputs preserve the input acquisition timestamp and camera frame ID.
The camera subscription uses best-effort sensor QoS and retains only the latest image.
Separate executor callback groups let acquisition continue while inference runs.
RViz's saved Husky configuration includes an Image display for the annotated topic.

The decoder expects the six raw outputs from `training/export_onnx.py`: classification
logits and stride-scaled left/top/right/bottom distances at strides 8,16,32, point offset 0.
Input is fixed at 640x640: aspect-preserving resize, bottom/right padding 114,
BGR mean [103.53,116.28,123.675], std [57.375,57.12,58.395], float32 NCHW.
Class names can be overridden using the node's `class_names` parameter when replacing
the model, but architecture/preprocessing changes require updating the decoder.

Training baselines and verification artifacts stay in machine_learning/baseline and
machine_learning/verification; only machine_learning/models/*.onnx is installed.
Share the model file with teammates along with the repository source.

The standalone node defaults to `best_effort`; the demo defaults to `reliable`
for its reliable Gazebo bridge. For a reliable camera publisher,
`camera_reliability:=reliable` enables retransmission of dropped large-image packets.
Use `ros2 topic info /husky1/camera/image --verbose` to inspect the publisher.
A reliable subscriber cannot receive from a best-effort publisher.

## Verified behaviour

The C++ ROS node was checked against all 14 reference images in CPU and auto modes.
Both modes matched all 65 detections, preserved timestamps/frame IDs, and published
annotated images including the zero-detection frame. Explicit CUDA and invalid modes
were correctly rejected on the installed CPU-only OpenCV build. A headless demo launch
also produced live 720x480 BGR8 annotated images and timestamped detections from Gazebo.
Successful CUDA execution and GPU-to-CPU runtime recovery require CUDA hardware/backend
and were not exercised here. RViz display configuration is supplied; GUI viewing was not
automated.

Reports: `machine_learning/verification/rtmdet_tiny_v1/ros_integration.json` and
`demo_smoke.json`. Re-run tests after sourcing ROS and this workspace:

```bash
python3 training/test_cpp_ros.py
python3 training/test_cpp_demo.py
```

Tests use isolated ROS domains 87/88; the demo test uses a separate Gazebo partition.
The reference JSON/image folders from `training/compare_opencv.py` must be available.

## Optional GPU overlay

CUDA inference is now verified using OpenCV 4.12.0 in a separate ROS overlay.
See [GPU dependency setup](gpu_inference_setup.md) for installation and activation.
The original system-OpenCV workspace remains CPU-only.
