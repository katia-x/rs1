# GPU inference setup

This guide enables **C++ OpenCV CUDA inference**, without installing PyTorch,
MMDetection, or a training virtual environment. Python is used by the ROS build/launch
tools and to download/extract native libraries; it does not execute the model.

The tested system is Ubuntu 22.04 x86_64, ROS 2 Humble, an RTX 5060 Laptop GPU (8 GB),
CUDA Toolkit 12.8.2, cuDNN 9.7.1.26, and OpenCV 4.12.0. Other NVIDIA GPUs must be
supported by the toolkit/cuDNN and compiled for their own compute capability.
AMD/Intel GPUs are not supported by this CUDA path; use the CPU build there.

## 1. Prepare the repository and base ROS environment

Get the Wombat repository and complete [simulation setup](simulation_setup.md).
Verify that `machine_learning/models/rtmdet_tiny_plants.onnx` exists. The ONNX model
is required; baseline checkpoints and training datasets are not required to run inference.
If model artifacts were not committed to the repository, obtain the ONNX file from the team.
Run the following from the repository root:

```bash
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  build-essential cmake git pkg-config python3-pip python3-dev python3-numpy \
  python3-colcon-common-extensions libboost-python-dev \
  libjpeg-dev libpng-dev libtiff-dev zlib1g-dev \
  ros-humble-vision-msgs ros-humble-cv-bridge libopencv-dev
```

## 2. Verify the NVIDIA driver and install the toolkit

```bash
nvidia-smi
```

This must show your GPU without a driver/library mismatch. Use Ubuntu's supported
driver installation process if it does not. The build script does not change drivers.

Install CUDA Toolkit 12.8 using [NVIDIA's Ubuntu installation instructions](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/).
Once NVIDIA's package repository is configured:

```bash
sudo apt-get install cuda-toolkit-12-8
/usr/local/cuda-12.8/bin/nvcc --version
nvidia-smi --query-gpu=name,compute_cap --format=csv,noheader
```

The CUDA version displayed by `nvidia-smi` is driver capability, not the installed
toolkit version. Check `nvcc` for the toolkit. The RTX 5060 has compute capability 12.0;
other cards have different values. See [NVIDIA's GPU table](https://developer.nvidia.com/cuda/gpus).

## 3. Install the tested cuDNN native libraries without a venv

The reproducible path uses NVIDIA's pinned wheel only as an archive of C++ headers
and shared libraries. It does not install a Python inference framework.

```bash
mkdir -p "$HOME/Downloads/wombat-gpu-deps"
python3 -m pip download --no-deps \
  --dest "$HOME/Downloads/wombat-gpu-deps" nvidia-cudnn-cu12==9.7.1.26

python3 tools/install_cudnn_local.py \
  "$HOME"/Downloads/wombat-gpu-deps/nvidia_cudnn_cu12-9.7.1.26-*.whl
```

This creates `~/opt/cudnn-9.7.1/include` and `~/opt/cudnn-9.7.1/lib`.
Do not copy native libraries from another team's machine; install them from NVIDIA.
[NVIDIA cuDNN installation guidance](https://docs.nvidia.com/cudnn/installation/backend/linux/)
also describes system package installation, but the above local version is the one verified here.

## 4. Build OpenCV and the ROS GPU overlay

In a **fresh terminal**, outside the training venv, run from the repository root:

```bash
bash tools/setup_opencv_gpu.sh
```

The script detects the first NVIDIA GPU's compute capability, downloads matching
OpenCV/contrib 4.12.0 sources, requires CUDA and cuDNN to be found, and compiles only
core, imgproc, imgcodecs, dnn, and cudev. It installs into `~/opt/opencv-cuda`, then
rebuilds **both cv_bridge and rs1** into `~/opencv_gpu_ws` against that OpenCV.
It preserves Ubuntu's OpenCV and your original CPU workspace.
The native build may take several minutes and needs several GB of disk space and RAM.
Default parallelism is three workers; reduce it if memory is tight:

```bash
BUILD_JOBS=2 bash tools/setup_opencv_gpu.sh
```

For multiple GPUs or custom paths:

```bash
GPU_ARCH=8.6 CUDA_ROOT=/usr/local/cuda-12.8 \
  bash tools/setup_opencv_gpu.sh
```

Replace 8.6 with **your** compute capability. Do not reuse the RTX 5060 value blindly.
Optional variables: `WOMBAT_REPO`, `GPU_WORKSPACE`, `OPENCV_PREFIX`, `CUDNN_PREFIX`,
`SOURCE_ROOT`, `CUDA_ROOT`, `GPU_ARCH`, and `BUILD_JOBS`.
OpenCV's [configuration reference](https://docs.opencv.org/4.12.0/db/d05/tutorial_config_reference.html)
documents WITH_CUDA, WITH_CUDNN and OPENCV_DNN_CUDA.

## 5. Activate and run

In each **fresh terminal**:

Compile with:
```bash
source ~/opencv_gpu_ws/activate_gpu.sh
cd ~/opencv_gpu_ws

colcon build \
  --base-paths src ~/wombat-RS1_ws/rs1 \
  --packages-select rs1 \
  --symlink-install

source activate_gpu.sh
ros2 launch rs1 rs1_demo.launch.py rviz:=true
```

```bash
source ~/opencv_gpu_ws/activate_gpu.sh
ros2 launch rs1 rs1_demo.launch.py parrot:=false rviz:=true device:=auto
```

Look for `Requested device=auto; selected backend=cuda`. To require GPU execution:

```bash
ros2 launch rs1 rs1_demo.launch.py parrot:=false rviz:=true device:=cuda
```

`cpu` forces CPU, `cuda` fails if GPU warm-up cannot run, and `auto` falls back on CPU.
The GPU-built binaries still need their CUDA/cuDNN shared libraries even in CPU mode.
On a computer without that stack, build the normal system-OpenCV workspace instead.
No training venv is needed to launch either C++ build.

The saved RViz configuration displays `/husky1/detections/image`. Structured detections
are published on `/husky1/detections`, retaining the camera's timestamp and frame ID.
For the reliable Gazebo image bridge, the demo defaults to reliable camera QoS.

## 6. Check library consistency and correctness

```bash
ldd ~/opencv_gpu_ws/install/rs1/lib/rs1/plant_detector \
  | grep -E 'opencv|cv_bridge|not found'
```

`cv_bridge` must resolve inside the GPU overlay and OpenCV libraries inside the custom
prefix. All OpenCV libraries should have the same version suffix (412 for this build).
If 4.5 and 4.12 are mixed, use a fresh terminal and source activate_gpu.sh again.
Do not globally replace system OpenCV or add these paths to /etc/ld.so.conf.

If the reference images are available, run:

```bash
python3 training/test_cpp_gpu_ros.py
```

This checks CPU/CUDA/auto detections, annotated images, zero-detection frames, and
acquisition headers in an isolated ROS domain. It expects a GPU-enabled setup;
the ordinary CPU-build test is `training/test_cpp_ros.py`.

The verified standalone benchmark on this laptop averaged **62.54 ms CPU versus
8.66 ms CUDA** over 20 warm runs of one reference image, including preprocessing and
decoding. All 14 validation images matched (65 detections), maximum box difference
0.00735 pixels and confidence difference 0.000194. These are local benchmark numbers,
not a promised live-camera rate; Gazebo was not running during that benchmark and the
node's inference-rate parameter defaults to 2 Hz.

## Troubleshooting

- `CUDA DNN target is unavailable`: check that the GPU overlay is sourced, nvidia-smi
  works, and the OpenCV configuration log says CUDA/cuDNN YES. PyTorch CUDA does not enable OpenCV CUDA.
- `no kernel image is available`: rebuild with the correct GPU_ARCH and a toolkit supporting that GPU.
- Missing libcudnn/libcublas: source activate_gpu.sh; check the local cuDNN prefix and toolkit paths.
- Driver/library mismatch: resolve the NVIDIA driver state, commonly by rebooting after a driver update.
- Build killed for memory: lower BUILD_JOBS; rerun the same build to continue incrementally.
- CUDA startup works but no images arrive: inspect camera topic/QoS rather than changing the model.

To return to the original CPU build, open a fresh terminal and source
`/opt/ros/humble/setup.bash` and your original repository `install/setup.bash`.

Live headless Gazebo integration was also verified with `auto` selecting CUDA,
720x480 annotated images, and matching detection headers. Reports are saved in
`machine_learning/verification/rtmdet_tiny_v1/cuda_comparison.json`,
`gpu_ros_integration.json`, and `gpu_demo_smoke.json`.
