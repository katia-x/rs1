#!/usr/bin/env bash
# Build a user-local OpenCV CUDA stack and a separate ROS Humble overlay.
set -euo pipefail

WOMBAT_REPO="${WOMBAT_REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
GPU_WORKSPACE="${GPU_WORKSPACE:-$HOME/opencv_gpu_ws}"
OPENCV_PREFIX="${OPENCV_PREFIX:-$HOME/opt/opencv-cuda}"
CUDNN_PREFIX="${CUDNN_PREFIX:-$HOME/opt/cudnn-9.7.1}"
CUDA_ROOT="${CUDA_ROOT:-/usr/local/cuda-12.8}"
SOURCE_ROOT="${SOURCE_ROOT:-$HOME/git}"
BUILD_JOBS="${BUILD_JOBS:-3}"
GPU_ARCH="${GPU_ARCH:-$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader | head -n1 | tr -d ' ')}"

test -f "$WOMBAT_REPO/machine_learning/models/rtmdet_tiny_plants.onnx" || {
  echo "Copy the exported rtmdet_tiny_plants.onnx into machine_learning/models first." >&2; exit 1;
}
test -x "$CUDA_ROOT/bin/nvcc" || { echo "CUDA toolkit missing at $CUDA_ROOT" >&2; exit 1; }
test -f "$CUDNN_PREFIX/include/cudnn.h" || { echo "cuDNN headers missing at $CUDNN_PREFIX" >&2; exit 1; }
test -f "$CUDNN_PREFIX/lib/libcudnn.so.9" || { echo "cuDNN libraries missing at $CUDNN_PREFIX" >&2; exit 1; }
[[ "$GPU_ARCH" =~ ^[0-9]+\.[0-9]+$ ]] || { echo "Invalid GPU_ARCH=$GPU_ARCH" >&2; exit 1; }
test -f /opt/ros/humble/setup.bash || { echo "ROS 2 Humble is required" >&2; exit 1; }
mkdir -p "$SOURCE_ROOT" "$GPU_WORKSPACE/src"

fetch_tag() {
  local url="$1" destination="$2" tag="$3"
  if [[ ! -d "$destination" ]]; then
    git clone --branch "$tag" --depth 1 "$url" "$destination"
  fi
  [[ "$(git -C "$destination" describe --tags --exact-match HEAD)" == "$tag" ]] || {
    echo "$destination must be at tag $tag; existing checkout was not modified." >&2; exit 1;
  }
}
fetch_tag https://github.com/opencv/opencv.git "$SOURCE_ROOT/opencv-cuda" 4.12.0
fetch_tag https://github.com/opencv/opencv_contrib.git "$SOURCE_ROOT/opencv-contrib-cuda" 4.12.0

cmake -S "$SOURCE_ROOT/opencv-cuda" -B "$SOURCE_ROOT/opencv-cuda/build" \
  -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX="$OPENCV_PREFIX" \
  -D OPENCV_EXTRA_MODULES_PATH="$SOURCE_ROOT/opencv-contrib-cuda/modules" \
  -D CUDA_TOOLKIT_ROOT_DIR="$CUDA_ROOT" -D CUDA_ARCH_BIN="$GPU_ARCH" -D CUDA_ARCH_PTX= \
  -D WITH_CUDA=ON -D WITH_CUBLAS=ON -D WITH_CUDNN=ON -D OPENCV_DNN_CUDA=ON \
  -D CUDNN_INCLUDE_DIR="$CUDNN_PREFIX/include" -D CUDNN_LIBRARY="$CUDNN_PREFIX/lib/libcudnn.so.9" \
  -D BUILD_LIST=core,imgproc,imgcodecs,dnn,cudev -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF \
  -D BUILD_EXAMPLES=OFF -D BUILD_opencv_python3=OFF -D WITH_FFMPEG=OFF \
  -D WITH_GSTREAMER=OFF -D WITH_OPENEXR=OFF -D WITH_IPP=OFF \
  -D CMAKE_INSTALL_RPATH="$OPENCV_PREFIX/lib;$CUDNN_PREFIX/lib;$CUDA_ROOT/lib64" \
  | tee "$GPU_WORKSPACE/opencv-configure.log"
grep -Eq 'NVIDIA CUDA: +YES' "$GPU_WORKSPACE/opencv-configure.log"
grep -Eq 'cuDNN: +YES' "$GPU_WORKSPACE/opencv-configure.log"
cmake --build "$SOURCE_ROOT/opencv-cuda/build" --parallel "$BUILD_JOBS"
cmake --install "$SOURCE_ROOT/opencv-cuda/build"

if [[ ! -d "$GPU_WORKSPACE/src/vision_opencv" ]]; then
  git clone --branch humble --depth 1 https://github.com/ros-perception/vision_opencv.git "$GPU_WORKSPACE/src/vision_opencv"
fi
# ROS setup scripts may read variables that are unset in a fresh shell.
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$GPU_WORKSPACE"
colcon build --base-paths src "$WOMBAT_REPO" --packages-select cv_bridge rs1 \
  --allow-overriding cv_bridge --symlink-install --cmake-args \
  -DOpenCV_DIR="$OPENCV_PREFIX/lib/cmake/opencv4" -DBUILD_TESTING=OFF \
  -DCMAKE_BUILD_TYPE=Release \
  "-DCMAKE_INSTALL_RPATH=$OPENCV_PREFIX/lib;$CUDNN_PREFIX/lib;$CUDA_ROOT/lib64"

# Shell-escape paths so activation works even when user paths contain spaces.
{
  echo '# Source this file in a fresh terminal; no training venv is required.'
  echo 'source /opt/ros/humble/setup.bash'
  printf 'source %q\n' "$GPU_WORKSPACE/install/setup.bash"
  printf 'export LD_LIBRARY_PATH=%q:"${LD_LIBRARY_PATH:-}"\n' "$OPENCV_PREFIX/lib:$CUDNN_PREFIX/lib:$CUDA_ROOT/lib64"
} > "$GPU_WORKSPACE/activate_gpu.sh"
echo "Done. Source $GPU_WORKSPACE/activate_gpu.sh in a fresh terminal."
