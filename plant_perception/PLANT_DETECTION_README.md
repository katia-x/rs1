## Plant Detection
The pipeline receives images from the Husky camera, converts them to OpenCV format, and runs RTMDet Tiny object detection.

### Plant Detection Pipeline Requirements:
- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- PyTorch 2.1.2 CPU
- MMCV 2.1.0
- MMEngine 0.10.7
- MMDetection 3.3.0

### Plant Detection Installation:
Assuming you have a ROS workspace set up for this project and have cloned this RS1 repo, follow these instructions to setup the plant detection pipeline on your machine.

#### 1 Create a python environment
Assuming you are naming this virtual environment "41068_perception" and creating it inside a venvs directory:
```
python3 -m venv --system-site-packages ~/venvs/41068_perception
source ~/venvs/41068_perception/bin/activate
python3 -m pip install --upgrade pip
```
Install the CPU versions required by the detector:
```
python3 -m pip install \
  torch==2.1.2 \
  torchvision==0.16.2 \
  torchaudio==2.1.2 \
  --index-url https://download.pytorch.org/whl/cpu

python3 -m pip install openmim

mim install "mmengine==0.10.7"
mim install "mmcv==2.1.0"
```

#### 2 Install MMDetection and the RTMDet Tiny
```
mkdir -p ~/git
cd ~/git

git clone --branch v3.3.0 --depth 1 \
  https://github.com/open-mmlab/mmdetection.git

cd ~/git/mmdetection

python3 -m pip install -v -e .

mkdir -p checkpoints

wget -O checkpoints/rtmdet_tiny_8xb32-300e_coco_20220902_112414-78e30dcc.pth \
  https://download.openmmlab.com/mmdetection/v3.0/rtmdet/rtmdet_tiny_8xb32-300e_coco/rtmdet_tiny_8xb32-300e_coco_20220902_112414-78e30dcc.pth
```
#### 3 Configure the shell environment
Run these commands in every new terminal before building or running the detector:
```
source ~/venvs/41068_perception/bin/activate
source /opt/ros/humble/setup.bash
source ~/41068_ws/install/setup.bash

export PYTHONPATH="$HOME/git/mmdetection:$HOME/venvs/41068_perception/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
```
#### 4 Verify the installation
This checks imports using the same system Python interpreter that ROS uses to launch the node:
```
/usr/bin/python3 -c "
from mmdet.apis import init_detector, inference_detector
import torch, mmcv, mmengine, mmdet

print('Torch:', torch.__version__)
print('MMCV:', mmcv.__version__)
print('MMEngine:', mmengine.__version__)
print('MMDetection:', mmdet.__version__)
print('Imports OK')
"
```
Expected versions:
```
Torch: 2.1.2+cpu
MMCV: 2.1.0
MMEngine: 0.10.7
MMDetection: 3.3.0
Imports OK
```

### Run the plant detection pipeline
First start the Husky Gazebo simulation and confirm its image topic:
```
ros2 topic list | grep image
```
Then run the detector:
```
ros2 run plant_perception detection_node --ros-args \
  -p camera_topic:=/husky1/camera/image \
  -p device:=cpu \
  -p confidence_threshold:=0.2
```
To display incoming image dimensions, enable debug logging:
```
ros2 run plant_perception detection_node --ros-args \
  --log-level detector_node:=debug \
  -p camera_topic:=/husky1/camera/image \
  -p device:=cpu
```

### Troubleshooting
If MMDetection is unavailable, confirm the virtual environment is active and rerun the PYTHONPATH export command.
If the detector receives images but no detections are logged, temporarily lower the threshold:
```
-p confidence_threshold:=0.05
```
To inspect the raw Gazebo camera feed:
```
ros2 run rqt_image_view rqt_image_view /husky1/camera/image
```

The pinned PyTorch CPU command is from PyTorch’s official previous-version installation guidance, and the checkpoint URL is the official RTMDet Tiny model release. [PyTorch](https://pytorch.org/get-started/previous-versions/) [RTMDet Tiny model](https://download.openmmlab.com/mmdetection/v3.0/rtmdet/rtmdet_tiny_8xb32-300e_coco/rtmdet_tiny_8xb32-300e_coco_20220902_112414-78e30dcc.pth)