# RS1 Robot for Bushland Preservation
Robot for identification and removal of invasive plant species from struggling Australian bushland regions.

## Overview
ROS 2 husky robot which navigates through a Gazebo simulation environment of bushland, identifying and removing invasive and harmful weed species to assist with environmental regeneration.

## Plant Detection
The pipeline receives images from the Husky camera, converts them to OpenCV format, and runs RTMDet Tiny object detection.

### Plant Detection Installation:
Assuming you have a ROS workspace set up for this project and have cloned this RS1 repo, follow these instructions to setup the plant detection pipeline on your machine.

See the [Plant Detection documentation](plant_perception/PLANT_DETECTION_README.md).
