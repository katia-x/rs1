# Preparing a dataset from ROS bags

From the repository root, use the system ROS Python environment (no training
virtual environment is needed):

```bash
source /opt/ros/humble/setup.bash
python3 tools/prepare_rosbag_dataset.py \
  training/bags/run_01 training/bags/run_02 \
  --output training/dataset_v2_frames
```

The destination must not already exist. The script reads bags offline without
playing them or starting any ROS nodes. It leaves the recordings and existing
labelled dataset untouched.

The output contains:

```text
dataset_v2_frames/
  images/
    train/
    validation/
    test/
  manifest.csv
  summary.json
```

Frames are saved as lossless PNG images at their original resolution. Names
include the source bag, frame index and bag timestamp. The manifest records
the source and outcome of every frame, including duplicate matches.

Exact duplicate detection hashes decoded BGR pixels and their dimensions,
ignoring ROS timestamps and headers. Matching frames are excluded globally
across both recordings and all three splits. Only the first eligible copy is
kept. The source bag order therefore determines which copy survives.

Each recording is split chronologically: the first 70% of its image time span
is training, the next 15% validation, and the final 15% testing. A five-second
gap is excluded around each boundary, 2.5 seconds on either side. Gaps and
duplicate removal mean final image counts will not be exactly 70/15/15.

Do not randomly distribute neighbouring video frames across splits. They can
show almost the same scene and inflate evaluation scores. These temporal splits
reduce that issue, but repeated views and the same plants can still cross split
boundaries. For final evaluation, collect independent routes/scenes and assign
whole recordings to separate sets. Two recordings cannot provide three
independent recording-level splits. Also check whether new recordings revisit
the scene used in the existing training dataset before treating them as unseen
validation/test data.

For a smaller annotation workload, sample at most one retained frame per second:

```bash
python3 tools/prepare_rosbag_dataset.py \
  training/bags/run_01 training/bags/run_02 \
  --output training/dataset_v2_sampled --sample-hz 1
```

Exact matching will retain almost identical frames when a moving camera changes
pixels. Optional `--near-threshold 1` also removes frames whose 64x48 colour
thumbnails have a mean absolute difference of at most 1 on the 0–255 scale.
This is a heuristic: review the retained images before using it because a small
plant may change without changing much of the thumbnail. It is disabled by
default. `--ratios 0.8 0.1 0.1` changes the split, and `--gap-seconds 10` increases
the boundary gap.

## Annotation and training

These folders contain unlabelled images, not a training-ready detection dataset.
In CVAT, create separate tasks for `train`, `validation` and `test`, with the
case-sensitive labels `Weed` and `Plant`. Annotate bounding boxes consistently,
including partially visible plants. Keep useful background-only images and
review class representation in each set after annotation.

Export each task in COCO detection format. Assemble a new labelled dataset with
these paths to match the existing training configuration:

```text
cvat_dataset_v2/
  images/train/
  images/validation/
  images/test/
  annotations/instances_Train.json
  annotations/instances_Validation.json
  annotations/instances_Test.json
```

Preserve the split membership from the manifest. Check that each annotation's
`file_name` resolves relative to its split's image directory and that class IDs
consistently identify `Weed` and `Plant` in all exports. Set `data_root` in a new
copy of `training/configs/rtmdet_tiny_plants.py` to the labelled v2 directory.
The supplied training config loads the validation/test annotation paths from
that root as well.

Use validation results to choose the model, thresholds and training settings.
Reserve the test set for the final evaluation.
