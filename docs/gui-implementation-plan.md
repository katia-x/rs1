# Foxglove GUI Implementation Plan

## Purpose

Build a Foxglove operator GUI around the ROS 2 interfaces that are currently
implemented in `src`. The first version should be read-only wherever the
system does not expose an explicit GUI-safe command or status contract.

Foxglove remains a client of ROS 2. Mission planning, perception, navigation,
and safety stay in ROS 2 nodes.

## Current implementation status

The source now provides a concrete mission and perception path:

1. The GUI publishes a `geometry_msgs/msg/PoseArray` to `/region`.
2. `mission_node` validates the region frame, stores its vertices, and builds
   alternating search lanes.
3. `mission_node` subscribes to `vision_msgs/msg/Detection2DArray`, by default
   on `/husky1/detections`.
4. `mission_node` exposes the `rs1/srv/WombatNextGoal` service named
   `wombat_next_goal`.
5. `wombat_node` calls that service and sends the returned goal to the Nav2
   `/husky1/navigate_to_pose` action.
6. Plant perception publishes an annotated image and
   `vision_msgs/msg/Detection2DArray`.
7. Mission processing correlates detections with
   `/husky1/camera/depth/points`, transforms high-confidence weeds into the
   `map` frame, and de-duplicates stored weed coordinates.
8. Mission processing also consumes `/husky1/scan` and contains the
   bounding-box-to-velocity helper, but there is no GUI-facing velocity
   subscriber or velocity command publisher wired into the current Wombat
   node.

The GUI must not claim that stored weed coordinates, navigation state, or
robot health are observable until those values are published on ROS topics or
services.

## Current ROS contract

| Interface | Direction | Type | Current source | Foxglove use |
| --- | --- | --- | --- | --- |
| `/region` | GUI -> mission | `geometry_msgs/msg/PoseArray` | `MissionNode` subscription | Implement region selection and publishing |
| `wombat_next_goal` | Wombat -> mission | `rs1/srv/WombatNextGoal` | `MissionNode` service | Inspect/call only if Foxglove service support and operator semantics are agreed |
| `/husky1/navigate_to_pose` | Wombat -> Nav2 | `nav2_msgs/action/NavigateToPose` | `WombatNode` action client | Observe through Nav2 action tooling if available; not a direct GUI command |
| `/husky1/detections` | detector -> mission/GUI | `vision_msgs/msg/Detection2DArray` | Plant detector publisher and mission subscription | Show classes, boxes, scores, and timestamps |
| `/husky1/detections/image` | detector -> GUI | `sensor_msgs/msg/Image` | Plant detector publisher | Image panel with drawn boxes and labels |
| `/husky1/camera/image` | camera -> detector | `sensor_msgs/msg/Image` | Plant detector subscription default | Optional raw camera panel when published |
| `/husky1/camera/depth/points` | camera -> mission | `sensor_msgs/msg/PointCloud2` | Mission processing subscription | 3D/depth panel and freshness display |
| `/husky1/scan` | lidar -> mission/GUI | `sensor_msgs/msg/LaserScan` | Mission processing subscription | LaserScan/3D panel |
| `/tf` and `/tf_static` | transform system -> consumers | TF messages | Mission and robot system | 3D frame and robot transform display |

Topic names for the detector are parameters and can be changed:
`camera_topic`, `annotated_topic`, and `detections_topic`. The layout should
show the configured names rather than assuming the defaults when a launch
configuration overrides them.

## Interfaces not currently implemented

The following names from the original GUI design are still planned rather
than supported by the current source:

- `/wombat/control`
- `/wombat/lidar`
- `/wombat/camera/image`
- `/glider/camera/image`
- `/glider/sonar`
- `/glider/trajectory_goals`
- `/plant_detection/weeds`
- `/poo`
- mission status, Wombat/Glider health, emergency-stop state, and command
  acknowledgement topics

The existing Glider and sensing files are placeholders. The mission node keeps
transformed weed points internally; it does not publish them. Therefore a
Foxglove panel cannot yet display a live global weed marker list.

## Recommended Foxglove layout

### Header and diagnostics

Display:

- Foxglove connection state.
- Last message time and age for `/region`, `/husky1/detections`,
  `/husky1/detections/image`, `/husky1/camera/depth/points`, and
  `/husky1/scan`.
- Active frames from message headers.
- Detector topic names and data availability.

Do not display mission completion, emergency state, or robot health as fact
until explicit status messages are added.

### Perception

- Image panel for `/husky1/detections/image`.
- Optional raw Image panel for `/husky1/camera/image`.
- Raw Message or Table panel for `/husky1/detections`.
- Detection summary showing total detections, weed count, plant count, and
  confidence values.

The detector uses the configured confidence threshold for inference. Mission
weed mapping applies an additional confidence threshold of `0.75`, so the GUI
should distinguish detector detections from mission-qualified weeds.

### Spatial and sensor view

Use a standard Foxglove 3D panel for:

- TF and robot frames.
- `/husky1/camera/depth/points`.
- `/husky1/scan`.
- The `/region` PoseArray, if the panel supports displaying it directly or a
  small visualization topic is added.

The GUI can show the region and sensor evidence now, but it cannot show the
mission's internal `stored_weed_coordinates_` without a new publisher.

### Mission and navigation view

Use Raw Message/Table panels for:

- The last `/region` message.
- Region frame and vertex count.
- Configured mission parameters such as `global_frame`,
  `search_goal_distance`, `camera_width_of_view`, and `location_tolerance`.
- Nav2 action feedback/result if exposed by the running bridge and tooling.

The `wombat_next_goal` service response contains a `PoseStamped`, `success`,
and `is_weed`, but the service is intended for the Wombat node's control loop.
It should not be polled continuously by a GUI because that would mutate or
race the mission's search-goal state.

## Phased implementation

### Phase 0: Verify the running graph

Before creating the saved layout:

```bash
ros2 topic list
ros2 topic type /region
ros2 topic type /husky1/detections
ros2 topic type /husky1/detections/image
ros2 topic type /husky1/camera/depth/points
ros2 topic type /husky1/scan
ros2 service type /wombat_next_goal
ros2 action list
ros2 action info /husky1/navigate_to_pose
```

Also record QoS, frequency, frame IDs, and sample messages. Confirm whether
the configured detector topics differ from the defaults.

### Phase 1: Read-only Foxglove layout

Create a standard-panel layout containing:

- Annotated detection image.
- Raw camera image, if available.
- Detection table.
- LaserScan panel.
- PointCloud2/3D panel.
- TF and robot-frame view.
- Region Raw Message/Table panel.
- Topic freshness and connection diagnostics.

This phase is implementable against the current source without adding ROS
interfaces.

### Phase 2: Region-selection panel

Add a custom Foxglove panel that:

1. Lets the operator select at least three region vertices in the `map` frame.
2. Shows the polygon before publishing.
3. Rejects an empty region or a frame other than the configured global frame.
4. Publishes one typed `geometry_msgs/msg/PoseArray` to `/region` after
   confirmation.
5. Shows the last published region and its vertex count.

The mission node currently accepts any non-empty PoseArray in the `map` frame;
the GUI should apply stronger polygon validation if it requires a usable
search region.

There is no region acknowledgement topic. The panel should confirm only that
the message was published and that a later `/region` observation is visible;
it must not report mission acceptance as a separate state.

### Phase 3: Publish mission observability

Add ROS publishers before implementing a richer mission panel:

- Active region or region acknowledgement.
- Search-line/waypoint visualization as `PoseArray`, `Path`, or markers.
- Stored global weed coordinates as a frame-aware message or visualization
  marker array.
- Current goal and goal index.
- Mission and Nav2 status.

These additions would enable map-level mission progress and weed displays.

### Phase 4: Wombat operator commands

Do not implement direct movement controls from the current source. First add
an explicit Wombat command interface with:

- bounded command type;
- dead-man/hold-to-run semantics;
- timeout and zero-command behavior;
- emergency-stop integration;
- command acknowledgement and result;
- node-side limits independent of GUI validation.

Only then add Foxglove `advertise()`/`publish()` controls. The existing
`wombat_next_goal` service and Nav2 action are mission-navigation internals,
not a safe substitute for a GUI movement command.

### Phase 5: Glider and additional sensors

Implement and publish the Glider camera, sonar, trajectory goals, and status
interfaces before adding those panels. Add `/poo` and any plant-target
interfaces only once their message types and ownership are defined.

## Foxglove extension shape

Keep a custom TypeScript extension separate from the ROS 2 package. The first
panel needs:

```text
src/
  index.ts
  OperatorPanel.tsx
  topicSubscriptions.ts
  detectionModel.ts
  regionSelection.ts
  freshness.ts
  styles.css
```

Use `context.subscribe()` for observation, `context.saveState()` for selected
topics and display settings, and `context.advertise()`/`context.publish()` for
the `/region` PoseArray. Do not persist active movement commands.

## Testing checklist

### Current-source GUI tests

- The layout opens with all currently configured topics available.
- Annotated images render with timestamps and detection overlays.
- Detection rows show class IDs, scores, and bounding boxes.
- Detection count and freshness update when messages arrive.
- LaserScan and PointCloud2 panels distinguish stale data from valid data.
- Region selection publishes exactly one valid `geometry_msgs/msg/PoseArray`.
- A region in the wrong frame is visibly rejected by the GUI and by the
  mission node.
- A later `/region` observation confirms that the region was received.
- Reconnects do not republish the last region or retain command state.

### Tests blocked on future interfaces

- Live global weed markers.
- Mission search progress and current-goal display.
- Wombat command acknowledgement.
- Emergency-stop state.
- Glider camera, sonar, and trajectory progress.

## Safety boundary

Foxglove validation is not a safety boundary. The GUI must remain read-only
for movement until the Wombat node exposes a dedicated, bounded, acknowledged
command path. Mission nodes and robot controllers must enforce limits and
timeouts themselves.
