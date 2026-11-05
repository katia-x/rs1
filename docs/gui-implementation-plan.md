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
   `map` frame, maintains smoothed, time-limited weed tracks, and
   de-duplicates observations by association distance.
8. `mission_node` publishes the tracked weed estimates as a
   `visualization_msgs/msg/MarkerArray` and publishes the currently selected
   target as both `geometry_msgs/msg/PointStamped` and a
   `visualization_msgs/msg/Marker`.
9. Mission processing also consumes `/husky1/scan` and contains the
   bounding-box-to-velocity helper, but there is no GUI-facing velocity
   subscriber or velocity command publisher wired into the current Wombat
   node.

The GUI can now display tracked and selected weed positions. It still must not
claim that navigation state, mission completion, robot health, or command
acknowledgement are observable until those values are published on ROS topics
or actions.

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
| `/husky1/target_weed` | mission -> GUI/consumers | `geometry_msgs/msg/PointStamped` | `MissionNode` publisher | Show the currently selected weed target |
| `/husky1/target_weed_marker` | mission -> GUI/consumers | `visualization_msgs/msg/Marker` | `MissionNode` publisher | Show or clear the selected target in 3D |
| `/husky1/weed_estimate_markers` | mission -> GUI/consumers | `visualization_msgs/msg/MarkerArray` | `MissionNode` publisher | Show the current tracked weed estimates |
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
- mission status, Wombat/Glider health, emergency-stop state, command
  acknowledgement, and navigation-result topics intended for the GUI

The existing Glider and sensing files are placeholders. The mission node does
publish visualization markers for weed estimates, but it does not publish a
typed weed-detection array, persistent weed database, or mission-status
message.

## Recommended Foxglove layout

### Header and diagnostics

Display:

- Foxglove connection state.
- Last message time and age for `/region`, `/husky1/detections`,
  `/husky1/detections/image`, `/husky1/camera/depth/points`, and
  `/husky1/scan`, `/husky1/weed_estimate_markers`, and
  `/husky1/target_weed`.
- Active frames from message headers.
- Detector topic names and data availability.
- Marker freshness and the number of currently rendered weed estimates.

Do not display mission completion, emergency state, or robot health as fact
until explicit status messages are added.

### Perception

- Image panel for `/husky1/detections/image`.
- Optional raw Image panel for `/husky1/camera/image`.
- Raw Message or Table panel for `/husky1/detections`.
- Detection summary showing total detections, weed count, plant count, and
  confidence values.
- A separate summary for mission-qualified weeds. These are detections that
  have passed the mission confidence threshold, depth/ROI validation,
  timestamp matching, TF transformation, and track association.

The detector uses the configured confidence threshold for inference. Mission
weed mapping applies an additional confidence threshold of `0.75`, so the GUI
should distinguish detector detections from mission-qualified weeds.

### Spatial and sensor view

Use a standard Foxglove 3D panel for:

- TF and robot frames.
- `/husky1/camera/depth/points`.
- `/husky1/scan`.
- The `/region` PoseArray, if the panel supports displaying it directly.
- `/husky1/weed_estimate_markers` for all active tracked weed estimates.
- `/husky1/target_weed_marker` for the currently selected target.
- `/husky1/target_weed` in a Raw Message or point overlay panel.

The estimate marker topic is a complete snapshot: the node sends a
`DELETEALL` marker followed by the current tracks. It is also transient-local,
so a late Foxglove subscriber can reconstruct the latest snapshot. Tracks
expire after the configured `weed.track_timeout`.

### Mission and navigation view

Use Raw Message/Table panels for:

- The last `/region` message.
- Region frame and vertex count.
- Configured mission parameters such as `global_frame`,
  `search_goal_distance`, `camera_width_of_view`, `location_tolerance`, and
  the read-only `weed.*` estimation parameters.
- Active weed target, target timestamp, and target marker visibility.
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
ros2 topic type /husky1/target_weed
ros2 topic type /husky1/target_weed_marker
ros2 topic type /husky1/weed_estimate_markers
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
- Weed estimate MarkerArray panel.
- Selected weed target Marker/PointStamped panels.
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

### Phase 3: Add operator override and top-down mission map

Phase 3 now covers the operator panel required to combine the existing
Foxglove teleop controls with mission-region selection and live Wombat
visualisation. The panel must not directly assume that publishing to the
teleop topic is safe: the ROS side needs an explicit manual-override gate.

#### Manual driving override

Add a bounded, node-side manual/automatic control contract for Wombat:

- Add a latched or otherwise observable manual-override state topic, for
  example `/husky1/manual_override`, with an explicit enabled/disabled value.
- Add a GUI-safe command path for changing that state, using a typed service or
  equivalent request/response interface rather than an unacknowledged topic
  toggle.
- Keep the existing teleop velocity input on `/husky1/cmd_vel`; no separate
 `manual_cmd_vel` topic or velocity relay is required.
- When manual override is enabled, cancel the active Nav2 goal and stop
 requesting autonomous mission goals. The existing Teleop panel can then
 publish directly to `/husky1/cmd_vel`.
- Make the override state observable so the panel can distinguish an active
 manual mode from a request that has not yet been applied.
- Add an acknowledgement/result or observable state transition so the panel
  can distinguish a requested toggle from an applied toggle.
- Apply node-side velocity limits, command timeout/dead-man behavior, and
  safe zero-command handling independently of Foxglove validation.

The existing teleop operation remains the source of manual
`geometry_msgs/msg/Twist` input on `/husky1/cmd_vel`. The custom panel should
only control the override state and should not persist or replay velocity
commands. The override control must default to disabled on startup and
reconnect. Re-enabling automatic mode resumes the existing mission/Nav2 loop.

#### Custom top-down map panel

Extend the separate TypeScript Foxglove extension with an operator panel that
provides a top-down XY view in the configured global frame, normally `map`.
The panel must:

1. Subscribe to `/region` and render the latest region polygon.
2. Allow the operator to select two opposite corners in the map view and
   publish exactly one validated, axis-aligned rectangular
   `geometry_msgs/msg/PoseArray` after confirmation. A third click starts a
   new rectangle.
3. Subscribe to `/husky1/weed_estimate_markers` and render all active tracked
   weed locations as XY points, ignoring the initial `DELETEALL` marker when
   calculating the displayed estimate set.
4. Subscribe to TF and render the current Wombat position from the
   `map -> husky1_base_link` transform.
5. Subscribe to `/husky1/target_weed_marker` and distinguish the currently
   selected weed from the other tracked weeds.
6. Use the marker frame IDs and TF data rather than assuming every incoming
   point is already in `map`.
7. Show stale, missing, and valid-empty data distinctly, including marker
   freshness and TF availability.
8. Preserve panel state such as selected topics, global frame, and display
   settings with `context.saveState()`, but never persist movement commands or
   an enabled manual override.

The existing `/husky1/target_weed` `PointStamped` should be shown as
frame-aware diagnostic data, but the target marker's add/delete lifecycle is
the authoritative visual indication that a target is currently selected.
The existing depth-point, LaserScan, and annotated detection-camera panels
remain standard panels; this custom panel only adds region, XY weed, Wombat,
target, and override controls.

#### Scope boundaries

Do not present search lanes, `search_goal_index_`, mission completion,
navigation success, robot health, or emergency-stop state as definitive GUI
state until corresponding ROS publishers/contracts exist. Search lanes and the
current search-goal index remain private to `MissionNode`; add a dedicated
publisher before visualising them.

Phase 3 implementation and verification should cover:

- manual override disabled, enabled, and disabled again;
- rejected or failed override requests;
- immediate zero output when override is disabled or times out;
- autonomous navigation being blocked while manual override is active;
- region selection and later `/region` observation;
- live weed estimates appearing and expiring in the XY view;
- Wombat pose following TF;
- selected-target marker appearance and deletion;
- stale/missing TF and marker data;
- reconnects not republishing a region, velocity, or override command.

### Phase 4: Wombat operator commands

Phase 3 provides only the manual-override gate around the existing teleop
operation. Do not add autonomous movement controls or new direct movement
semantics from Foxglove. Any future direct command interface must first define:

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

- Search-lane and current-goal visualization.
- Mission search progress and current-goal display.
- Wombat command acknowledgement.
- Emergency-stop state.
- Glider camera, sonar, and trajectory progress.

## Safety boundary

Foxglove validation is not a safety boundary. The GUI must remain read-only
for movement until the Wombat node exposes a dedicated, bounded, acknowledged
command path. Mission nodes and robot controllers must enforce limits and
timeouts themselves.
