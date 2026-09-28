# Foxglove GUI Structure

## Role of the GUI

The GUI is an operator-facing view and command surface for the ROS 2 system. It should make the state of the mission, Wombat, Glider, sensing, and plant-detection pipeline easy to inspect while keeping decision-making and robot safety inside ROS 2 nodes.

The GUI should be designed around three questions:

1. What is the system doing now?
2. What evidence does the system have?
3. What command can the operator safely send next?

## Connection Model

```text
+-------------------+       ROS 2 topics       +----------------------+
| Mission node      | <----------------------> | Wombat / Glider nodes|
+-------------------+                           +----------------------+
          ^                                               ^
          |                                               |
          +---------------- Foxglove Bridge ---------------+
                                  ^
                                  | Foxglove WebSocket
                                  v
                    +-----------------------------+
                    | Foxglove layout              |
                    | standard panels              |
                    | custom operator panel        |
                    +-----------------------------+
```

The bridge should normally run alongside the simulation or robot system. For local development, Foxglove connects to `ws://localhost:8765`. For a remote robot, use a secured network connection or Foxglove remote access rather than exposing an unauthenticated bridge publicly.

## Layout Structure

The initial layout should use a small number of stable areas rather than trying to place every topic on screen at once.

```text
+------------------------------------------------------------------+
| Connection / mission state / robot freshness / emergency state   |
+-------------------------------+----------------------------------+
| Primary camera                | Processed camera / detections   |
| Glider or selected robot      | Boxes, labels, confidence      |
+-------------------------------+----------------------------------+
| Map / 3D / region view        | Mission and detection status   |
| poses, goals, poo, TF         | weeds, poo, sonar, progress   |
+-------------------------------+----------------------------------+
| Operator controls: region selection and guarded Wombat command   |
+------------------------------------------------------------------+
```

### Header: system state

Display:

- Foxglove connection state.
- Last message time for important topics.
- Current mission state.
- Active robot or namespace.
- Wombat and Glider health state.
- Whether commands are enabled.
- Emergency-stop state if the system exposes one.

The header should use status indicators derived from message age and explicit ROS status, not from GUI assumptions.

### Camera area

Display:

- `/glider/camera/image` as the glider live feed.
- `/wombat/camera/image` as the Wombat live feed when available.
- The concrete processed-image topic or topics replacing `/**/processed_image`.

The operator should be able to select the primary camera and compare raw and processed views. If RGB-D data is split into color, depth, and camera-info topics, keep those topic relationships documented in the panel configuration.

### Map and spatial area

Use a standard Foxglove 3D panel where possible. Display:

- Region boundary from `/region` or a derived visualization.
- Robot poses and TF.
- `/poo` locations.
- `/glider/trajectory_goals` with goal completion state.
- Weed positions from `/plant_detection/weeds`.
- Navigation map and path when those topics are available.

A custom converter or support node may be needed if the custom messages cannot be consumed directly by the 3D panel. Prefer converting custom data into standard visualization messages rather than duplicating geometry logic in the UI.

### Mission and perception status

Display:

- Number of detected weeds.
- Detection confidence summary.
- Locations or poses associated with detections.
- Current region and region validity.
- Poo targets and whether each has been processed.
- Glider trajectory goals and completion flags.
- Mission progress and current task.

Use Raw Messages or Table panels during development. The custom panel can later aggregate these into a concise operator view.

### Sensor area

Display:

- `/wombat/lidar` using a LaserScan-compatible panel or 3D visualization.
- `/glider/sonar` as a numeric value and a freshness indicator.
- Sensor timestamps and frame IDs where relevant.

Sensor status should distinguish between a valid zero measurement and no recent measurement.

### Operator controls

The control area should contain two separate workflows:

#### Region selection

1. Allow the operator to select four points in a map or spatial view.
2. Show the polygon before publishing.
3. Validate that the points form a usable region.
4. Ask for confirmation.
5. Publish the typed message on `/region`.
6. Show the mission acknowledgement and active region.

The current plan describes `/region` as a vector of poses. A custom region message may be preferable to `geometry_msgs/PoseArray` because it can include an ID, frame, validity, and timestamp.

#### Wombat movement

The current planned topic is `/wombat/control` with `geometry_msgs/Twist` semantics:

- `linear.x`: forward/reverse velocity.
- `angular.z`: yaw rate.
- Other fields should be zero unless the Wombat interface explicitly uses them.

The panel should include:

- Dead-man or hold-to-run behavior for continuous movement.
- A visible stop action that publishes zero velocity.
- Maximum linear and angular limits.
- Clear indication that commands are live.
- No retained movement command after disconnect or panel reload.

The Wombat node should also enforce its own limits and timeout. GUI validation is not a safety boundary.

## Topic Ownership and Integration

| Subsystem | Publishes | Subscribes | GUI role |
| --- | --- | --- | --- |
| Mission | `/poo`, mission status, possibly region state | `/region`, `/plant_detection/weeds` | Select region, observe mission state |
| Wombat | `/wombat/lidar`, `/wombat/camera/image`, status | `/poo`, `/wombat/control` | View sensors, send bounded movement |
| Glider | `/glider/trajectory_goals` result/state, `/glider/sonar`, `/glider/camera/image` | trajectory goals or mission commands | View camera, goals, sonar, progress |
| Plant detection | `/plant_detection/weeds`, processed images | camera topics | View detections and confidence |
| GUI | `/region`, `/wombat/control` | all GUI-facing observation topics | Operator view and command surface |

The table describes intended ownership. Verify it against the running ROS graph because the current source tree and package build files are still evolving.

## Message Contract Guidance

### Prefer explicit custom messages

The following data should likely be represented by named messages in a shared interface package:

- `Region`: header, region ID, four or more points, frame ID, validity.
- `TrajectoryGoal`: pose, goal ID, target type, completion state.
- `TrajectoryGoalArray`: timestamp and goals.
- `WeedDetection`: pose, plant class, confidence, image region, detection ID.
- `WeedDetectionArray`: timestamp, source robot, and detections.
- `SystemStatus`: subsystem state, error code, message, timestamp.
- `CommandAck`: command ID, accepted state, execution state, error text.

Use standard messages where they already express the contract well:

- `geometry_msgs/msg/Twist` for Wombat velocity commands.
- `geometry_msgs/msg/Pose` for a single pose if frame and semantics are sufficient.
- `geometry_msgs/msg/PoseArray` for a simple homogeneous collection of poses.
- `sensor_msgs/msg/LaserScan` for Wombat lidar.
- `sensor_msgs/msg/Image` and `sensor_msgs/msg/CameraInfo` for camera data.

### Frames and timestamps

Every spatial message should have a clear frame and timestamp. The GUI should not silently combine poses from different frames. The panel should display a warning when a message is missing a frame, has an unexpected frame, or is too old to be trustworthy.

## Custom Foxglove Panel Responsibilities

The custom panel should own presentation and operator interaction, not mission logic.

### Subscriptions

Subscribe to concrete topic names once they are agreed:

```ts
context.subscribe([
  { topic: "/region" },
  { topic: "/poo" },
  { topic: "/glider/trajectory_goals" },
  { topic: "/wombat/lidar" },
  { topic: "/wombat/camera/image" },
  { topic: "/glider/camera/image" },
  { topic: "/plant_detection/weeds" },
  { topic: "/wombat/status" },
]);
```

Processed images should be added using their actual topic names, not the `/**/processed_image` planning wildcard.

### Publishing

For ROS 2 over Foxglove Bridge, the panel must advertise before publishing and provide the ROS 2 datatype definition for custom messages:

```ts
context.advertise?.("/wombat/control", "geometry_msgs/msg/Twist");
context.publish?.("/wombat/control", {
  linear: { x: 0.0, y: 0.0, z: 0.0 },
  angular: { x: 0.0, y: 0.0, z: 0.0 },
});
```

For custom messages, use the exact ROS 2 Humble message schema and definitions expected by the bridge. Publishing should be disabled when the current data source does not support `advertise` or `publish`.

### Panel state

Persist only user preferences, such as:

- Selected robot namespace.
- Selected camera.
- Panel display mode.
- Detection confidence threshold for visualization.

Do not persist active velocity commands or assume a command remains active after a reconnect.

## Integration with Existing Repository Sections

- **`src/mission_node.*`**: Defines the mission-facing region, poo, trajectory, plant-detection, and status contracts.
- **`src/wombat/`**: Consumes `/poo` and `/wombat/control`; publishes Wombat sensor topics and should publish control acknowledgement/status.
- **`src/glider/`**: Publishes camera and sonar data and consumes trajectory goals or mission commands.
- **`src/plant_perception/`**: Publishes weed detections and processed images; should document image encoding, detection schema, and topic names.
- **`src/sensing/`**: Supplies sensor data and frame conventions needed by the GUI.
- **`launch/launch.py`**: Eventually provides a launch option for `foxglove_bridge`, but the bridge can initially be started separately during development.
- **`test/`**: Add message-contract tests and fake publisher/subscriber tests before relying on the full simulation.
- **`docs/`**: Keep this structure document, the implementation plan, and later message/interface specifications together.

## Suggested Custom Panel Modules

```text
wombat-foxglove-extension/
  src/
    index.ts                 Extension registration
    OperatorPanel.tsx        Main panel layout
    topicSubscriptions.ts    Topic names and subscriptions
    messageTypes.ts          ROS message type aliases
    commandPublisher.ts      Advertise, validate, and publish commands
    statusModel.ts           Freshness and subsystem status aggregation
    regionSelection.ts       Four-point validation and region message creation
    styles.css               Panel-specific styling
  package.json
  tsconfig.json
```

Keep topic names and message schemas in one place. This avoids scattering strings across the panel and makes namespace changes easier when single-robot and multi-robot modes are tested.

## Open Questions

- What are the exact ROS package and message names for region, trajectory goals, weeds, sonar, and status?
- Will the GUI select one robot namespace or display Wombat and Glider simultaneously?
- Are the camera topics raw RGB, RGB-D, or separate color/depth streams?
- What is the exact set of legal Wombat commands and velocity limits?
- What node publishes command acknowledgement and mission state?
- Which frame should region points use: global map, world, or a mission-specific frame?
- Is a GUI-only aggregation node needed, or can Foxglove panels consume all source topics directly?
