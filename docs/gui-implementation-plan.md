# Foxglove GUI Implementation Plan

## Purpose

Build a Foxglove-based operator GUI that acts as a window into the ROS 2 system and a controlled interface for sending commands back into it. The first version should support observing the mission, wombat, glider, sensing, and plant-detection subsystems before adding complex operator workflows.

The GUI should remain a client of ROS 2. Mission and robot nodes remain responsible for navigation, planning, safety, perception, and command execution.

## Current Topic Contract

This is the working mapping supplied by the team. Message names marked with `?` or `<custom>` must be confirmed before implementation.

| Topic | Direction | Main owner or consumer | Working meaning | Working type |
| --- | --- | --- | --- | --- |
| `/region` | GUI -> Mission | Mission | Four points defining the region to work in | Vector of poses; likely `geometry_msgs/PoseArray` or a custom message |
| `/poo` | Mission -> Wombat | Wombat | Location of poo | `geometry_msgs/Pose` or custom pose message |
| `/glider/trajectory_goals` | Mission -> Glider | Glider | Global locations of poo for glider inspection | Vector of pose and boolean pairs; custom message likely |
| `/wombat/lidar` | Wombat -> GUI/consumers | Wombat | Lidar scan of the wombat surroundings | `sensor_msgs/LaserScan` |
| `/wombat/camera/image` | ML pipeline -> consumers | ML pipeline | Live RGB-D camera feed from wombat | Image type to confirm, likely `sensor_msgs/Image` or image plus depth topic |
| `/**/processed_image` | ML pipeline -> GUI | GUI | Camera feed with identification boxes and information | Image type to confirm; glob must become concrete topic names |
| `/glider/sonar` | Glider -> consumers | Glider | Sonar value directly beneath the glider | Sonar message type to confirm |
| `/glider/camera/image` | Glider -> GUI | Glider | Live glider camera feed | Image type to confirm, likely `sensor_msgs/Image` |
| `/plant_detection/weeds` | Plant detection -> Mission/GUI | Mission and GUI | Plant detection outputs used to evaluate global weed positions | Vector of wombat pose and plant data; custom message likely |
| `/wombat/control` | GUI -> Wombat | Wombat | Skid-steer movement command | `geometry_msgs/Twist` |

### Contract decisions required first

1. Replace `/**/processed_image` with one or more real topic names, such as `/wombat/processed_image` and `/glider/processed_image`.
2. Confirm every topic with `ros2 topic type` while the complete system is running.
3. Define custom messages for regions, trajectory goals, weed detections, and acknowledgements instead of relying on ambiguous arrays or encoded strings.
4. Decide whether images are raw `sensor_msgs/msg/Image`, compressed images, or RGB-D pairs with camera information.
5. Define the command acknowledgement/status path. A GUI should not infer that a movement or mission command succeeded merely because it published a message.

## Recommended Architecture

```text
ROS 2 nodes
  mission <-> wombat <-> glider <-> plant detection / sensing
       ^                         |
       |                         v
  Foxglove Bridge <------ ROS 2 topics
       ^
       |
  Foxglove standard panels + custom operator panel
```

Use Foxglove standard panels for exploration and visualization first. Add a custom Foxglove extension only for controls and domain-specific state aggregation.

### Components

- **Foxglove Bridge**: Exposes ROS 2 topics and schemas over WebSocket.
- **Saved Foxglove layout**: Reusable panel arrangement for cameras, detections, state, and diagnostics.
- **Custom operator panel**: Displays mission state, selected region, detection summaries, and intentional command controls.
- **ROS 2 nodes**: Continue owning all robot behavior and safety decisions.
- **Optional GUI support node**: Only needed if the team chooses to add GUI-specific aggregation, command validation, or a dedicated status message.

## Phased Implementation

### Phase 0: Establish the ROS interface

- Start the existing launch system and list all active topics.
- Capture the type, QoS, frequency, namespace, and example payload for every GUI topic.
- Identify publishers and subscribers with `ros2 topic info -v`.
- Add or document custom message packages and definitions.
- Define acknowledgement and system-status topics.

Deliverable: a versioned topic/message contract that the GUI can implement against.

### Phase 1: Connect Foxglove without custom code

Install the bridge for ROS 2 Humble:

```bash
sudo apt install ros-humble-foxglove-bridge
```

Run it in a terminal after sourcing ROS and the workspace:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  port:=8765 \
  use_sim_time:=true
```

Connect the Foxglove desktop or web app to:

```text
ws://localhost:8765
```

Create a layout containing:

- Image panels for the glider and wombat feeds.
- Image panels for processed image topics.
- Raw Message or Table panels for weeds, region, trajectory goals, poo, and sonar.
- A 3D panel for poses, TF, maps, and markers when those messages are available.
- A Plot panel for numeric values such as sonar, command velocity, and detection counts.

Deliverable: a read-only layout that proves the bridge and message contracts work.

### Phase 2: Add the custom operator panel

Create a separate TypeScript Foxglove extension project. Keep it separate from the ROS 2 C++ and Python packages so the UI can be built and tested independently.

The first custom panel should:

- Subscribe to the concrete GUI topics.
- Show topic freshness and connection state.
- Show the current selected region and mission state.
- Display a compact weed/detection summary.
- Expose an explicit region-selection workflow.
- Expose guarded movement controls only after the control message contract is confirmed.

Use the Foxglove extension APIs:

- `context.subscribe()` for live topic subscriptions.
- `context.watch("currentFrame")` and `context.onRender` for rendering updates.
- `context.advertise()` followed by `context.publish()` for ROS topic publishing.
- `context.callService()` only if a future command is better represented as a ROS service.
- `context.saveState()` for panel settings such as selected robot or topic names.

Deliverable: a panel that displays real messages and can send one controlled test command.

### Phase 3: Integrate the command workflow

Implement `/region` first because it is a mission-level command with a bounded payload. Add `/wombat/control` after the system has a clear safety and acknowledgement design.

For every command:

1. Validate the input in the panel.
2. Show the selected target and the exact command about to be published.
3. Require an intentional operator action for commands that move a robot.
4. Publish the typed ROS message.
5. Display `sent`, `accepted`, `executing`, `completed`, or `failed` from ROS status data.
6. Time out stale acknowledgements and show the command as unknown rather than successful.

Deliverable: an end-to-end command path with observable acknowledgement.

### Phase 4: Test and harden

- Add deterministic fake publishers for images, detections, region state, and statuses.
- Add a fake Wombat subscriber that records and validates control messages.
- Test missing topics, stale data, malformed messages, reconnects, and duplicate clicks.
- Test with simulation and both single-robot and multi-robot namespaces where applicable.
- Save representative MCAP recordings for repeatable display testing.
- Add the bridge to the project launch flow only after the standalone bridge workflow is reliable.

## Testing Checklist

### ROS checks

```bash
ros2 topic list
ros2 topic type /region
ros2 topic type /wombat/control
ros2 topic type /wombat/lidar
ros2 topic type /glider/camera/image
ros2 topic type /plant_detection/weeds
ros2 topic info /wombat/control -v
ros2 topic hz /glider/camera/image
```

### GUI checks

- The layout opens with no missing topic configuration.
- Both live camera feeds render at the expected rate.
- Processed images retain detection overlays and timestamps.
- Weed detections are shown with position and confidence where available.
- Region selection publishes exactly one correctly typed message.
- Wombat control publishes only when the operator confirms the action.
- Reconnecting Foxglove does not duplicate subscriptions or commands.
- Stale topics are visibly different from healthy topics.

### Acceptance criteria for the first usable version

- An operator can see the glider and wombat camera feeds.
- An operator can inspect processed detections and weed data.
- An operator can identify the current region and mission state.
- A test operator can publish a region to `/region` and verify the Mission node receives it.
- A test operator can publish a bounded Wombat control command and verify the Wombat node receives it.
- The GUI displays the resulting acknowledgement or clearly reports that no acknowledgement arrived.

## Risks and Boundaries

- Foxglove cannot compensate for unclear ROS message contracts. Define those first.
- Image and detection bandwidth may require compressed transport or bridge QoS tuning.
- `/wombat/control` can move a real or simulated robot, so it needs explicit limits and an emergency-stop strategy outside the GUI.
- A wildcard topic such as `/**/processed_image` is a planning shorthand, not a topic that a publisher can use directly.
- The root `CMakeLists.txt` currently contains older PFMS-style targets and should be treated as existing build debt while the GUI integration is added. Avoid coupling the first UI prototype to unrelated build cleanup.
