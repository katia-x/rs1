# RS1 Foxglove Web user guide

This guide is for users who have been given a ready-to-use `.foxe` extension
file. It assumes the extension has already been built by a maintainer. You do
not need Node.js, npm, or the source repository.

The RS1 panel selects a polygon and publishes one
`geometry_msgs/msg/PoseArray` message to `/region`. It does not report mission
acceptance.

## Start ROS 2 and the Foxglove bridge

The ROS system and bridge must be running before the panel can publish:

```bash
source /opt/ros/humble/setup.bash
ros2 launch rs1 rs1_demo.launch.py
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  port:=8765 use_sim_time:=true
```

The bridge endpoint for local Foxglove Web use is:

```text
ws://localhost:8765
```

Verify the topic type if needed:

```bash
ros2 topic type /region
# geometry_msgs/msg/PoseArray
```

## Install the `.foxe` extension in Foxglove Web

1. Open [Foxglove Web](https://app.foxglove.dev/).
2. Open the target Foxglove Web window or organization workspace.
3. Drag the supplied `.foxe` file into the Foxglove Web window.
4. Accept the extension installation prompt if one appears.
5. Reload Foxglove Web if the panel does not immediately appear.

No Foxglove Desktop installation is required for this workflow.

## Open and configure the panel

1. Connect the Foxglove layout to `ws://localhost:8765`.
2. Add the **RS1 region selection** panel to the layout.
3. Set **Global frame** to `map`.
4. Set **Region frame** to `map`.
5. Set **Topic** to `/region`.

If you were given a layout file, import or open it after installing the
extension. The `.foxe` file provides the panel; the layout file provides its
screen arrangement and saved settings.

## Use or adjust the layout

To use a prepared operator layout, import or open the supplied layout after
installing the `.foxe` file. To make your own arrangement:

1. Add the **RS1 region selection** panel.
2. Add any standard Foxglove panels you need.
3. Resize and reorder the panels.
4. Set the region panel's global frame, region frame, and topic.
5. Save the layout.

The layout is separate from the `.foxe` extension. Share or export the layout
separately if another operator needs the same arrangement.

## Publish a region

1. Click at least three vertices in the panel's map area.
2. Review the numbered vertices and polygon preview.
3. Click **Publish region** once.
4. Confirm **Last published** shows the vertex count.
5. Confirm **Last observed** updates when `/region` is received through the
   bridge.

The panel rejects:

- Fewer than three vertices
- A region frame other than the configured global frame
- Zero-area polygons
- Self-intersecting polygons

The panel uses a `[-10, 10]` metre selection area on both axes, with the
origin at its centre. Published poses have `z = 0` and identity orientation.

## Troubleshooting

- **Panel missing:** reload Foxglove Web and confirm the `.foxe` installation
  completed.
- **No connection:** confirm the bridge is running and the layout uses exactly
  `ws://localhost:8765`.
- **Last observed stays at waiting:** check that `/region` is visible in the
  Foxglove topic list and that the ROS type is
  `geometry_msgs/msg/PoseArray`.
- **Mission rejects the region:** confirm both frame fields are `map`.
- **Wombat location is missing or stale:** confirm the TF and Static TF topics
  are `/husky1/tf` and `/husky1/tf_static`, and that **Base frame** is
  `husky1_base_link`.
- **Glider location is missing:** confirm the Glider TF and Static TF topics
  are `/parrot1/tf` and `/parrot1/tf_static`, and that **Glider frame** is
  `parrot1_base_link`.
- **Cannot drag the file:** save the `.foxe` file locally, then drag it from
  the file manager into the Foxglove Web window. If the organization blocks
  extension installation, ask its Foxglove administrator to enable or install
  the extension.
