# Foxglove web setup

Phase 2 provides the `rs1-region-selection` custom Foxglove panel. It is
intentionally a separate TypeScript project in
[`foxglove-extension/`](../foxglove-extension/), rather than part of the ROS 2
package.

## 1. Start ROS 2 and the bridge

In the repository root, use separate terminals:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rs1 --symlink-install
source install/setup.bash
ros2 launch rs1 rs1_demo.launch.py
```

```bash
source /opt/ros/humble/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  port:=8765 use_sim_time:=true
```

Verify the region contract before opening the panel:

```bash
ros2 topic type /region
# geometry_msgs/msg/PoseArray
```

The bridge is a WebSocket server. Keep it on localhost for development
(`ws://localhost:8765`); do not expose an unauthenticated bridge to an
untrusted network.

## 2. Build the extension

Install Node.js 18 or newer. On Ubuntu, `nvm` is a convenient way to install
the required version without changing the system Node.js packages:

```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.3/install.sh | bash
source ~/.bashrc
nvm install 20
nvm use 20
```

Check that both commands are available:

```bash
node --version
npm --version
```

The Node.js version should be 18 or newer. If `npm` is still not found, open a
new terminal and run `nvm use 20` again. Then run:

```bash
cd foxglove-extension
npm install
npm run build
```

make sure foxglvoe desktop is installed
```
https://foxglove.dev/download
```

For Foxglove Desktop, install the extension into the local application:

```bash
npm run local-install
```

The extension package must include `CHANGELOG.md`; it is already included in
the repository. If an older checkout reports
`Missing required file .../CHANGELOG.md`, update the checkout and rerun
`npm run local-install`.

Reload Foxglove Desktop, add a panel, and choose **RS1 region selection**.

To package it for another desktop installation:

```bash
npm run package
```

This creates a `.foxe` file. In Foxglove Desktop, open the extensions manager
and install that file.

## 3. Connect Foxglove Web

Open [Foxglove](https://app.foxglove.dev/), create or open a layout, and add
the **RS1 region selection** panel after the extension is available to the
client. Connect the layout to:

```text
ws://localhost:8765
```

If the web client cannot load a locally developed extension, use Foxglove
Desktop for extension development and validation. The bridge connection and
standard panels work in either client; custom extension availability depends on
the client's extension installation and organization policy.

## run foxglove desktop

## 4. Select and publish a region

1. Set **Global frame** to the mission node's `global_frame` parameter
   (normally `map`).
2. Keep **Region frame** equal to that value.
3. Keep **Topic** as `/region` unless the mission launch configuration changes
   it.
4. Click at least three vertices in the map area. The area represents
   `[-10, 10]` metres on each axis, with the origin at its centre.
5. Review the numbered vertices and polygon geometry.
6. Click **Publish region** once.
7. Confirm that **Last published** is populated and **Last observed** changes
   when `/region` is received back through the bridge.

The panel rejects fewer than three vertices, zero-area polygons, self-crossing
polygons, and a frame different from the configured global frame. It publishes
one `geometry_msgs/msg/PoseArray` containing zero-z poses with identity
orientation. Publication is not mission acceptance: the panel only reports
that it published and later observed the topic.

## Troubleshooting

- `ros2 topic type /region` must report `geometry_msgs/msg/PoseArray`.
- If the panel remains at **waiting**, check the bridge terminal and confirm
  that `/region` appears in the Foxglove topic list.
- If the mission node rejects the region, compare the panel frame with
  `global_frame` in [`config/mission_params.yaml`](../config/mission_params.yaml).
- If `npm run build` fails, use Node.js 18+ and confirm that the shell is in
  the intended checkout. In particular, if the error says
  `TS5083: Cannot read file .../@foxglove/tsconfig/tsconfig.json`, check that
  the build is not using an older checkout or stale `node_modules`:

  ```bash
  cd foxglove-extension
  pwd
  test -f package.json
  test -f tsconfig.json
  grep -n '"@foxglove/tsconfig"' package.json
  npm install
  npm run build
  ```

  The extension's `tsconfig.json` is self-contained and no longer requires
  `@foxglove/tsconfig`. If the old path is still shown, remove only
  `foxglove-extension/node_modules` and `foxglove-extension/package-lock.json`,
  then run `npm install` again.

- If the build reports `TypeScript emitted no output`, check that the
  extension's `tsconfig.json` does not contain `"noEmit": true`. Foxglove's
  webpack build needs TypeScript to emit intermediate output. Also ensure the
  panel's `context.onRender` callback does not force an older `RenderState`
  type; let the installed `@foxglove/extension` SDK infer that callback type.

- If the build reports `Can't resolve 'style-loader'`, run `npm install` from
  `foxglove-extension/` again. The custom panel imports CSS and requires the
  `style-loader` and `css-loader` development dependencies.
