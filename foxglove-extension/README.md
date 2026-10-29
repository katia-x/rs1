# RS1 Foxglove region-selection extension

This folder contains the custom Foxglove panel for selecting and publishing a
mission region. It is a separate TypeScript project and is not built by the
ROS 2 `colcon` build.

The panel publishes one `geometry_msgs/msg/PoseArray` message to `/region`.
The default frame is `map`. It only reports that the message was published and
later observed; it does not report mission acceptance.

## Requirements

- Node.js 18 or newer
- npm
- Foxglove Desktop for local extension development, or an installation of the
  packaged extension supported by your Foxglove Web organization
- A running ROS 2 system with `foxglove_bridge`

The bridge can be installed on ROS 2 Humble with:

```bash
sudo apt update
sudo apt install ros-humble-foxglove-bridge
```

### Install Node.js and npm

If `npm: command not found` is displayed, install Node.js 18 or newer. The
recommended Ubuntu setup uses `nvm`, which keeps the extension's Node.js
version separate from system packages:

```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.3/install.sh | bash
source ~/.bashrc
nvm install 20
nvm use 20
```

Verify the installation:

```bash
node --version
npm --version
```

If either command is still unavailable, close and reopen the terminal, then
run `nvm use 20`. The `nvm` command must be run in every new shell unless the
installer has added its initialization to your shell startup file.

## First-time setup

From the repository root, build and launch the ROS 2 system in one terminal:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rs1 --symlink-install
source install/setup.bash
ros2 launch rs1 rs1_demo.launch.py
```

Start the Foxglove bridge in another terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  port:=8765 use_sim_time:=true
```

Verify that the topic has the expected type:

```bash
ros2 topic type /region
# geometry_msgs/msg/PoseArray
```

Then install the extension dependencies:

```bash
cd foxglove-extension
npm install
```

Build and install the extension into Foxglove Desktop:

```bash
npm run build
npm run local-install
```

Reload Foxglove Desktop. Add a panel and choose **RS1 region selection**.
Connect the layout to:

```text
ws://localhost:8765
```

Set the panel's global and region frames to the mission node's
`global_frame` value, normally `map`.

## Running it after setup

Each time you use the GUI:

1. Start the ROS 2 simulation or robot system.
2. Start `foxglove_bridge` on port `8765`.
3. Open Foxglove Desktop and use the saved layout containing **RS1 region
   selection**.
4. Confirm the layout is connected to `ws://localhost:8765`.

You do not need to run `npm install` again unless dependencies change or
`node_modules` has been removed. You do not need to run `npm run build` again
unless the extension source has changed.

If Foxglove does not show the panel after restarting, run:

```bash
npm run local-install
```

Then reload Foxglove Desktop.

The first `npm run local-install` may fail if the extension has no
`CHANGELOG.md`. This file is required by Foxglove's packaging tool and is
included in this project. After pulling this file, rerun:

```bash
npm run local-install
```

If `npm run build` reports a missing `@foxglove/tsconfig/tsconfig.json`, make
sure you are building the same checkout where you installed the dependencies.
The extension now uses a self-contained `tsconfig.json`, so this error usually
means that an older checkout or an old `node_modules` tree is being used.
Check the paths before reinstalling:

```bash
pwd
test -f package.json
test -f tsconfig.json
grep -n '"@foxglove/tsconfig"' package.json
```

`pwd` should be the extension directory you intend to use, and the final
command should produce no output. Then refresh dependencies and build:

```bash
npm install
npm run build
```

If the build still mentions `@foxglove/tsconfig`, remove only that checkout's
installed dependencies and lockfile, then install again:

```bash
rm -rf node_modules package-lock.json
npm install
npm run build
```

Do not run those removal commands from the repository root. The
The extension's TypeScript settings are kept in its local `tsconfig.json`, so
no separate shared TypeScript configuration package is required.

If the build reports `TypeScript emitted no output`, make sure your checkout
contains the current `tsconfig.json`. It must not set `noEmit` to `true`;
Foxglove's webpack bundler needs TypeScript to emit the intermediate output.

If the build reports `Can't resolve 'style-loader'`, update the dependencies
from the extension directory. The panel imports `src/styles.css`, which
requires both `style-loader` and `css-loader`:

```bash
npm install
npm run build
```

## Making and testing changes

Edit the TypeScript or CSS files under `src/`. The main entry point is
[`src/index.ts`](src/index.ts), which registers the panel. The panel UI and
region-publishing behavior are in
[`src/RegionSelectionPanel.tsx`](src/RegionSelectionPanel.tsx), and styling is
in [`src/styles.css`](src/styles.css).

After changing the extension:

```bash
npm run build
npm run lint
npm run local-install
```

Reload Foxglove Desktop to load the new build. Reconnect to the bridge if
necessary, then verify:

- The panel opens without errors.
- At least three vertices are required.
- Wrong frames, zero-area polygons, and self-crossing polygons are rejected.
- A valid region publishes exactly one `/region` message.
- **Last observed** updates when the topic is received back through the
  bridge.
- Reopening or reconnecting the panel does not republish a previous region.

When changing dependencies, update `package.json` and run:

```bash
npm install
npm run build
```

When changing the published message type or topic, update the panel code,
mission-node contract, and the documentation together. Confirm the new ROS
type with `ros2 topic type` before testing in Foxglove.

## Packaging for another installation

To create a distributable Foxglove extension package:

```bash
npm run package
```

This creates a `.foxe` file in this directory. Install that file through the
Foxglove Desktop extensions manager, or publish it through the appropriate
Foxglove organization workflow. Re-run `npm run package` whenever source or
dependency changes need to be distributed.

For the complete ROS and Foxglove setup, see
[`docs/foxglove-extension.md`](../docs/foxglove-extension.md).
