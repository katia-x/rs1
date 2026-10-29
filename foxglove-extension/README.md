# RS1 Foxglove extension development

This is the maintainer guide for the RS1 custom Foxglove Web panel. It covers
creating, editing, testing, formatting, and packaging the panel as a `.foxe`
file.

For people who only need to use a ready-made extension, see
[`README-user.md`](README-user.md).

The extension publishes one `geometry_msgs/msg/PoseArray` message to `/region`
in the `map` frame by default. It does not report mission acceptance because no
region acknowledgement topic exists.

## Important distinction: panel code versus layout

- The TypeScript and CSS under `src/` define the panel and are packaged into a
  `.foxe` file.
- A Foxglove layout is created and arranged in Foxglove Web. It contains panel
  placement, topic selections, and saved panel state.
- The layout is not compiled into the `.foxe` file. Export or share the layout
  separately if other users need the same arrangement.

## Requirements

- Node.js 18 or newer
- npm
- Foxglove Web
- ROS 2 Humble with `foxglove_bridge` for live testing

If `npm` is unavailable on Ubuntu, install Node.js with `nvm`:

```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.3/install.sh | bash
source ~/.bashrc
nvm install 20
nvm use 20
node --version
npm --version
```

## First-time developer setup

From the extension directory:

```bash
cd ~/git/rs1/foxglove-extension
npm install
```

Start the ROS system and bridge in separate terminals for live testing:

```bash
source /opt/ros/humble/setup.bash
ros2 launch rs1 rs1_demo.launch.py
```

```bash
source /opt/ros/humble/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  port:=8765 use_sim_time:=true
```

Check the ROS contract:

```bash
ros2 topic type /region
# geometry_msgs/msg/PoseArray
```

## Create and edit the panel

The entry point is [`src/index.ts`](src/index.ts). It registers the panel with
Foxglove.

The panel behavior is in
[`src/RegionSelectionPanel.tsx`](src/RegionSelectionPanel.tsx). Update this
file when changing:

- Region selection and validation
- Published message fields
- Topic subscriptions
- Foxglove panel state
- Panel controls and status text

Styles are in [`src/styles.css`](src/styles.css). Keep CSS imports in the
panel source so the webpack build includes them.

When adding dependencies, update `package.json` and run:

```bash
npm install
```

Commit the resulting `package-lock.json` with the manifest change.

## Format, lint, and build

After editing the panel:

```bash
npm run lint
npm run build
```

The build output is placed in `dist/`. It is an intermediate artifact and
should not be edited manually.

The build must complete before packaging. If the build reports a missing CSS
loader, run `npm install`; the panel requires `style-loader` and `css-loader`.

## Test in Foxglove Web

Foxglove Web connects to the local bridge at:

```text
ws://localhost:8765
```

Use the Foxglove Web extension workflow to load the development or packaged
extension. Add the **RS1 region selection** panel to a layout, then configure:

- Global frame: `map`
- Region frame: `map`
- Topic: `/region`

Test that:

- Fewer than three points are rejected.
- A wrong frame is rejected.
- A zero-area polygon is rejected.
- A self-intersecting polygon is rejected.
- A valid polygon publishes one `/region` message.
- **Last observed** updates when `/region` is received.
- Reconnecting or reopening the panel does not republish a previous region.

## Create and format a Foxglove layout

After the extension is installed in Foxglove Web:

1. Create a new layout, or open the layout you want to edit.
2. Connect it to `ws://localhost:8765`.
3. Add the **RS1 region selection** panel.
4. Add standard panels for the camera, detections, 3D view, laser scan, and
   raw `/region` messages as needed.
5. Resize panels by dragging their borders and reorder them to create the
   operator view.
6. Configure the region panel's global frame, region frame, and topic.
7. Save the layout.
8. Export or share the layout separately when other users need the same panel
   arrangement.

The layout's panel arrangement is not part of the `.foxe` file. The `.foxe`
contains the extension code; the layout contains the workspace arrangement and
panel settings. A release that includes a prepared operator view should
therefore distribute both files.

## Build a `.foxe` file

Ensure [`CHANGELOG.md`](CHANGELOG.md) is present and update it for meaningful
panel changes. Then run:

```bash
npm run package
```

The command runs the production build and creates a `.foxe` file in the
extension directory. The `.foxe` file is the distributable extension package.

Use a new semantic version in `package.json` for a release that is distributed
to other users, and add a matching section to `CHANGELOG.md`. Re-run:

```bash
npm install
npm run lint
npm run package
```

Do not commit `node_modules/`, `dist/`, or generated `.foxe` files unless the
repository's release process explicitly requires them.

## Distribute the extension and layout

For Foxglove Web users:

1. Provide the generated `.foxe` file.
2. Provide an exported Foxglove layout separately if a pre-arranged layout is
   required.
3. Tell users to drag the `.foxe` file into the Foxglove Web window.
4. Tell users to import or open the layout after the extension is loaded.

The bridge still needs to be running at `ws://localhost:8765` for local ROS
testing.

## Troubleshooting

If the build mentions an old `@foxglove/tsconfig` path, confirm that the
symlinked checkout is the one being used:

```bash
pwd
readlink -f .
test -f package.json
test -f tsconfig.json
```

The extension has a self-contained `tsconfig.json`. If an old dependency tree
is being used, refresh only this extension:

```bash
rm -rf node_modules package-lock.json
npm install
npm run build
```

Do not run that removal command from the repository root.
