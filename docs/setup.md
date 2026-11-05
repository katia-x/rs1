to run sim in /git/rs1 run:
source /opt/ros/humble/setup.bash
source ~/git/rs1/install/setup.bash
colcon build --packages-select rs1 --symlink-install
ros2 launch rs1 rs1_demo.launch.py gui:=true


to run foxglove handshake in other terminal same directory:
ros2 launch foxglove_bridge foxglove_bridge_launch.xml   port:=8765 use_sim_time:=true
(maybe source too?)


to make new .doxe in git/rs1/foxglove-extension run:
npm install
npm run package