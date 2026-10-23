If you get *"Detected jump back in time. Clearing TF buffer."* try clearing and remaining Gazebo clock bridges with:
```bash
pkill -INT -f '[p]arameter_bridge.*__node:=ros_gz_bridge_clock'
```