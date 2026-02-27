# red_rover_sim

Gazebo simulation for the Pioneer 3-AT ground robot (Red Rover) in ROS 2. Includes URDF/xacro model, diff-drive plugin, and an empty world. No C++ code -- install-only package.

## Dependencies

- ROS 2 Humble
- `gazebo_ros_pkgs`, `robot_state_publisher`, `joint_state_publisher`, `xacro`, `rviz2`

## Build

```bash
cd ~/trajectory_generator_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select red_rover_sim
source install/setup.bash
```

## Run

```bash
# With RViz
ros2 launch red_rover_sim simulation.launch.py

# Without RViz
ros2 launch red_rover_sim simulation.launch.py use_rviz:=false
```

### Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_rviz` | `true` | Launch RViz for visualization |
| `world` | `worlds/empty.world` | Gazebo world file path |
| `robot_name` | `$ROVER_NAME` or `RR03` | Namespace for diff-drive plugin topics (`odom`, `cmd_vel`) |

## Topics provided by the diff-drive plugin

All topics are namespaced under `/<robot_name>/`:

| Topic | Type | Description |
|-------|------|-------------|
| `odom` | `nav_msgs/msg/Odometry` | Wheel odometry |
| `cmd_vel` | `geometry_msgs/msg/Twist` | Velocity commands (subscribed) |

The plugin also publishes TF: `odom` -> `base_link`.

## Robot model

- Pioneer 3-AT chassis with 4 wheels
- Diff-drive on back wheels only
- Front wheels are passive with low friction (`mu1=0.1, mu2=0.1`) for clean in-place rotation
- Meshes use `package://red_rover_sim/meshes/...` URIs
