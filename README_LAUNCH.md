# PyroGuard Launch Files

This package contains launch files for running TurtleBot3 with DQN agent in a forest simulation environment.

## Launch Files

### 1. turtlebot4_forest_world.launch.py
Launches TurtleBot4 in the forest.sdf using Ignition Gazebo with integrated fire management system.

**Usage:**
```bash
ros2 launch pyroguard turtlebot4_forest_world.launch.py
```

**Parameters:**
- `use_sim_time`: Use simulation clock (default: true)
- `x_pose`: Initial x position (default: 0.0 - center for visibility)
- `y_pose`: Initial y position (default: 0.0 - center for visibility)
- `z_pose`: Initial z position (default: 0.3)
- `yaw`: Initial yaw orientation (default: 0.0)
- `world`: Ignition world to load (default: forest)
- `model`: TurtleBot4 model [standard, lite] (default: lite)
- `nav2`: Enable navigation stack (default: false)
- `slam`: Enable SLAM (default: false)
- `rviz`: Start RViz visualization (default: false)

**Example with parameters:**
```bash
ros2 launch pyroguard turtlebot4_forest_world.launch.py x_pose:=5.0 y_pose:=3.0 yaw:=1.57 rviz:=true
```

### 2. dqn_forest_simulation.launch.py
Launches the complete simulation environment with TurtleBot4 in forest.sdf and runs the DQN agent.

**Usage:**
```bash
ros2 launch pyroguard dqn_forest_simulation.launch.py
```

**Parameters:**
All parameters from turtlebot4_forest_world.launch.py plus:
- `obs_size`: Size of observation space for DQN (default: 8)
- `action_size`: Size of action space for DQN (default: 5)
- `model_path`: Path to trained DQN model (default: empty - uses latest model)
- `epsilon`: Epsilon value for exploration (default: 0.1)

**Example with DQN parameters:**
```bash
ros2 launch pyroguard dqn_forest_simulation.launch.py model_path:=/path/to/your/model.pth epsilon:=0.05
```

## Prerequisites

1. Make sure you have the custom gazebo folder set up with the forest.world file at:
   `/home/adji714/custom_gazebo_worlds/forest/forest.world`

2. Ensure you have trained DQN models in the models directory:
   `/home/adji714/ros2_ws/src/pyroguard/models/`

3. Required ROS2 packages:
   - turtlebot3_gazebo
   - turtlebot3_description
   - ros_ign_bridge (for Ignition Gazebo integration)
   - robot_state_publisher
   - rviz2

4. Ignition Gazebo must be installed:
   ```bash
   sudo apt install ignition-gazebo6
   ```

## Fire Management

The package now includes comprehensive fire management capabilities:

### Quick Fire Operations
```bash
# List current fires
python3 fire_utils.py list

# Add 5 strategic fires
sudo python3 fire_utils.py add 5

# Remove all fires
sudo python3 fire_utils.py remove

# Show fire management help
python3 fire_utils.py help
```

### Advanced Fire Management
```bash
# Add realistic fire patterns
sudo python3 fire_manager.py --action add --type realistic --areas 3

# Add strategic fires
sudo python3 fire_manager.py --action add --type strategic --fires 6

# Remove all fires
sudo python3 fire_manager.py --action remove

# List all fires with positions
python3 fire_manager.py --action list
```

## Troubleshooting

### Common Warnings and Errors

1. **ros_ign_bridge library errors**
   ```
   Error while loading the library [/opt/ros/humble/lib/ros_ign_bridge]: cannot read file data: Is a directory
   ```
   - **Status**: Warning only, simulation works normally
   - **Cause**: ROS2 Humble bridge compatibility issue
   - **Action**: No action needed, this doesn't affect functionality

2. **GUI camera conversion warning**
   ```
   [Wrn] [Conversions.cc:722] <gui><camera> can't be converted yet
   ```
   - **Status**: Warning only, camera topics work normally
   - **Cause**: Ignition Gazebo GUI configuration
   - **Action**: No action needed, robot camera functions properly

3. **RPLIDAR/Camera cycling issues**
   - **Symptoms**: Sensors stop and restart periodically
   - **Workaround**: Launch with `rviz:=true` for better sensor monitoring
   - **Robot visibility**: Robot positioned at center (0,0) for better visibility

### Verification Steps

1. **Check if simulation loaded successfully**:
   ```bash
   ros2 topic list | grep -E "(scan|image|odom)"
   ```
   Expected output: `/scan`, `/oakd/rgb/preview/image_raw`, `/odom`

2. **Verify robot topics are active**:
   ```bash
   ros2 topic info /scan
   ros2 topic info /oakd/rgb/preview/image_raw
   ```
   Should show "Publisher count: 3" (normal for TurtleBot4)

3. **Test robot movement**:
   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
   ```

4. **Check fire status**:
   ```bash
   python3 fire_utils.py list
   ```
   Should show current fire locations in the world

### Performance Tips

1. **For training**: Use fires strategically placed for consistent scenarios
2. **For testing**: Remove fires to isolate navigation issues
3. **For development**: Use `rviz:=true` to visualize sensor data

## File Structure

The launch files expect this directory structure:
```
/home/adji714/
├── ros2_ws/src/
│   └── pyroguard/
│       ├── launch/
│       │   ├── turtlebot4_forest_world.launch.py
│       │   ├── dqn_forest_simulation.launch.py
│       │   └── dqn_agent.launch.py
│       ├── models/
│       │   └── (your trained DQN models)
│       ├── fire_manager.py          # Advanced fire management
│       ├── fire_utils.py            # Quick fire operations
│       ├── fire_2d.sdf             # Fire model template
│       └── fire.png                # Fire texture
└── TurtleBot4 system files:
    └── /opt/ros/humble/share/turtlebot4_ignition_bringup/worlds/
        └── forest.sdf               # Enhanced with fire support
```

## Notes

- **Fire Management**: Use the integrated fire management system to add/remove fires dynamically
- **Robot Position**: Robot spawns at center (0,0) for optimal fire detection training
- **Sensor Access**: Camera at `/oakd/rgb/preview/image_raw`, LIDAR at `/scan`
- **Warnings**: `ros_ign_bridge` and GUI conversion warnings are normal and don't affect functionality
- **Build Command**: `colcon build --packages-select pyroguard && source install/setup.bash`
- **Fire Safety**: Always backup world files before fire operations (automatic with fire_manager.py)
