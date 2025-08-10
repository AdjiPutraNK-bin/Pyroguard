# Fire Management System Documentation

## Overview
This package now includes a comprehensive fire management system that integrates the logic from the original `fix_fires.py` and `remove_fire_effects.py` scripts. The system allows you to dynamically add and remove fires in the TurtleBot4 forest world for training fire detection and navigation algorithms.

## Features

### 🔥 Fire Management Tools

1. **`fire_manager.py`** - Comprehensive fire management system
   - Add strategic fires (simple placement)
   - Add realistic fires (complex fire patterns with ground, trunk, and canopy fires)
   - Remove all fires
   - List current fires
   - Backup and restore world files
   - Original fix_fires.py logic for realistic fire behavior

2. **`fire_utils.py`** - Simple utility script for quick operations
   - Quick add/remove fires
   - Easy command-line interface
   - Perfect for testing and development

### 🚀 Launch Files

1. **`turtlebot4_forest_world.launch.py`** - Main launch file
   - Launches TurtleBot4 in forest world with fire management support
   - Robot positioned at center (0,0) for better visibility
   - Full argument support for positioning and configuration

### 🛠 Fire Types and Patterns

The system supports multiple fire patterns based on the original fix_fires.py logic:

#### Strategic Fires (Simple)
- **Ground Only**: Single ground-level fires
- **Ground Cluster**: Multiple fires in a cluster formation
- **Vertical Spread**: Fires at different heights (ground, trunk, canopy)

#### Realistic Fires (Complex)
- **Ground Only**: Multiple ground fires with random positioning
- **Ground to Tree**: Fire spread chains from ground to canopy
- **Tree Cluster**: Complete fire scenarios with ground fires, trunk fires, and canopy fires

## Usage Guide

### Quick Start
```bash
# List current fires
python3 fire_utils.py list

# Add 5 strategic fires
sudo python3 fire_utils.py add 5

# Remove all fires  
sudo python3 fire_utils.py remove

# Show help
python3 fire_utils.py help
```

### Advanced Usage
```bash
# Add strategic fires
sudo python3 fire_manager.py --action add --type strategic --fires 6

# Add realistic fires
sudo python3 fire_manager.py --action add --type realistic --areas 3

# Remove all fires
sudo python3 fire_manager.py --action remove

# List fires
python3 fire_manager.py --action list

# Create backup
sudo python3 fire_manager.py --action backup

# Restore from backup
sudo python3 fire_manager.py --action restore
```

### Launch TurtleBot4 in Fire-Enabled Forest
```bash
# Basic launch
ros2 launch pyroguard turtlebot4_forest_world.launch.py

# Launch with custom robot position
ros2 launch pyroguard turtlebot4_forest_world.launch.py x_pose:=5.0 y_pose:=3.0

# Launch with navigation enabled
ros2 launch pyroguard turtlebot4_forest_world.launch.py nav2:=true

# Show available arguments
ros2 launch pyroguard turtlebot4_forest_world.launch.py --show-args
```

## Fire System Features

### Original Logic Integration
- **Fire Spread Chains**: Realistic fire progression from ground to tree canopy
- **Multiple Fire Types**: Ground, trunk, and canopy fires with different visual properties
- **Lighting Effects**: Point lights for each fire with realistic attenuation
- **Visual Layers**: Multiple visual elements per fire for realistic appearance
- **Unique Naming**: Automatic fire ID generation to avoid conflicts

### Safety Features
- **Automatic Backup**: World file backed up before any modifications
- **Restore Capability**: Easy restoration from backup if needed
- **Error Handling**: Comprehensive error checking and reporting
- **Permission Management**: Clear indication when sudo privileges are needed

### Strategic Fire Placement
Fires are placed at strategic locations for training:
- Northeast area (5.0, 5.0)
- Northwest area (-3.0, 8.0) 
- Far northeast (15.0, 10.0)
- Southwest area (-8.0, -5.0)
- Far southeast (20.0, -8.0)
- North area (0.0, 15.0)
- And more based on configuration

## Integration with DQN Training

The fire management system is designed to work seamlessly with DQN training:

1. **Dynamic Fire Scenarios**: Add/remove fires between training episodes
2. **Consistent Placement**: Strategic fire placement for reproducible training
3. **Visual Feedback**: Realistic fire appearance for camera-based detection
4. **Performance Optimized**: Efficient fire models that don't impact simulation performance

## Robot Configuration

### Improved Visibility
- Robot spawned at center position (0,0) instead of far corner
- Better camera angle for fire detection
- Central position provides 360-degree fire detection opportunities

### Sensor Access
- RPLIDAR for obstacle avoidance around fires
- Camera for visual fire detection: `/oakd/rgb/preview/image_raw`
- Odometry for navigation and positioning

## File Structure
```
pyroguard/
├── fire_manager.py          # Main fire management system
├── fire_utils.py            # Simple utility functions
├── add_fires_to_turtlebot_world.py  # Legacy script (preserved)
├── fix_fires.py             # Original fire generation script
├── remove_fire_effects.py   # Original fire removal script
├── fire_2d.sdf             # Fire model template
├── fire.png                # Fire texture
└── launch/
    └── turtlebot4_forest_world.launch.py  # Main launch file
```

## Notes

### Permissions
- Adding/removing fires requires sudo privileges (modifies system files)
- Listing fires does not require sudo
- Backup/restore operations require sudo

### World File Location
- Default: `/opt/ros/humble/share/turtlebot4_ignition_bringup/worlds/forest.sdf`
- Automatic backup creation before modifications
- Backup file: `forest.sdf.backup`

### Future Enhancements
- Tree position detection for even more realistic fire placement
- Dynamic fire spread simulation during runtime
- Integration with fire behavior models
- Multiple world support

This fire management system provides everything needed for realistic fire detection training scenarios while maintaining the original sophisticated fire generation logic from your custom world.
