# Pyroguard

Autonomous Forest Fire Fighting Robot (ROS2, Reinforcement Learning, VLA, Segmentation)

## Project Overview
Pyroguard is a ROS2-based system for simulating and developing a TurtleBot4 robot that detects, localizes, and extinguishes forest fires using:
- **Deep Reinforcement Learning (DQN)** for navigation and fire suppression
- **Visual-Lidar Attention (VLA)** for robust fire detection
- **Segmentation models** for precise fire area identification
- **Dynamic fire management** (add/remove/list fires in simulation)

## Directory Structure

```
pyroguard/
├── README.md
├── fire_manager/
│   ├── fire_2d.sdf
│   ├── fix_fires.py
│   └── remove_fire_effects.py
├── launch/
│   ├── dqn_agent.launch.py
│   ├── turtlebot4_forest_world.launch.py
│   └── turtlebot4_forest_world.launch.py.backup
├── models/                # (trained DQN models go here)
├── package.xml
├── pyroguard/
│   └── __init__.py
├── setup.cfg
├── setup.py
├── src/
│   └── pyroguard/
│       ├── __init__.py
│       ├── dqn_agent.py
│       ├── dqn_agent_node.py
│       ├── fire_segmentation.py
│       ├── fire_suppression_handler.py
│       ├── image_preprocessor.py
│       ├── lidar_vla_processor.py
│       └── reward_publisher.py
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── worlds/
    └── textures/
        ├── dirt_diffusespecular.png
        ├── fire.png
        ├── grass.jpg
        ├── grass_plane.dae
        ├── grass_seamless.jpg
        ├── tree_simple_treetop.jpg
        └── tree_simple_trunk.jpg
```

## Launch Files

### 1. `turtlebot4_forest_world.launch.py`
Launches TurtleBot4 in a forest world with integrated fire management.

**Usage:**
```bash
ros2 launch pyroguard turtlebot4_forest_world.launch.py
```
**Parameters:**
- `use_sim_time`, `x_pose`, `y_pose`, `z_pose`, `yaw`: Initial robot pose
- `model`: TurtleBot4 model [standard, lite]
- `nav2`, `slam`, `rviz`: Enable navigation, SLAM, or RViz

### 2. `dqn_agent.launch.py`
Launches the DQN agent node for autonomous fire fighting.

**Usage:**
```bash
ros2 launch pyroguard dqn_agent.launch.py
```
**Parameters:**
- `model_path`: Path to trained DQN model
- `obs_size`, `action_size`, `epsilon`: DQN hyperparameters

## Fire Management

- **Quick operations:**
  - `python3 fire_manager/fix_fires.py add 5` — Add 5 fires
  - `python3 fire_manager/fix_fires.py remove` — Remove all fires
  - `python3 fire_manager/fix_fires.py list` — List all fires

- **Advanced:**
  - `python3 fire_manager/fire_manager.py --action add --type strategic --fires 6`
  - `python3 fire_manager/fire_manager.py --action remove`
  - `python3 fire_manager/fire_manager.py --action list`

## Prerequisites

- ROS2 Humble, Ignition Gazebo, ros_gz_bridge
- TurtleBot4 packages
- Trained DQN models in `models/`
- Custom world and textures in `worlds/` and `worlds/textures/`

## Notes

- Use your local `worlds/` directory for custom fire scenarios.
- All fire management is dynamic—no need to manually edit SDF files.
- For best results, visualize with RViz and monitor `/scan` and camera topics.

---
For more details, see the code and launch files in each subdirectory.
