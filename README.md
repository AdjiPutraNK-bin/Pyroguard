
# Pyroguard

Autonomous Forest Fire Fighting Robot (ROS2, Deep RL, YOLO, Segmentation)


## Project Overview
Pyroguard is a ROS2-based system for simulating and developing a TurtleBot4 robot that detects, localizes, and extinguishes forest fires using:
- **Deep Reinforcement Learning (DQN)** for navigation and automatic fire suppression
- **YOLOv8** for robust fire detection
- **Segmentation models** for precise fire area identification
- **Dynamic fire management** (add/remove/list fires in simulation)


## Directory Structure

```
pyroguard/
├── README.md
├── fire_manager/
│   ├── fire_positions.txt
│   └── fix_fires.py
├── launch/
│   ├── dqn_agent.launch.py
│   ├── turtlebot4_forest_world.launch.py
│   └── turtlebot4_forest_world.launch.py.backup
├── models/
│   ├── best.pt
│   └── pytorch_deeplabv3_fire_segmentation .pth
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
│       ├── fire_node.py
│       ├── fire_suppression_handler.py
│       ├── image_preprocessor.py
│       ├── lidar_vla_processor.py
│       ├── map_coverage_node.py
│       ├── reward_publisher.py
│       ├── train.py
│       └── Flame Segmentation.ipynb
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── tree_manager/
│   ├── tree.py
│   ├── tree_positions.py
│   └── tree_positions.txt
└── worlds/
  ├── forest.sdf
  └── textures/
    ├── GrassTerrain.dae
    ├── fire.png
    ├── grass.jpg
    ├── grass_plane.dae
    ├── grass_seamless.jpg
    ├── tree_simple_treetop.jpg
    └── tree_simple_trunk.jpg
```


## Launch & Node Interactions

### 1. Launch the simulation world
```bash
ros2 launch pyroguard turtlebot4_forest_world.launch.py
```
Parameters: initial robot pose, model type, navigation, SLAM, RViz options.

### 2. Launch the DQN agent
```bash
ros2 launch pyroguard dqn_agent.launch.py
```
Parameters: model path, observation size, action size, epsilon, etc.

### 3. Node startup order (recommended)
```bash
source ~/ros2_ws/install/setup.bash
ros2 run pyroguard image_preprocessor.py
ros2 run pyroguard fire_node.py
ros2 run pyroguard lidar_vla_processor.py
ros2 run pyroguard fire_suppression_handler.py
ros2 run pyroguard reward_publisher.py
ros2 run pyroguard map_coverage_node.py
ros2 run pyroguard dqn_agent_node --ros-args -p mode:=collect
```

### 4. System logic
- **Fire detection**: YOLOv8, publishes fire pose and bounding box.
- **Suppression**: Automatic when robot is within 1.2m of fire *and* in suppression mode (locked on fire).
- **Suppression mode**: RL agent publishes `/suppression_mode` (True/False) to coordinate suppression.
- **Suppressed fires**: Masked from detection and visualization in all nodes.
- **Reward logic**: Updated to use suppression event and 1.2m threshold.


## Fire Management

- **Quick operations:**
  - `python3 fire_manager/fix_fires.py add 5` — Add 5 fires
  - `python3 fire_manager/fix_fires.py remove` — Remove all fires
  - `python3 fire_manager/fix_fires.py list` — List all fires
- **Fire positions:**
  - Fire locations are tracked in `fire_manager/fire_positions.txt`.


## Prerequisites

- ROS2 Humble, Ignition Gazebo, ros_gz_bridge
- TurtleBot4 packages
- Trained DQN models in `models/`
- Custom world and textures in `worlds/` and `worlds/textures/`


## Notes

- Use your local `worlds/` directory for custom fire scenarios.
- All fire management is dynamic—no need to manually edit SDF files.
- Suppression is fully automatic and coordinated by RL agent and handler node.
- Suppression/masking threshold is 1.2 meters throughout the system.
- For best results, visualize with RViz and monitor `/scan`, camera, and `/suppression_mode` topics.

---
For more details, see the code and launch files in each subdirectory.


