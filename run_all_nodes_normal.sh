#!/bin/bash
# Run all Pyroguard nodes in separate normal terminal windows
# Usage: bash run_all_nodes_normal.sh

# Each command to run in a separate terminal
NODES=(
  "source ~/.bashrc && source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run pyroguard image_preprocessor_node"
  "source ~/.bashrc && source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run pyroguard fire_node"
  "source ~/.bashrc && source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run pyroguard lidar_vla_processor_node"
  "source ~/.bashrc && source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run pyroguard fire_suppression_handler_node"
  "source ~/.bashrc && source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run pyroguard reward_publisher_node"
  "source ~/.bashrc && source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run pyroguard map_coverage_node"
  "source ~/.bashrc && source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run pyroguard dqn_agent_node"
  "source ~/.bashrc && source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run slam_toolbox async_slam_toolbox_node"
)

# Use gnome-terminal for normal terminals (change to xterm, konsole, etc. if needed)
TERMINAL_CMD=$(command -v gnome-terminal)
if [ -z "$TERMINAL_CMD" ]; then
  echo "gnome-terminal is not installed. Please install it or change TERMINAL_CMD in this script."
  exit 1
fi

for CMD in "${NODES[@]}"; do
  $TERMINAL_CMD -- bash -c "$CMD; exec bash" &
done
