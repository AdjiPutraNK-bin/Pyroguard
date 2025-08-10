#!/usr/bin/env python3
"""
Quick test script for TurtleBot4 forest world launch.
This script helps verify that the robot spawns correctly and provides troubleshooting tips.
"""

import subprocess
import time
import os

def check_ros_nodes():
    """Check if ROS2 nodes are running"""
    try:
        result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True, timeout=5)
        nodes = result.stdout.strip().split('\n') if result.stdout.strip() else []
        return nodes
    except:
        return []

def check_topics():
    """Check if key robot topics are available"""
    try:
        result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True, timeout=5)
        topics = result.stdout.strip().split('\n') if result.stdout.strip() else []
        
        key_topics = ['/scan', '/oakd/rgb/preview/image_raw', '/odom', '/cmd_vel']
        found_topics = [topic for topic in key_topics if topic in topics]
        return found_topics, topics
    except:
        return [], []

def main():
    print("🚀 TurtleBot4 Forest World Test")
    print("=" * 40)
    
    print("\n1. 🔍 Checking current fire status...")
    fire_script = "/home/adji714/ros2_ws/src/pyroguard/fire_utils.py"
    if os.path.exists(fire_script):
        try:
            subprocess.run(['python3', fire_script, 'list'], timeout=10)
        except:
            print("Could not check fire status")
    
    print("\n2. 🤖 Checking ROS2 nodes...")
    nodes = check_ros_nodes()
    if nodes:
        robot_nodes = [node for node in nodes if 'turtlebot' in node.lower()]
        if robot_nodes:
            print(f"✅ Found {len(robot_nodes)} TurtleBot nodes:")
            for node in robot_nodes[:5]:  # Show first 5
                print(f"   - {node}")
        else:
            print("⚠️ No TurtleBot nodes found yet")
    else:
        print("❌ No ROS2 nodes detected")
    
    print("\n3. 📡 Checking robot topics...")
    found_topics, all_topics = check_topics()
    if found_topics:
        print(f"✅ Found {len(found_topics)}/4 key robot topics:")
        for topic in found_topics:
            print(f"   - {topic}")
    else:
        print("⚠️ Key robot topics not found yet")
    
    print("\n4. 💡 Robot Visibility Tips:")
    print("   - Robot spawns at center (0, 0) with fires around it")
    print("   - In Gazebo GUI: Use mouse wheel to zoom out")
    print("   - Look for a small circular robot with a black top")
    print("   - Try View → Transparent to see through objects")
    print("   - Use right-click + drag to orbit around the scene")
    
    print("\n5. 🔥 Fire Management Commands:")
    print("   - List fires: python3 fire_utils.py list")
    print("   - Add fires: sudo python3 fire_utils.py add 5")
    print("   - Remove fires: sudo python3 fire_utils.py remove")
    
    print("\n6. 🎮 Test Robot Movement:")
    if '/cmd_vel' in found_topics:
        print("   - ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.2}}\"")
        print("   - ros2 run teleop_twist_keyboard teleop_twist_keyboard")
    else:
        print("   - Wait for /cmd_vel topic to become available")
    
    print("\n✨ Ready for DQN training when robot is visible and responsive!")

if __name__ == "__main__":
    main()
