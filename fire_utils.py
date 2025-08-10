#!/usr/bin/env python3
"""
Quick fire management utilities for TurtleBot4 forest world.
This script provides convenient functions for adding and removing fires.
"""

import os
import sys
sys.path.append(os.path.dirname(__file__))

from fire_manager import FireManager

def quick_add_fires(num_fires=6):
    """Quickly add strategic fires to TurtleBot4 world"""
    world_file = '/opt/ros/humble/share/turtlebot4_ignition_bringup/worlds/forest.sdf'
    manager = FireManager(world_file)
    
    print(f"🔥 Adding {num_fires} strategic fires to TurtleBot4 forest world...")
    manager.create_backup()
    
    if manager.add_strategic_fires(num_fires):
        print("✅ Fires added successfully!")
        manager.list_fires()
        return True
    else:
        print("❌ Failed to add fires")
        return False

def quick_remove_fires():
    """Quickly remove all fires from TurtleBot4 world"""
    world_file = '/opt/ros/humble/share/turtlebot4_ignition_bringup/worlds/forest.sdf'
    manager = FireManager(world_file)
    
    print("🧯 Removing all fires from TurtleBot4 forest world...")
    manager.create_backup()
    
    if manager.remove_all_fires():
        print("✅ All fires removed successfully!")
        return True
    else:
        print("❌ Failed to remove fires")
        return False

def show_fires():
    """Show current fires in TurtleBot4 world"""
    world_file = '/opt/ros/humble/share/turtlebot4_ignition_bringup/worlds/forest.sdf'
    manager = FireManager(world_file)
    
    print("🔍 Current fire status in TurtleBot4 forest world:")
    manager.list_fires()

def main():
    """Simple command line interface"""
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 fire_utils.py add [number_of_fires]")
        print("  python3 fire_utils.py remove")
        print("  python3 fire_utils.py list")
        print("  python3 fire_utils.py help")
        print("\nNote: Adding/removing fires requires sudo privileges")
        return
    
    command = sys.argv[1].lower()
    
    if command == 'add':
        num_fires = int(sys.argv[2]) if len(sys.argv) > 2 else 6
        quick_add_fires(num_fires)
    elif command == 'remove':
        quick_remove_fires()
    elif command == 'list':
        show_fires()
    elif command == 'help':
        print("Fire Management Utilities for TurtleBot4 Forest World")
        print("=" * 55)
        print()
        print("Commands:")
        print("  add [N]    - Add N strategic fires (default: 6)")
        print("  remove     - Remove all fires")
        print("  list       - List current fires")
        print("  help       - Show this help")
        print()
        print("Examples:")
        print("  sudo python3 fire_utils.py add 4     # Add 4 fires")
        print("  sudo python3 fire_utils.py remove    # Remove all fires")
        print("  python3 fire_utils.py list           # List fires (no sudo needed)")
        print()
        print("Note: Adding/removing fires requires sudo privileges")
        print("      Listing fires does not require sudo")
    else:
        print(f"Unknown command: {command}")
        print("Use 'python3 fire_utils.py help' for usage information")

if __name__ == "__main__":
    main()
