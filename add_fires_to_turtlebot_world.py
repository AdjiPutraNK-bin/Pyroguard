#!/usr/bin/env python3

import xml.etree.ElementTree as ET
import random
import shutil
import os

def add_fires_to_world():
    """
    Add fire effects to the TurtleBot4 forest world
    """
    
    # Paths
    turtlebot_world_path = '/opt/ros/humble/share/turtlebot4_ignition_bringup/worlds/forest.sdf'
    backup_path = '/opt/ros/humble/share/turtlebot4_ignition_bringup/worlds/forest_no_fires.sdf'
    
    # Create backup if it doesn't exist
    if not os.path.exists(backup_path):
        print("Creating backup of original world file...")
        shutil.copy2(turtlebot_world_path, backup_path)
    
    # Parse the world file
    tree = ET.parse(turtlebot_world_path)
    root = tree.getroot()
    
    # Find the world element
    world = root.find('world')
    if world is None:
        print("Error: Could not find world element in SDF file")
        return
    
    print("Adding fires to TurtleBot4 forest world...")
    
    # Define fire locations (avoid robot spawn area around x=10, y=-2)
    fire_locations = [
        (5.0, 5.0, 0.1, "fire_1"),
        (-3.0, 8.0, 0.1, "fire_2"), 
        (15.0, 10.0, 0.1, "fire_3"),
        (-8.0, -5.0, 0.1, "fire_4"),
        (20.0, -8.0, 0.1, "fire_5"),
        (0.0, 15.0, 0.1, "fire_6"),
    ]
    
    # Remove existing fires first
    for model in world.findall('model'):
        if model.get('name') and 'fire' in model.get('name'):
            world.remove(model)
    
    # Add fire models
    for x, y, z, name in fire_locations:
        fire_model = ET.SubElement(world, 'model')
        fire_model.set('name', name)
        
        # Pose
        pose = ET.SubElement(fire_model, 'pose')
        pose.text = f"{x} {y} {z} 0 0 0"
        
        # Link
        link = ET.SubElement(fire_model, 'link')
        link.set('name', 'fire_link')
        
        # Visual
        visual = ET.SubElement(link, 'visual')
        visual.set('name', 'fire_visual')
        
        # Geometry
        geometry = ET.SubElement(visual, 'geometry')
        plane = ET.SubElement(geometry, 'plane')
        normal = ET.SubElement(plane, 'normal')
        normal.text = "0 0 1"
        size = ET.SubElement(plane, 'size')
        size.text = "2 2"
        
        # Material
        material = ET.SubElement(visual, 'material')
        script = ET.SubElement(material, 'script')
        uri = ET.SubElement(script, 'uri')
        uri.text = "file://media/materials/scripts/gazebo.material"
        name_elem = ET.SubElement(script, 'name')
        name_elem.text = "Gazebo/Flames"
        
        # Plugin for particle effects
        plugin = ET.SubElement(fire_model, 'plugin')
        plugin.set('name', f'{name}_particle_emitter')
        plugin.set('filename', 'libignition-gazebo-particle-emitter-system.so')
        
        # Emitter configuration
        emitter_config = {
            'type': 'point',
            'emitting': 'true',
            'duration': '0',
            'lifetime': '5',
            'rate': '10',
            'scale_rate': '0',
            'min_velocity': '0.1',
            'max_velocity': '0.5',
            'size': '1.0 1.0 1.0',
            'color_start': '1.0 0.3 0.0 1.0',
            'color_end': '1.0 0.1 0.0 0.0',
            'material': 'Gazebo/Flames',
            'pose': f'{x} {y} {z+0.5} 0 0 0'
        }
        
        for key, value in emitter_config.items():
            elem = ET.SubElement(plugin, key)
            elem.text = value
    
    # Save the modified world file
    tree.write(turtlebot_world_path, encoding='utf-8', xml_declaration=True)
    print(f"Successfully added {len(fire_locations)} fires to the world!")
    print("Fire locations added:")
    for x, y, z, name in fire_locations:
        print(f"  {name}: ({x}, {y}, {z})")

def remove_fires_from_world():
    """
    Remove all fires from the TurtleBot4 forest world
    """
    
    turtlebot_world_path = '/opt/ros/humble/share/turtlebot4_ignition_bringup/worlds/forest.sdf'
    
    # Parse the world file
    tree = ET.parse(turtlebot_world_path)
    root = tree.getroot()
    
    # Find the world element
    world = root.find('world')
    if world is None:
        print("Error: Could not find world element in SDF file")
        return
    
    print("Removing fires from TurtleBot4 forest world...")
    
    # Remove fire models
    fires_removed = 0
    for model in world.findall('model'):
        if model.get('name') and 'fire' in model.get('name'):
            world.remove(model)
            fires_removed += 1
    
    # Save the modified world file
    tree.write(turtlebot_world_path, encoding='utf-8', xml_declaration=True)
    print(f"Successfully removed {fires_removed} fires from the world!")

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "remove":
        remove_fires_from_world()
    else:
        add_fires_to_world()
