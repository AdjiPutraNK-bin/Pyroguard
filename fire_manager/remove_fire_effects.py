#!/usr/bin/env python3

import xml.etree.ElementTree as ET
import argparse

def remove_fire_effects(world_file):
    """
    Remove all fire effects from the world file
    """
    
    # Parse the existing world file
    tree = ET.parse(world_file)
    root = tree.getroot()
    
    # Find the world element
    world = root.find('world')
    if world is None:
        print("Error: Could not find world element in SDF file")
        return
    
    # Find and remove all fire models
    fire_models = []
    for model in world.findall('model'):
        model_name = model.get('name', '')
        if 'fire' in model_name.lower():
            fire_models.append(model)
    
    for fire_model in fire_models:
        world.remove(fire_model)
        print(f"Removed fire model: {fire_model.get('name')}")
    
    # Write the updated world file
    tree.write(world_file, encoding='utf-8', xml_declaration=True)
    print(f"\nRemoved {len(fire_models)} fire effects from {world_file}")

def main():
    parser = argparse.ArgumentParser(description='Remove all fire effects from forest world')
    parser.add_argument('--world', default='worlds/forest.sdf', help='World file to modify (default: custom gazebo world)')
    
    args = parser.parse_args()
    
    try:
        remove_fire_effects(args.world)
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
