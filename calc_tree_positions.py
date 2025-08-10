#!/usr/bin/env python3
import xml.etree.ElementTree as ET
import argparse

def extract_tree_positions(world_file, output_file='tree_positions.txt'):
    tree = ET.parse(world_file)
    root = tree.getroot()
    world = root.find('world')
    if world is None:
        print('Could not find <world> element in SDF')
        return

    count = 0
    with open(output_file, 'w') as f:
        for model in world.findall('model'):
            name = model.get('name', '')
            if 'tree' in name.lower():
                pose_elem = model.find('pose')
                if pose_elem is not None:
                    pose = pose_elem.text.strip().split()
                    if len(pose) >= 3:
                        x, y, z = pose[:3]
                        f.write(f"{x},{y},{z},{name}\n")
                        count += 1
    print(f"Extracted {count} tree positions to {output_file}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Extract tree positions from SDF world file')
    parser.add_argument('--world', default='/opt/ros/humble/share/turtlebot4_ignition_bringup/worlds/forest.sdf', help='Path to SDF world file')
    parser.add_argument('--output', default='tree_positions.txt', help='Output file for tree positions')
    args = parser.parse_args()
    extract_tree_positions(args.world, args.output)
