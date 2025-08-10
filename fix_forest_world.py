#!/usr/bin/env python3
"""
Script to fix the forest world file by:
1. Making all visual names unique to eliminate duplication errors
2. Fixing lighting issues that cause segmentation faults
"""
import re
import sys

def fix_forest_world(input_file, output_file):
    print(f"Reading {input_file}...")
    
    with open(input_file, 'r') as f:
        content = f.read()
    
    print("Fixing visual name duplications...")
    
    # Pattern to match tree models and their visual names
    tree_pattern = r'<model name="(tree_simple\d+)">(.*?)</model>'
    visual_name_pattern = r'<visual name="(trunk|treetop)">'
    
    def fix_tree_visuals(match):
        tree_name = match.group(1)
        tree_content = match.group(2)
        
        # Extract the tree number from the name
        tree_num = tree_name.replace('tree_simple', '')
        
        # Replace visual names to make them unique
        tree_content = re.sub(
            r'<visual name="trunk">',
            f'<visual name="trunk_{tree_num}">',
            tree_content
        )
        tree_content = re.sub(
            r'<visual name="treetop">',
            f'<visual name="treetop_{tree_num}">',
            tree_content
        )
        
        return f'<model name="{tree_name}">{tree_content}</model>'
    
    # Apply the fix to all tree models
    fixed_content = re.sub(tree_pattern, fix_tree_visuals, content, flags=re.DOTALL)
    
    print("Fixing lighting issues...")
    
    # Fix the lighting issue that causes segmentation fault
    # Remove the problematic directional light and simplify lighting
    light_pattern = r'<light type="directional" name="forest_sun">.*?</light>'
    
    # Replace with a simpler, safer light configuration
    simple_light = '''<light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 100 0 0.5 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.5 -0.5 -1</direction>
    </light>'''
    
    fixed_content = re.sub(light_pattern, simple_light, fixed_content, flags=re.DOTALL)
    
    print("Updating texture paths...")
    
    # Update texture paths to be relative instead of absolute
    fixed_content = fixed_content.replace(
        'file:///home/adji714/custom_gazebo_worlds/forest/',
        'file://textures/'
    )
    
    print(f"Writing fixed content to {output_file}...")
    
    with open(output_file, 'w') as f:
        f.write(fixed_content)
    
    print("✅ Forest world file fixed successfully!")
    print("Changes made:")
    print("- Made all visual names unique (trunk_N, treetop_N)")
    print("- Simplified lighting to prevent segmentation faults")
    print("- Updated texture paths to be relative")

if __name__ == "__main__":
    input_file = "/home/adji714/ros2_ws/src/pyroguard/worlds/forest.sdf"
    output_file = "/home/adji714/ros2_ws/src/pyroguard/worlds/forest_fixed.sdf"
    
    fix_forest_world(input_file, output_file)
