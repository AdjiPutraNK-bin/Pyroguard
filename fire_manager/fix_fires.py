GROUND_FIRE_HEIGHT = 1.0  # Height above ground for ground fires

#!/usr/bin/env python3

import xml.etree.ElementTree as ET
import random
import math
import os

# Consistent prefix for all fire models
MODEL_PREFIX = "fire_"

def load_tree_positions(file_path='../tree_manager/tree_positions.txt'):
    """Load tree positions from file"""
    # Always resolve relative to this script's directory
    if not os.path.isabs(file_path):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        file_path = os.path.join(script_dir, file_path)
    tree_positions = []
    try:
        with open(file_path, 'r') as f:
            for line in f:
                parts = line.strip().split(',')
                if len(parts) >= 4:
                    x, y, z, name = float(parts[0]), float(parts[1]), float(parts[2]), parts[3]
                    tree_positions.append((x, y, z, name))
    except FileNotFoundError:
        print(f"Tree positions file {file_path} not found.")
        return []
    return tree_positions

def save_fire_positions(positions, file_path='../fire_manager/fire_positions.txt'):
    """Save fire positions to file"""
    if not os.path.isabs(file_path):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        file_path = os.path.join(script_dir, file_path)
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    try:
        with open(file_path, 'w') as f:
            for x, y, z, name in positions:
                f.write(f"{x:.6f},{y:.6f},{z:.6f},{name}\n")
        print(f"Saved fire positions to {file_path}")
    except Exception as e:
        print(f"Error saving fire positions: {e}")

def get_next_fire_id(world_elem):
    """Get the next available fire ID to avoid duplicates"""
    existing_ids = set()
    for model in world_elem.findall('model'):
        name = model.get('name', '')
        if name.startswith(MODEL_PREFIX):
            try:
                # Extract ID from "fire_model_<id>"
                id_part = name.replace(MODEL_PREFIX, "")
                if id_part.isdigit():
                    existing_ids.add(int(id_part))
            except:
                pass
    
    next_id = 1
    while next_id in existing_ids:
        next_id += 1
    return next_id

def clean_and_add_fires(world_file, num_fire_areas=5):
    """Clean existing fires and add new steady individual ground fires"""
    
    tree_positions = load_tree_positions()
    if not tree_positions:
        print("No tree positions available.")
        return
    
    tree = ET.parse(world_file)
    root = tree.getroot()
    world_elem = root.find('world')
    if world_elem is None:
        print("Could not find world element")
        return
    
    print("Cleaning existing fires...")
    fires_removed = 0
    models_to_remove = []
    removed_names = []

    for model in list(world_elem.findall('model')):
        model_name = model.get('name', '')
        if model_name.startswith(MODEL_PREFIX):
            models_to_remove.append(model)
            removed_names.append(model_name)
            fires_removed += 1

    for model in models_to_remove:
        world_elem.remove(model)

    if removed_names:
        print(f"Removed fire models: {removed_names}")
    print(f"Removed {fires_removed} existing fires")
    
    fire_counter = get_next_fire_id(world_elem)
    fire_positions = []

    # Set seed for reproducible randomness
    random.seed(42)

    # Select unique trees deterministically
    num_areas = min(num_fire_areas, len(tree_positions))
    selected_trees = random.sample(tree_positions, num_areas)

    for target_tree in selected_trees:
        tree_x, tree_y, tree_z, tree_name = target_tree

        # Deterministic angle and distance for scattering
        angle = random.uniform(0, 2 * math.pi)
        distance = random.uniform(2.0, 8.0)
        ground_x = tree_x + distance * math.cos(angle)
        ground_y = tree_y + distance * math.sin(angle)
        z = GROUND_FIRE_HEIGHT
        scale = 1.0  # Fixed scale for simplicity

        model_name = f"{MODEL_PREFIX}{fire_counter}"
        add_single_fire(world_elem, fire_counter, ground_x, ground_y, z, scale, 'ground')
        fire_positions.append((ground_x, ground_y, z, model_name))
        fire_counter += 1
    
    tree.write(world_file, encoding='utf-8', xml_declaration=True)
    total_fires = fire_counter - 1
    print(f"✅ Added {total_fires} new fires to {world_file}")

    save_fire_positions(fire_positions)

def add_single_fire(world_elem, fire_id, x, y, z, scale, fire_type='ground'):
    """Add a single fire model with consistent naming"""
    # Use unified prefix for model name
    model_name = f"{MODEL_PREFIX}{fire_id}"
    
    fire_model = ET.Element('model')
    fire_model.set('name', model_name)
    pose_elem = ET.SubElement(fire_model, 'pose')
    yaw = 0.0  # Fixed yaw for simplicity
    pose_elem.text = f"{x:.2f} {y:.2f} {z:.2f} 0 0 {yaw:.2f}"
    static_elem = ET.SubElement(fire_model, 'static')
    static_elem.text = 'true'
    link_elem = ET.SubElement(fire_model, 'link')
    link_elem.set('name', 'fire_link')

    # Only 3 visuals per fire, at 0, 60, and 120 degrees
    for j, (visual_name, width_mult, height_mult, depth, rotation) in enumerate([
        ('fire_main', 1.0, 1.0, 0.1, 0),
        ('fire_60', 1.0, 1.0, 0.1, math.radians(60)),
        ('fire_120', 1.0, 1.0, 0.1, math.radians(120)),
    ]):
        visual_elem = ET.SubElement(link_elem, 'visual')
        # Include model prefix in visual name
        visual_elem.set('name', f'{model_name}_{visual_name}_{j}')
        offset_z = j * 0.1
        offset_x = 0.0  # Fixed offset for simplicity
        offset_y = 0.0  # Fixed offset for simplicity
        pose_visual = ET.SubElement(visual_elem, 'pose')
        layer_yaw = rotation  # Fixed yaw without random variation
        pose_visual.text = f"{offset_x:.2f} {offset_y:.2f} {offset_z:.2f} 0 0 {layer_yaw:.2f}"
        geometry_elem = ET.SubElement(visual_elem, 'geometry')
        box_elem = ET.SubElement(geometry_elem, 'box')
        size_elem = ET.SubElement(box_elem, 'size')
        if fire_type == 'ground':
            base_width = 1.8 * scale
            base_height = 2.0 * scale
        elif fire_type == 'trunk':
            base_width = 1.5 * scale
            base_height = 3.0 * scale
        else:  # canopy
            base_width = 2.2 * scale
            base_height = 3.5 * scale
        width = base_width * width_mult
        height = base_height * height_mult
        size_elem.text = f"{width:.2f} {depth:.2f} {height:.2f}"
        material_elem = ET.SubElement(visual_elem, 'material')
        ambient_elem = ET.SubElement(material_elem, 'ambient')
        ambient_elem.text = "0.7 0.35 0.0 1"
        diffuse_elem = ET.SubElement(material_elem, 'diffuse')
        diffuse_elem.text = "1 1 1 1"
        specular_elem = ET.SubElement(material_elem, 'specular')
        specular_elem.text = "0 0 0 1"
        emissive_elem = ET.SubElement(material_elem, 'emissive')
        emissive_elem.text = "0 0 0 1"
        pbr_elem = ET.SubElement(material_elem, 'pbr')
        metal_elem = ET.SubElement(pbr_elem, 'metal')
        albedo_elem = ET.SubElement(metal_elem, 'albedo_map')
        albedo_elem.text = "textures/fire.png"
        metalness_elem = ET.SubElement(metal_elem, 'metalness')
        metalness_elem.text = "0.0"
        roughness_elem = ET.SubElement(metal_elem, 'roughness')
        roughness_elem.text = "0.6"
        transparency_elem = ET.SubElement(visual_elem, 'transparency')
        transparency_elem.text = "0.0"
        cast_shadows_elem = ET.SubElement(visual_elem, 'cast_shadows')
        cast_shadows_elem.text = 'false'

    # Add 3 point lights per fire, arranged in a circle
    light_radius = 0.4 * scale
    for j in range(3):
        angle = math.radians(j * 120)
        lx = light_radius * math.cos(angle)
        ly = light_radius * math.sin(angle)
        lz = 0.5
        light_elem = ET.SubElement(link_elem, 'light')
        light_elem.set('type', 'point')
        # Include model prefix in light name
        light_elem.set('name', f'{model_name}_light_{j}')
        light_pose = ET.SubElement(light_elem, 'pose')
        light_pose.text = f"{lx:.2f} {ly:.2f} {lz:.2f} 0 0 0"
        light_diffuse = ET.SubElement(light_elem, 'diffuse')
        light_diffuse.text = "1.0 0.7 0.3 1"
        light_specular = ET.SubElement(light_elem, 'specular')
        light_specular.text = "1.0 0.8 0.3 1"
        intensity_elem = ET.SubElement(light_elem, 'intensity')
        intensity_elem.text = "2"
        attenuation_elem = ET.SubElement(light_elem, 'attenuation')
        range_elem = ET.SubElement(attenuation_elem, 'range')
        range_elem.text = "2"
        constant_elem = ET.SubElement(attenuation_elem, 'constant')
        constant_elem.text = "0.1"
        linear_elem = ET.SubElement(attenuation_elem, 'linear')
        linear_elem.text = "0.05"
        quadratic_elem = ET.SubElement(attenuation_elem, 'quadratic')
        quadratic_elem.text = "0.02"
        cast_shadows_light = ET.SubElement(light_elem, 'cast_shadows')
        cast_shadows_light.text = 'false'

    world_elem.append(fire_model)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Clean and add realistic fires')
    parser.add_argument('--world', default='../worlds/forest.sdf', help='World file')
    parser.add_argument('--areas', type=int, default=8, help='Number of fire areas')

    args = parser.parse_args()
    # Always resolve world file relative to this script's directory if not absolute
    world_file = args.world
    if not os.path.isabs(world_file):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        world_file = os.path.join(script_dir, world_file)
    clean_and_add_fires(world_file, args.areas)