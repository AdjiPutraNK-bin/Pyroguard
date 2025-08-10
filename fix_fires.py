GROUND_FIRE_HEIGHT = 2.0  # Height above ground for ground fires
TREE_BASE_Z = 0.2        # All trees have the same base z
TREE_HEIGHT = 6.7         # All trees have the same height
#!/usr/bin/env python3

import xml.etree.ElementTree as ET
import random
import math
import time

# Consistent prefix for all fire models
MODEL_PREFIX = "fire_model_"

def load_tree_positions(file_path='/opt/ros/humble/share/turtlebot4_ignition_bringup/worlds/tree_positions.txt'):
    """Load tree positions from file"""
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
    """Clean existing fires and add new ones with unique names"""
    
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
    min_dist = 1.0  # Reduced minimum allowed distance between fires (meters)
    grid_size = min_dist
    fire_grid = dict()  # (ix, iy) -> list of (x, y)

    def grid_key(x, y):
        return (int(x // grid_size), int(y // grid_size))

    def is_far_enough(x, y):
        gx, gy = grid_key(x, y)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                cell = (gx + dx, gy + dy)
                if cell in fire_grid:
                    for fx, fy in fire_grid[cell]:
                        if (x - fx) ** 2 + (y - fy) ** 2 < min_dist ** 2:
                            return False
        return True

    def add_to_grid(x, y):
        key = grid_key(x, y)
        if key not in fire_grid:
            fire_grid[key] = []
        fire_grid[key].append((x, y))

    for area in range(num_fire_areas):
        target_tree = random.choice(tree_positions)
        tree_x, tree_y, tree_z, tree_name = target_tree
        # Use constants for tree base and height
        base_z = TREE_BASE_Z
        tree_height = TREE_HEIGHT

        angle = random.uniform(0, 2 * math.pi)
        distance = random.uniform(2.0, 8.0)
        ground_x = tree_x + distance * math.cos(angle)
        ground_y = tree_y + distance * math.sin(angle)

        fire_type = random.choice(['ground_only', 'ground_to_tree', 'tree_cluster'])

        # Always at least 3-4 fires per area
        min_fires = 3
        max_fires = 4
        fires_this_area = random.randint(min_fires, max_fires)

        if fire_type == 'ground_only':
            for i in range(fires_this_area):
                tries = 0
                while True:
                    x = ground_x + random.uniform(-6.0, 6.0)
                    y = ground_y + random.uniform(-6.0, 6.0)
                    z = GROUND_FIRE_HEIGHT
                    scale = random.uniform(0.8, 1.5)
                    if is_far_enough(x, y) or tries > 10:
                        break
                    tries += 1
                add_single_fire(world_elem, fire_counter, x, y, z, scale, 'ground')
                add_to_grid(x, y)
                fire_counter += 1

        elif fire_type == 'ground_to_tree':
            # Always at least 3 fires: ground, trunk, canopy
            spread_chain = create_fire_spread_chain(ground_x, ground_y, tree_x, tree_y, 3)
            for i, (x, y, z, scale) in enumerate(spread_chain):
                fire_subtype = 'ground' if i == 0 else 'trunk' if i == 1 else 'canopy'
                tries = 0
                while not is_far_enough(x, y) and tries < 10:
                    x += random.uniform(-0.5, 0.5)
                    y += random.uniform(-0.5, 0.5)
                    tries += 1
                # Offset trunk and canopy fires to be proportional to tree height
                if fire_subtype == 'trunk':
                    z = base_z + tree_height * (1.0/3.0)
                elif fire_subtype == 'canopy':
                    z = base_z + tree_height * 0.9
                elif fire_subtype == 'ground':
                    z = GROUND_FIRE_HEIGHT
                add_single_fire(world_elem, fire_counter, x, y, z, scale, fire_subtype)
                add_to_grid(x, y)
                fire_counter += 1
            # If more than 3 fires requested, add extra ground fires
            extra_fires = fires_this_area - 3
            for i in range(extra_fires):
                tries = 0
                while True:
                    x = ground_x + random.uniform(-6.0, 6.0)
                    y = ground_y + random.uniform(-6.0, 6.0)
                    z = GROUND_FIRE_HEIGHT
                    scale = random.uniform(0.8, 1.5)
                    if is_far_enough(x, y) or tries > 10:
                        break
                    tries += 1
                add_single_fire(world_elem, fire_counter, x, y, z, scale, 'ground')
                add_to_grid(x, y)
                fire_counter += 1

        else:  # tree_cluster
            # Always at least 3 fires: 2 ground, 1 trunk, 1 canopy (if fires_this_area==4)
            n_ground = min(2, fires_this_area)
            for i in range(n_ground):
                tries = 0
                while True:
                    angle = random.uniform(0, 2 * math.pi)
                    distance = random.uniform(1.0, 8.0)
                    x = tree_x + distance * math.cos(angle)
                    y = tree_y + distance * math.sin(angle)
                    z = GROUND_FIRE_HEIGHT
                    scale = random.uniform(0.6, 1.2)
                    if is_far_enough(x, y) or tries > 10:
                        break
                    tries += 1
                add_single_fire(world_elem, fire_counter, x, y, z, scale, 'ground')
                add_to_grid(x, y)
                fire_counter += 1

            if fires_this_area >= 3:
                trunk_x = tree_x + random.uniform(-0.5, 0.5)
                trunk_y = tree_y + random.uniform(-0.5, 0.5)
                tries = 0
                while not is_far_enough(trunk_x, trunk_y) and tries < 10:
                    trunk_x += random.uniform(-0.2, 0.2)
                    trunk_y += random.uniform(-0.2, 0.2)
                    tries += 1
                trunk_z = base_z + tree_height * (1.0/3.0)
                add_single_fire(world_elem, fire_counter, trunk_x, trunk_y, trunk_z, 1.3, 'trunk')
                add_to_grid(trunk_x, trunk_y)
                fire_counter += 1

            if fires_this_area == 4:
                canopy_x = tree_x + random.uniform(-1.0, 1.0)
                canopy_y = tree_y + random.uniform(-1.0, 1.0)
                tries = 0
                while not is_far_enough(canopy_x, canopy_y) and tries < 10:
                    canopy_x += random.uniform(-0.5, 0.5)
                    canopy_y += random.uniform(-0.5, 0.5)
                    tries += 1
                canopy_z = base_z + tree_height * 0.9
                add_single_fire(world_elem, fire_counter, canopy_x, canopy_y, canopy_z, 0.8, 'canopy')
                add_to_grid(canopy_x, canopy_y)
                fire_counter += 1
    
    tree.write(world_file, encoding='utf-8', xml_declaration=True)
    total_fires = fire_counter - 1
    print(f"✅ Added {total_fires} new fires to {world_file}")

def create_fire_spread_chain(start_x, start_y, end_x, end_y, num_fires=3):
    """Create a chain of fires spreading from ground to tree canopy"""
    fire_positions = []
    
    for i in range(num_fires):
        progress = i / (num_fires - 1) if num_fires > 1 else 0
        
        x = start_x + (end_x - start_x) * progress
        y = start_y + (end_y - start_y) * progress
        
        if i == 0:
            z = 0.3
        elif i == 1:
            z = random.uniform(1.5, 2.5)
        else:
            z = random.uniform(3.0, 5.0)
        
        x += random.uniform(-0.5, 0.5)
        y += random.uniform(-0.5, 0.5)
        scale = 1.5 - (progress * 0.7)
        
        fire_positions.append((x, y, z, scale))
    
    return fire_positions

def add_single_fire(world_elem, fire_id, x, y, z, scale, fire_type='ground'):
    """Add a single fire model with consistent naming"""
    # Use unified prefix for model name
    model_name = f"{MODEL_PREFIX}{fire_id}"
    
    fire_model = ET.Element('model')
    fire_model.set('name', model_name)
    pose_elem = ET.SubElement(fire_model, 'pose')
    yaw = random.uniform(0, 2 * math.pi)
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
        offset_x = random.uniform(-0.05, 0.05)
        offset_y = random.uniform(-0.05, 0.05)
        pose_visual = ET.SubElement(visual_elem, 'pose')
        layer_yaw = rotation + random.uniform(-0.1, 0.1)
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
        albedo_elem.text = "file:///home/adji714/custom_gazebo_worlds/forest/fire.png"
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
    parser.add_argument('--world', default='/opt/ros/humble/share/turtlebot4_ignition_bringup/worlds/forest.sdf', help='World file')
    parser.add_argument('--areas', type=int, default=8, help='Number of fire areas')
    
    args = parser.parse_args()
    clean_and_add_fires(args.world, args.areas)