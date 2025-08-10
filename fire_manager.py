#!/usr/bin/env python3

import xml.etree.ElementTree as ET
import random
import math
import time
import os
import shutil
import argparse

class FireManager:
    """
    Comprehensive fire management system for TurtleBot4 forest world.
    Handles adding and removing fires with realistic fire behavior patterns.
    """
    
    def __init__(self, world_file_path):
        self.world_file = world_file_path
        self.backup_file = world_file_path + '.backup'
        
    def create_backup(self):
        """Create a backup of the world file before modifications"""
        try:
            shutil.copy2(self.world_file, self.backup_file)
            print(f"✅ Backup created: {self.backup_file}")
        except Exception as e:
            print(f"⚠️ Warning: Could not create backup: {e}")
    
    def restore_backup(self):
        """Restore world file from backup"""
        try:
            if os.path.exists(self.backup_file):
                shutil.copy2(self.backup_file, self.world_file)
                print(f"✅ Restored from backup: {self.backup_file}")
                return True
            else:
                print(f"❌ No backup file found: {self.backup_file}")
                return False
        except Exception as e:
            print(f"❌ Error restoring backup: {e}")
            return False
    
    def get_next_fire_id(self, world_elem):
        """Get the next available fire ID to avoid duplicates"""
        existing_ids = set()
        for model in world_elem.findall('model'):
            name = model.get('name', '')
            if name.startswith('fire_'):
                try:
                    # Extract number from names like fire_123, realistic_fire_456, etc.
                    parts = name.split('_')
                    if len(parts) >= 2 and parts[-1].isdigit():
                        existing_ids.add(int(parts[-1]))
                except:
                    pass
        
        # Find next available ID
        next_id = 1
        while next_id in existing_ids:
            next_id += 1
        return next_id
    
    def remove_all_fires(self):
        """Remove all fire effects from the world file using original logic"""
        try:
            # Parse the existing world file
            tree = ET.parse(self.world_file)
            root = tree.getroot()
            
            # Find the world element
            world = root.find('world')
            if world is None:
                print("Error: Could not find world element in SDF file")
                return False
            
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
            tree.write(self.world_file, encoding='utf-8', xml_declaration=True)
            print(f"\n✅ Removed {len(fire_models)} fire effects from {self.world_file}")
            return True
            
        except Exception as e:
            print(f"❌ Error removing fires: {e}")
            return False
    
    def add_strategic_fires(self, num_fires=6, fire_areas=3):
        """
        Add fires using the original fix_fires.py logic but adapted for TurtleBot4 world.
        Creates strategic fire placements for training scenarios.
        """
        try:
            tree = ET.parse(self.world_file)
            root = tree.getroot()
            world_elem = root.find('world')
            if world_elem is None:
                print("Could not find world element")
                return False

            # Get starting fire ID
            fire_counter = self.get_next_fire_id(world_elem)
            
            # Predefined strategic positions for TurtleBot4 training
            # These positions are chosen to create interesting navigation challenges
            strategic_positions = [
                (5.0, 5.0, 0.5),    # Northeast area
                (-3.0, 8.0, 0.4),   # Northwest area
                (15.0, 10.0, 0.6),  # Far northeast
                (-8.0, -5.0, 0.3),  # Southwest area
                (20.0, -8.0, 0.5),  # Far southeast
                (0.0, 15.0, 0.4),   # North area
                (12.0, -3.0, 0.5),  # Southeast area
                (-5.0, 12.0, 0.6),  # Far northwest
            ]
            
            fires_added = 0
            
            # Add fires at strategic positions
            for i in range(min(num_fires, len(strategic_positions))):
                x, y, base_z = strategic_positions[i]
                
                # Create fire cluster around this position
                fire_type = random.choice(['ground_only', 'ground_cluster', 'vertical_spread'])
                
                if fire_type == 'ground_only':
                    # Single ground fire with some variation
                    z = base_z + random.uniform(-0.1, 0.2)
                    scale = random.uniform(0.8, 1.2)
                    self._add_single_fire(world_elem, fire_counter, x, y, z, scale, 'ground')
                    fire_counter += 1
                    fires_added += 1
                    
                elif fire_type == 'ground_cluster':
                    # Multiple ground fires in cluster
                    num_cluster_fires = random.randint(2, 4)
                    for j in range(num_cluster_fires):
                        cluster_x = x + random.uniform(-2.0, 2.0)
                        cluster_y = y + random.uniform(-2.0, 2.0)
                        cluster_z = base_z + random.uniform(-0.1, 0.3)
                        scale = random.uniform(0.6, 1.0)
                        
                        self._add_single_fire(world_elem, fire_counter, cluster_x, cluster_y, cluster_z, scale, 'ground')
                        fire_counter += 1
                        fires_added += 1
                        
                else:  # vertical_spread
                    # Fires at different heights to simulate tree burning
                    heights = [base_z, base_z + 1.5, base_z + 3.0]
                    fire_types = ['ground', 'trunk', 'canopy']
                    scales = [1.2, 1.0, 0.8]
                    
                    for height, f_type, scale in zip(heights, fire_types, scales):
                        fire_x = x + random.uniform(-0.5, 0.5)
                        fire_y = y + random.uniform(-0.5, 0.5)
                        
                        self._add_single_fire(world_elem, fire_counter, fire_x, fire_y, height, scale, f_type)
                        fire_counter += 1
                        fires_added += 1

            # Save the file
            tree.write(self.world_file, encoding='utf-8', xml_declaration=True)
            print(f"✅ Added {fires_added} strategic fires to {self.world_file}")
            return True
            
        except Exception as e:
            print(f"❌ Error adding fires: {e}")
            return False
    
    def add_realistic_fires(self, num_fire_areas=5):
        """
        Add fires using the original complex fire generation logic from fix_fires.py
        This creates more realistic fire patterns but requires tree position data.
        """
        try:
            # For now, use strategic positions as "tree" positions
            # In the future, this could be enhanced to read actual tree positions
            tree_positions = [
                (5.0, 5.0, 0.0, 'tree_1'),
                (10.0, 8.0, 0.0, 'tree_2'),
                (-5.0, 10.0, 0.0, 'tree_3'),
                (15.0, -5.0, 0.0, 'tree_4'),
                (-8.0, -8.0, 0.0, 'tree_5'),
                (0.0, 12.0, 0.0, 'tree_6'),
                (12.0, 2.0, 0.0, 'tree_7'),
                (-3.0, -12.0, 0.0, 'tree_8'),
            ]
            
            if not tree_positions:
                print("No tree positions available for realistic fire generation.")
                return False
            
            tree = ET.parse(self.world_file)
            root = tree.getroot()
            world_elem = root.find('world')
            if world_elem is None:
                print("Could not find world element")
                return False

            # Get starting fire ID
            fire_counter = self.get_next_fire_id(world_elem)
            
            # Add new fires using original logic
            for area in range(num_fire_areas):
                target_tree = random.choice(tree_positions)
                tree_x, tree_y, tree_z, tree_name = target_tree
                
                # Create ground fire start point
                angle = random.uniform(0, 2 * math.pi)
                distance = random.uniform(2.0, 6.0)
                ground_x = tree_x + distance * math.cos(angle)
                ground_y = tree_y + distance * math.sin(angle)
                
                fire_type = random.choice(['ground_only', 'ground_to_tree', 'tree_cluster'])
                
                if fire_type == 'ground_only':
                    num_ground_fires = random.randint(2, 4)
                    for i in range(num_ground_fires):
                        x = ground_x + random.uniform(-3.0, 3.0)
                        y = ground_y + random.uniform(-3.0, 3.0)
                        z = random.uniform(0.2, 0.8)
                        scale = random.uniform(0.8, 1.5)
                        
                        self._add_single_fire(world_elem, fire_counter, x, y, z, scale, 'ground')
                        fire_counter += 1
                
                elif fire_type == 'ground_to_tree':
                    spread_chain = self._create_fire_spread_chain(ground_x, ground_y, tree_x, tree_y, 3)
                    
                    for i, (x, y, z, scale) in enumerate(spread_chain):
                        fire_subtype = 'ground' if i == 0 else 'trunk' if i == 1 else 'canopy'
                        self._add_single_fire(world_elem, fire_counter, x, y, z, scale, fire_subtype)
                        fire_counter += 1
                
                else:  # tree_cluster
                    # Ground fires around tree
                    for i in range(random.randint(2, 3)):
                        angle = random.uniform(0, 2 * math.pi)
                        distance = random.uniform(1.0, 4.0)
                        x = tree_x + distance * math.cos(angle)
                        y = tree_y + distance * math.sin(angle)
                        z = random.uniform(0.3, 1.0)
                        scale = random.uniform(0.6, 1.2)
                        
                        self._add_single_fire(world_elem, fire_counter, x, y, z, scale, 'ground')
                        fire_counter += 1
                    
                    # Tree trunk fire
                    trunk_x = tree_x + random.uniform(-0.5, 0.5)
                    trunk_y = tree_y + random.uniform(-0.5, 0.5)
                    self._add_single_fire(world_elem, fire_counter, trunk_x, trunk_y, 2.0, 1.3, 'trunk')
                    fire_counter += 1
                    
                    # Canopy fire
                    canopy_x = tree_x + random.uniform(-1.0, 1.0)
                    canopy_y = tree_y + random.uniform(-1.0, 1.0)
                    self._add_single_fire(world_elem, fire_counter, canopy_x, canopy_y, 4.5, 0.8, 'canopy')
                    fire_counter += 1

            # Save the file
            tree.write(self.world_file, encoding='utf-8', xml_declaration=True)
            total_fires = fire_counter - self.get_next_fire_id(world_elem) + len(world_elem.findall('.//model[contains(@name, "fire_")]'))
            print(f"✅ Added realistic fires to {self.world_file}")
            return True
            
        except Exception as e:
            print(f"❌ Error adding realistic fires: {e}")
            return False
    
    def _create_fire_spread_chain(self, start_x, start_y, end_x, end_y, num_fires=3):
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
    
    def _add_single_fire(self, world_elem, fire_id, x, y, z, scale, fire_type='ground'):
        """Add a single fire model with unique name using original logic"""
        
        fire_model = ET.Element('model')
        fire_model.set('name', f'fire_{fire_id}')  # Simple unique naming
        
        pose_elem = ET.SubElement(fire_model, 'pose')
        yaw = random.uniform(0, 2 * math.pi)
        pose_elem.text = f"{x:.2f} {y:.2f} {z:.2f} 0 0 {yaw:.2f}"
        
        static_elem = ET.SubElement(fire_model, 'static')
        static_elem.text = 'true'
        
        link_elem = ET.SubElement(fire_model, 'link')
        link_elem.set('name', 'fire_link')
        
        # Fire intensity based on type
        if fire_type == 'ground':
            intensity_mult = 1.2
            emissive_base = (1.0, 0.7, 0.3)
        elif fire_type == 'trunk':
            intensity_mult = 1.0
            emissive_base = (1.0, 0.6, 0.2)
        else:  # canopy
            intensity_mult = 0.8
            emissive_base = (0.9, 0.5, 0.1)
        
        # Add visual elements (original complex fire visual logic)
        for j, (visual_name, width_mult, height_mult, depth, transparency, rotation) in enumerate([
            ('fire_main', 1.0, 1.0, 0.1, 0.0, 0),
            ('fire_cross', 1.1, 1.1, 0.1, 0.0, 1.57),
            ('fire_diagonal', 0.9, 0.95, 0.08, 0.0, 0.785),
            ('fire_core', 0.6, 0.8, 0.06, 0.0, 0.4)
        ]):
            visual_elem = ET.SubElement(link_elem, 'visual')
            visual_elem.set('name', f'{visual_name}_{fire_id}_{j}')  # Unique visual names
            
            offset_z = j * 0.1
            offset_x = random.uniform(-0.05, 0.05)
            offset_y = random.uniform(-0.05, 0.05)
            
            pose_visual = ET.SubElement(visual_elem, 'pose')
            layer_yaw = rotation + random.uniform(-0.1, 0.1)
            pose_visual.text = f"{offset_x:.2f} {offset_y:.2f} {offset_z:.2f} 0 0 {layer_yaw:.2f}"
            
            geometry_elem = ET.SubElement(visual_elem, 'geometry')
            box_elem = ET.SubElement(geometry_elem, 'box')
            size_elem = ET.SubElement(box_elem, 'size')
            
            # Size based on fire type
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
            
            # Material with original color logic
            material_elem = ET.SubElement(visual_elem, 'material')
            
            if j == 0:
                ambient_color = (1.0, 0.8, 0.2)
                diffuse_color = (1.0, 0.6, 0.1)
                emissive_color = tuple(c * intensity_mult for c in emissive_base)
            elif j == 1:
                ambient_color = (1.0, 0.7, 0.1)
                diffuse_color = (1.0, 0.5, 0.0)
                emissive_color = tuple(c * intensity_mult * 0.8 for c in emissive_base)
            elif j == 2:
                ambient_color = (1.0, 0.6, 0.0)
                diffuse_color = (1.0, 0.4, 0.0)
                emissive_color = tuple(c * intensity_mult * 0.6 for c in emissive_base)
            else:
                ambient_color = (1.0, 0.9, 0.4)
                diffuse_color = (1.0, 0.8, 0.3)
                emissive_color = tuple(c * intensity_mult * 1.2 for c in emissive_base)
            
            emissive_color = tuple(min(1.0, c) for c in emissive_color)
            
            ambient_elem = ET.SubElement(material_elem, 'ambient')
            ambient_elem.text = f"{ambient_color[0]:.1f} {ambient_color[1]:.1f} {ambient_color[2]:.1f} 1.0"
            
            diffuse_elem = ET.SubElement(material_elem, 'diffuse')
            diffuse_elem.text = f"{diffuse_color[0]:.1f} {diffuse_color[1]:.1f} {diffuse_color[2]:.1f} 1.0"
            
            specular_elem = ET.SubElement(material_elem, 'specular')
            specular_elem.text = "0.0 0.0 0.0 1"
            
            emissive_elem = ET.SubElement(material_elem, 'emissive')
            emissive_elem.text = f"{emissive_color[0]:.1f} {emissive_color[1]:.1f} {emissive_color[2]:.1f} 1"
            
            pbr_elem = ET.SubElement(material_elem, 'pbr')
            metal_elem = ET.SubElement(pbr_elem, 'metal')
            
            # Use local fire texture if available, otherwise use original path
            fire_texture_path = os.path.join(os.path.dirname(self.world_file), '..', 'fire.png')
            if os.path.exists(fire_texture_path):
                albedo_elem = ET.SubElement(metal_elem, 'albedo_map')
                albedo_elem.text = f"file://{os.path.abspath(fire_texture_path)}"
            else:
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
        
        # Add point light (original lighting logic)
        light_elem = ET.SubElement(link_elem, 'light')
        light_elem.set('type', 'point')
        light_elem.set('name', f'fire_light_{fire_id}')  # Unique light name
        
        light_pose = ET.SubElement(light_elem, 'pose')
        light_pose.text = "0 0 1.5 0 0 0"
        
        light_diffuse = ET.SubElement(light_elem, 'diffuse')
        light_color = tuple(c * intensity_mult for c in (1.0, 0.7, 0.3))
        light_color = tuple(min(1.0, c) for c in light_color)
        light_diffuse.text = f"{light_color[0]:.2f} {light_color[1]:.2f} {light_color[2]:.2f} 1"
        
        light_specular = ET.SubElement(light_elem, 'specular')
        light_specular.text = "1.0 0.8 0.3 1"
        
        attenuation_elem = ET.SubElement(light_elem, 'attenuation')
        
        if fire_type == 'ground':
            light_range = scale * 8.0
        elif fire_type == 'trunk':
            light_range = scale * 12.0
        else:
            light_range = scale * 15.0
        
        range_elem = ET.SubElement(attenuation_elem, 'range')
        range_elem.text = f"{light_range:.1f}"
        
        constant_elem = ET.SubElement(attenuation_elem, 'constant')
        constant_elem.text = "0.1"
        
        linear_elem = ET.SubElement(attenuation_elem, 'linear')
        linear_elem.text = "0.05"
        
        quadratic_elem = ET.SubElement(attenuation_elem, 'quadratic')
        quadratic_elem.text = "0.02"
        
        cast_shadows_light = ET.SubElement(light_elem, 'cast_shadows')
        cast_shadows_light.text = 'false'
        
        world_elem.append(fire_model)
    
    def list_fires(self):
        """List all fire models currently in the world"""
        try:
            tree = ET.parse(self.world_file)
            root = tree.getroot()
            world = root.find('world')
            if world is None:
                print("Error: Could not find world element in SDF file")
                return []
            
            fire_models = []
            for model in world.findall('model'):
                model_name = model.get('name', '')
                if 'fire' in model_name.lower():
                    pose_elem = model.find('pose')
                    if pose_elem is not None:
                        pose_data = pose_elem.text.strip().split()
                        if len(pose_data) >= 3:
                            x, y, z = pose_data[0], pose_data[1], pose_data[2]
                            fire_models.append((model_name, f"({x}, {y}, {z})"))
                        else:
                            fire_models.append((model_name, "Unknown position"))
                    else:
                        fire_models.append((model_name, "No pose data"))
            
            if fire_models:
                print(f"\n🔥 Found {len(fire_models)} fire models:")
                for name, position in fire_models:
                    print(f"  - {name} at {position}")
            else:
                print("\n✅ No fire models found in the world")
            
            return fire_models
            
        except Exception as e:
            print(f"❌ Error listing fires: {e}")
            return []


def main():
    """Command line interface for fire management"""
    parser = argparse.ArgumentParser(description='Comprehensive fire management for TurtleBot4 forest world')
    parser.add_argument('--world', default='/opt/ros/humble/share/turtlebot4_ignition_bringup/worlds/forest.sdf',
                        help='World file to modify')
    parser.add_argument('--action', choices=['add', 'remove', 'list', 'backup', 'restore'],
                        required=True, help='Action to perform')
    parser.add_argument('--type', choices=['strategic', 'realistic'], default='strategic',
                        help='Type of fire pattern to add (strategic=simple, realistic=complex)')
    parser.add_argument('--fires', type=int, default=6, help='Number of fires to add')
    parser.add_argument('--areas', type=int, default=3, help='Number of fire areas for realistic pattern')
    
    args = parser.parse_args()
    
    # Initialize fire manager
    fire_manager = FireManager(args.world)
    
    # Execute requested action
    if args.action == 'backup':
        fire_manager.create_backup()
        
    elif args.action == 'restore':
        if fire_manager.restore_backup():
            print("✅ World file restored from backup")
        else:
            print("❌ Failed to restore from backup")
            
    elif args.action == 'list':
        fire_manager.list_fires()
        
    elif args.action == 'remove':
        fire_manager.create_backup()
        if fire_manager.remove_all_fires():
            print("✅ All fires removed successfully")
        else:
            print("❌ Failed to remove fires")
            
    elif args.action == 'add':
        fire_manager.create_backup()
        
        if args.type == 'strategic':
            if fire_manager.add_strategic_fires(args.fires, args.areas):
                print("✅ Strategic fires added successfully")
            else:
                print("❌ Failed to add strategic fires")
        else:  # realistic
            if fire_manager.add_realistic_fires(args.areas):
                print("✅ Realistic fires added successfully")
            else:
                print("❌ Failed to add realistic fires")
        
        # List fires after adding
        print("\nCurrent fire status:")
        fire_manager.list_fires()


if __name__ == "__main__":
    main()
