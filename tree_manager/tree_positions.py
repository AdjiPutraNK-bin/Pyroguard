import xml.etree.ElementTree as ET

input_file = "../worlds/forest.sdf"
output_file = "tree_positions.txt"

tree = ET.parse(input_file)
root = tree.getroot()

positions = []
for model in root.iter('model'):
    name = model.attrib.get('name', '')
    if name.startswith('tree_simple'):
        # Only consider direct child <pose> of <model>
        pose_elem = None
        for child in model:
            if child.tag == 'pose':
                pose_elem = child
        if pose_elem is not None:
            pose = pose_elem.text.split()
            if len(pose) >= 3:
                x, y, z = pose[0], pose[1], pose[2]
                positions.append(f"{x},{y},{z},{name}")

with open(output_file, "w") as f:
    for pos in positions:
        f.write(pos + "\n")

print(f"Extracted {len(positions)} tree positions to {output_file}.")