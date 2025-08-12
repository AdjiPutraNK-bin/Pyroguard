
import re
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
input_file = os.path.join(script_dir, '../worlds/forest.sdf')
output_file = os.path.join(script_dir, '../worlds/forest.sdf')

TREE_Z = 0      # Desired z for trees
WALL_TOP_Z = 2    # Desired z for walls

with open(input_file, "r") as f:
    content = f.read()

# This pattern matches the entire model, capturing the model tag, body, and closing tag
pattern = re.compile(
    r'(<model name="(?:tree_simple|wall_)[^>]*>)(.*?)(</model>)',
    re.DOTALL
)

def replacer(match):
    model_tag = match.group(1)
    body = match.group(2)
    closing = match.group(3)
    # Find all <pose>...</pose> in body
    poses = list(re.finditer(r'<pose>([^<]*)</pose>', body, re.DOTALL))
    if not poses:
        return match.group(0)
    last_pose = poses[-1]
    start, end = last_pose.span()
    pose_text = last_pose.group(1)
    # Decide which z to use based on model name
    if "tree_simple" in model_tag:
        z_val = TREE_Z
    else:
        z_val = WALL_TOP_Z
    parts = pose_text.split()
    if len(parts) >= 3:
        parts[2] = str(z_val)
    else:
        while len(parts) < 3:
            parts.append("0")
        parts[2] = str(z_val)
    new_pose = "<pose>" + " ".join(parts) + "</pose>"
    # Replace the last pose in body
    new_body = body[:start] + new_pose + body[end:]
    return f"{model_tag}{new_body}{closing}"

new_content = pattern.sub(replacer, content)

with open(output_file, "w") as f:
    f.write(new_content)

print(f"Updated file written to {output_file}")