#!/usr/bin/env python3
"""
Filter trees outside given rectangular bounds from an SDF world file.
Usage: python3 filter_forest.py --input <input.sdf> --output <output.sdf> --bounds <half_extent>
Default bounds half_extent is 25.95 (matches walls in world).
This script creates a backup of the input file as <input>.bak before overwriting.
"""
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--input', '-i', required=True)
    p.add_argument('--output', '-o')
    p.add_argument('--half_extent', '-b', type=float, default=25.95,
                   help='Half-extent of inner square (default 25.95)')
    p.add_argument('--overwrite', action='store_true')
    return p.parse_args()


def main():
    args = parse_args()
    inp = Path(args.input)
    if not inp.exists():
        raise SystemExit(f"Input not found: {inp}")
    out = Path(args.output) if args.output else inp.with_suffix('.pruned.sdf')

    # parse preserving namespaces
    tree = ET.parse(str(inp))
    root = tree.getroot()

    # find the <world> element; models are children of <world>
    world = root.find('world') if root is not None else None
    if world is None and root.tag == 'world':
        world = root
    if world is None:
        raise SystemExit('Could not find <world> element in SDF')

    removed = 0
    kept = 0
    half = args.half_extent

    # iterate over model children under <world>
    for model in list(world.findall('model')):
        name = model.get('name', '')
        if not name.startswith('tree_simple'):
            kept += 1
            continue
        pose = model.find('pose')
        if pose is None or pose.text is None:
            kept += 1
            continue
        coords = pose.text.strip().split()
        try:
            x = float(coords[0])
            y = float(coords[1])
        except Exception:
            kept += 1
            continue
        if abs(x) > half or abs(y) > half:
            world.remove(model)
            removed += 1
        else:
            kept += 1

    # backup if overwriting original
    if args.overwrite:
        bak = inp.with_suffix(inp.suffix + '.bak')
        if not bak.exists():
            bak.write_bytes(inp.read_bytes())

    # write output
    tree.write(str(out), encoding='utf-8', xml_declaration=True)
    print(f"Wrote {out} (kept={kept}, removed={removed})")

if __name__ == '__main__':
    main()
