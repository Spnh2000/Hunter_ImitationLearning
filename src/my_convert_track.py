import numpy as np
import yaml
from PIL import Image
import xml.etree.ElementTree as ET
from xml.dom import minidom
import argparse
import os
import cv2


def load_map_data(image_file, yaml_file):
    with open(yaml_file, 'r') as f:
        map_metadata = yaml.safe_load(f)
    image = Image.open(image_file).convert('L')
    map_array = np.array(image)
    return map_array, map_metadata


def _compute_depths(hierarchy):
    """Return a dict mapping contour index → hierarchy depth (0 = top-level)."""
    h = hierarchy[0]
    depths = {}

    def depth_of(idx):
        if idx in depths:
            return depths[idx]
        parent = h[idx][3]
        depths[idx] = 0 if parent == -1 else depth_of(parent) + 1
        return depths[idx]

    for i in range(len(h)):
        depth_of(i)
    return depths


def get_contours(map_array, threshold=128, epsilon_factor=0.002):
    """
    Extract and simplify wall contours, keeping only one contour per physical wall.

    The original script used RETR_TREE which returns both the outer and inner edges
    of each wall band. These near-duplicate contours produce double walls in Gazebo.
    Fix: compute hierarchy depth and keep only even-depth contours (outer edges).
    """
    print("Detecting contours...")
    binary_map = np.where(map_array < threshold, 255, 0).astype(np.uint8)

    contours, hierarchy = cv2.findContours(
        binary_map, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )
    print(f"Found {len(contours)} raw contours.")

    if hierarchy is None or len(contours) == 0:
        return []

    depths = _compute_depths(hierarchy)

    simplified = []
    total_points = 0

    for i, cnt in enumerate(contours):
        # Odd-depth contours are the "inner face" of a wall band — skip them.
        # Keeping only even depths gives exactly one contour per physical wall.
        if depths[i] % 2 != 0:
            continue
        if cv2.contourArea(cnt) < 5:
            continue

        epsilon = epsilon_factor * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        if len(approx) > 1:
            simplified.append(approx)
            total_points += len(approx)

    print(f"Kept {len(simplified)} contours (double-wall fix applied).")
    print(f"Total vertices after simplification: {total_points}.")
    return simplified


def generate_walls(contours, resolution, origin, map_height, wall_height, wall_thickness):
    walls = []
    wall_count = 0
    origin_x, origin_y = origin[0], origin[1]

    for contour in contours:
        for i in range(len(contour)):
            p1 = contour[i][0]
            p2 = contour[(i + 1) % len(contour)][0]

            if np.array_equal(p1, p2):
                continue

            x1 = origin_x + p1[0] * resolution
            y1 = origin_y + (map_height - p1[1]) * resolution
            x2 = origin_x + p2[0] * resolution
            y2 = origin_y + (map_height - p2[1]) * resolution

            dx, dy = x2 - x1, y2 - y1
            length = np.sqrt(dx**2 + dy**2)
            angle = np.arctan2(dy, dx)

            walls.append({
                'name': f'wall_{wall_count}',
                'x': (x1 + x2) / 2.0,
                'y': (y1 + y2) / 2.0,
                'z': wall_height / 2.0,
                'length': length,
                'width': wall_thickness,
                'height': wall_height,
                'yaw': angle,
            })
            wall_count += 1

    return walls


def create_sdf(walls, world_name="track_world"):
    sdf = ET.Element("sdf", version="1.6")
    world = ET.SubElement(sdf, "world", name=world_name)

    scene = ET.SubElement(world, "scene")
    ET.SubElement(scene, "ambient").text = "0.4 0.4 0.4 1"
    ET.SubElement(scene, "background").text = "0.7 0.7 0.7 1"
    ET.SubElement(scene, "shadows").text = "true"

    light = ET.SubElement(world, "light", name="sun", type="directional")
    ET.SubElement(light, "pose").text = "0 0 10 0 0 0"
    ET.SubElement(light, "diffuse").text = "0.8 0.8 0.8 1"
    ET.SubElement(light, "direction").text = "-0.5 0.1 -0.9"

    ground = ET.SubElement(world, "model", name="ground_plane")
    ET.SubElement(ground, "static").text = "true"
    link = ET.SubElement(ground, "link", name="link")

    coll = ET.SubElement(link, "collision", name="collision")
    geom = ET.SubElement(coll, "geometry")
    plane = ET.SubElement(geom, "plane")
    ET.SubElement(plane, "normal").text = "0 0 1"
    ET.SubElement(plane, "size").text = "1000 1000"

    vis = ET.SubElement(link, "visual", name="visual")
    geom_vis = ET.SubElement(vis, "geometry")
    plane_vis = ET.SubElement(geom_vis, "plane")
    ET.SubElement(plane_vis, "normal").text = "0 0 1"
    ET.SubElement(plane_vis, "size").text = "1000 1000"
    mat = ET.SubElement(vis, "material")
    ET.SubElement(mat, "ambient").text = "0.3 0.3 0.3 1"
    ET.SubElement(mat, "diffuse").text = "0.3 0.3 0.3 1"

    print(f"Generating XML for {len(walls)} wall segments...")
    for wall in walls:
        model = ET.SubElement(world, "model", name=wall['name'])
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = (
            f"{wall['x']} {wall['y']} {wall['z']} 0 0 {wall['yaw']}"
        )

        link = ET.SubElement(model, "link", name="link")
        size_str = f"{wall['length']} {wall['width']} {wall['height']}"

        vis = ET.SubElement(link, "visual", name="visual")
        geom = ET.SubElement(vis, "geometry")
        ET.SubElement(ET.SubElement(geom, "box"), "size").text = size_str
        mat = ET.SubElement(vis, "material")
        ET.SubElement(mat, "ambient").text = "1 0 0 1"
        ET.SubElement(mat, "diffuse").text = "1 0 0 1"

        coll = ET.SubElement(link, "collision", name="collision")
        geom = ET.SubElement(coll, "geometry")
        ET.SubElement(ET.SubElement(geom, "box"), "size").text = size_str

    return sdf


def main():
    parser = argparse.ArgumentParser(
        description="Racetrack image-to-SDF converter (double-wall fix)"
    )
    parser.add_argument("image_file", help="Path to PGM or PNG map file")
    parser.add_argument("yaml_file", help="Path to YAML metadata file")
    parser.add_argument("-o", "--output", default="track.world", help="Output SDF file")
    parser.add_argument("--height", type=float, default=1.0, help="Wall height (m)")
    parser.add_argument("--thickness", type=float, default=0.15, help="Wall thickness (m)")
    parser.add_argument(
        "--smoothness",
        type=float,
        default=0.002,
        help="Contour simplification factor (lower=more detail)",
    )
    args = parser.parse_args()

    if not os.path.exists(args.image_file) or not os.path.exists(args.yaml_file):
        print("Error: Input files not found.")
        return

    map_array, metadata = load_map_data(args.image_file, args.yaml_file)

    contours = get_contours(map_array, epsilon_factor=args.smoothness)

    walls = generate_walls(
        contours,
        metadata["resolution"],
        metadata["origin"],
        map_array.shape[0],
        args.height,
        args.thickness,
    )

    sdf_root = create_sdf(walls)

    xml_str = minidom.parseString(ET.tostring(sdf_root)).toprettyxml(indent="  ")
    with open(args.output, "w") as f:
        f.write(xml_str)

    print(f"Done! Saved to {args.output}  ({len(walls)} wall segments)")


if __name__ == "__main__":
    main()
