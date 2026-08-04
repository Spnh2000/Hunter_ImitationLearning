import os
import yaml
import cv2
import numpy as np
import sys
from scipy.ndimage import maximum_filter

def evaluate_map(yaml_path, pgm_path):
    print(f"--- Map Evaluation Report ---")
    print(f"Map YAML: {yaml_path}")
    print(f"Map Image: {pgm_path}\n")

    # Load YAML
    try:
        with open(yaml_path, 'r') as f:
            metadata = yaml.safe_load(f)
        resolution = metadata['resolution']
        origin = metadata['origin']
        print(f"Resolution: {resolution} meters/pixel")
        print(f"Origin: {origin}\n")
    except Exception as e:
        print(f"Error loading YAML: {e}")
        return

    # Load Image
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print("Error loading PGM image.")
        return

    height, width = img.shape
    print(f"Raw Image Dimensions: {width}x{height} pixels")

    # ROS Map standard:
    # 0 = occupied (wall)
    # 254 = free
    # ~205 = unknown

    # Thresholds
    walls = (img < 50).astype(np.uint8) * 255
    free = (img > 250).astype(np.uint8) * 255
    known = ((img < 50) | (img > 250)).astype(np.uint8) * 255

    # 1. Bounding Box of Known Area
    y_idx, x_idx = np.where(known == 255)
    if len(x_idx) > 0:
        min_x, max_x = np.min(x_idx), np.max(x_idx)
        min_y, max_y = np.min(y_idx), np.max(y_idx)
        
        map_width_px = max_x - min_x
        map_height_px = max_y - min_y
        
        map_width_m = map_width_px * resolution
        map_height_m = map_height_px * resolution
        
        print(f"Track Bounding Box (Known Area):")
        print(f"  Width:  {map_width_m:.2f} meters ({map_width_px} px)")
        print(f"  Height: {map_height_m:.2f} meters ({map_height_px} px)")
        
        # Area calculations
        wall_area_m2 = np.sum(walls == 255) * (resolution ** 2)
        free_area_m2 = np.sum(free == 255) * (resolution ** 2)
        total_mapped_area_m2 = wall_area_m2 + free_area_m2
        
        print(f"\nArea Statistics:")
        print(f"  Total Mapped Area: {total_mapped_area_m2:.2f} m²")
        print(f"  Free Space Area:   {free_area_m2:.2f} m²")
        print(f"  Occupied Area:     {wall_area_m2:.2f} m²")
    else:
        print("No known area found!")
        return

    # 2. Wall Thickness Estimation
    # Compute Euclidean distance transform on the walls
    # distanceTransform calculates distance to the closest zero pixel.
    # So we want walls to be 255 and everything else to be 0.
    dist = cv2.distanceTransform(walls, cv2.DIST_L2, 5)
    
    # Find local maxima in the distance transform (the "skeleton" or centerline of the walls)
    # A local maximum filter
    local_max = maximum_filter(dist, size=3)
    
    # The centerline is where the distance transform equals the local maximum (and is > 0)
    centerline = (dist == local_max) & (dist > 0)
    
    # Thickness is 2 * distance at the centerline
    thickness_px = dist[centerline] * 2.0
    thickness_m = thickness_px * resolution
    
    if len(thickness_m) > 0:
        avg_thickness = np.mean(thickness_m)
        std_thickness = np.std(thickness_m)
        max_thickness = np.max(thickness_m)
        min_thickness = np.min(thickness_m)
        
        print(f"\nWall Thickness Metrics:")
        print(f"  Average Thickness: {avg_thickness:.3f} meters")
        print(f"  Thickness Std Dev: {std_thickness:.3f} meters")
        print(f"  Max Thickness:     {max_thickness:.3f} meters")
        print(f"  Min Thickness:     {min_thickness:.3f} meters")
        
        # For a track wall, consistency is key. A low standard deviation means the SLAM map 
        # accurately registered the walls without "ghosting" or duplicated lines from loop closure errors.
        consistency_score = max(0, 100 - (std_thickness / avg_thickness) * 100)
        print(f"  Map Consistency Score: {consistency_score:.1f}% (100% is perfect uniform thickness)")
    else:
        print("\nCould not calculate wall thickness.")

if __name__ == '__main__':
    if len(sys.argv) != 3:
        yaml_path = "/home/sima/Project_Lab/Hunter_RL/src/hunter_gazebo/maps/track_map.yaml"
        pgm_path = "/home/sima/Project_Lab/Hunter_RL/src/hunter_gazebo/maps/track_map.pgm"
    else:
        yaml_path = sys.argv[1]
        pgm_path = sys.argv[2]
        
    evaluate_map(yaml_path, pgm_path)
