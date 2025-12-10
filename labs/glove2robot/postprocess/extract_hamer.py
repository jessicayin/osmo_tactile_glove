#!/usr/bin/env python3
"""
HaMeR Hand Pose Estimation Pipeline

This module provides functionality for extracting hand poses from RGB images using the HaMeR model.
It processes synchronized visual data from RealSense cameras and glove data, outputting 3D hand
meshes, keypoints, and wrist poses.

Key Features:
- Hand detection using ViT or SAM models
- HaMeR-based 3D hand pose estimation
- Point cloud registration with hand meshes
- Stereo depth processing
- Data synchronization with glove sensors

"""

import argparse
import os
import sys
import gc
import copy
import pickle
import pathlib
import imageio
import cv2
import numpy as np
import torch
import trimesh
import matplotlib.pyplot as plt
from pathlib import Path
from tqdm import tqdm
from tabulate import tabulate
from PIL import Image
import hydra
from omegaconf import DictConfig, OmegaConf
from matplotlib.animation import FuncAnimation

# Additional imports for ICP registration
import open3d as o3d
from scipy.optimize import minimize

os.environ["PATH"] = "/usr/local/cuda-12.4/bin:$PATH"
os.environ["LD_LIBRARY_PATH"] = "/usr/local/cuda-12.4/lib64:$LD_LIBRARY_PATH"

# Additional environment setup to help with SAM2 config path issues
if "PYTHONPATH" in os.environ:
    current_pythonpath = os.environ["PYTHONPATH"]
else:
    current_pythonpath = ""

# Ensure current directory is in Python path for config discovery
current_dir = os.getcwd()
if current_dir not in current_pythonpath:
    os.environ["PYTHONPATH"] = f"{current_dir}:{current_pythonpath}" if current_pythonpath else current_dir

# ============================================================================
# MEMORY MANAGEMENT UTILITIES
# ============================================================================

class CUDAOutOfMemoryError(Exception):
    """Custom exception for CUDA out of memory errors."""
    pass

def check_cuda_memory():
    """Check CUDA memory usage and print statistics."""
    if torch.cuda.is_available():
        allocated = torch.cuda.memory_allocated() / 1024**3  # GB
        reserved = torch.cuda.memory_reserved() / 1024**3   # GB
        max_allocated = torch.cuda.max_memory_allocated() / 1024**3  # GB
        print(f"CUDA Memory - Allocated: {allocated:.2f}GB, Reserved: {reserved:.2f}GB, Max: {max_allocated:.2f}GB")
        return allocated, reserved, max_allocated
    return 0, 0, 0

def cleanup_memory():
    """Comprehensive memory cleanup function."""
    # Force garbage collection
    gc.collect()
    
    # Clear CUDA cache if available
    if torch.cuda.is_available():
        torch.cuda.empty_cache()
        torch.cuda.synchronize()
    
    print("Memory cleanup completed")

def cleanup_variables(*variables):
    """Clean up specific variables and their references."""
    for var in variables:
        if var is not None:
            del var
    gc.collect()

def emergency_cleanup():
    """
    Emergency cleanup function that can be called manually or automatically.
    Use this when you want to force a complete memory cleanup.
    """
    print("Performing emergency memory cleanup...")
    
    # Force garbage collection multiple times
    for i in range(3):
        gc.collect()
    
    # Clear CUDA cache aggressively
    if torch.cuda.is_available():
        torch.cuda.empty_cache()
        torch.cuda.synchronize()
        torch.cuda.reset_peak_memory_stats()
        
        # Try to clear more aggressively
        try:
            torch.cuda.ipc_collect()
        except:
            pass
    
    print("Emergency cleanup completed")
    check_cuda_memory()

# HaMeR imports
try:
    import hamer
    print(sys.modules['hamer'].__file__)  # check location of hamer
    from hamer.vitpose_model import ViTPoseModel
    from hamer.models import HAMER, download_models, load_hamer, DEFAULT_CHECKPOINT
    from hamer.utils import recursive_to
    from hamer.datasets.vitdet_dataset import ViTDetDataset, DEFAULT_MEAN, DEFAULT_STD
    from hamer.utils.renderer import Renderer, cam_crop_to_full
    from hamer.utils.utils_detectron2 import DefaultPredictor_Lazy
    from detectron2.config import LazyConfig
except ImportError as e:
    print(f"Warning: HaMeR imports failed: {e}")

# Other imports
try:
    from glove2robot.utils.bowie_data import BowieSyncData
    from mmint_foundationstereo.stereo_offline import StereoDepthProcessor
    from glove2robot.utils.hamer_utils import *
except ImportError as e:
    print(f"Warning: Some imports failed: {e}")


def visualize_alignment_headless(aligned_mesh, original_mesh, hand_pcd, save_folder = "~/human2robot/", frame_idx=0):
    """
    Save multiple views of hand mesh and point cloud for debugging without GLFW.
    
    Args:
        aligned_mesh: Trimesh after depth-aligned translation is applied
        original_mesh: Original Trimesh before alignment
        hand_pcd: Point cloud array
    """
 
    # Set Open3D to headless mode
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
    
    # Convert to Open3D format
    aligned_mesh_o3d = o3d.geometry.TriangleMesh()
    aligned_mesh_o3d.vertices = o3d.utility.Vector3dVector(aligned_mesh.vertices)
    aligned_mesh_o3d.triangles = o3d.utility.Vector3iVector(aligned_mesh.faces)
    aligned_mesh_o3d.paint_uniform_color([0.8, 0.2, 0.2])  # Red

    # Convert to Open3D format
    original_mesh_o3d = o3d.geometry.TriangleMesh()
    original_mesh_o3d.vertices = o3d.utility.Vector3dVector(original_mesh.vertices)
    original_mesh_o3d.triangles = o3d.utility.Vector3iVector(original_mesh.faces)
    original_mesh_o3d.paint_uniform_color([0.2, 0.2, 0.8])  # Blue
    
    pcd_o3d = o3d.geometry.PointCloud()
    pcd_o3d.points = o3d.utility.Vector3dVector(hand_pcd)
    pcd_o3d.paint_uniform_color([0.2, 0.8, 0.2])  # Green
    
    # Create off-screen renderer
    render = o3d.visualization.rendering.OffscreenRenderer(800, 600)
    
    # Add geometries
    render.scene.add_geometry("mesh", original_mesh_o3d, o3d.visualization.rendering.MaterialRecord())
    render.scene.add_geometry("pointcloud", pcd_o3d, o3d.visualization.rendering.MaterialRecord())
    
    # Set up camera and lighting
    render.scene.set_lighting(o3d.visualization.rendering.Open3DScene.LightingProfile.SOFT_SHADOWS, [0, 0, -1])
    
    # Different viewpoints
    views = [
        {"name": "front", "center": [0, 0, 0], "eye": [0, 0, -0.5], "up": [0, -1, 0]},
        {"name": "side", "center": [0, 0, 0], "eye": [0.5, 0, 0], "up": [0, 0, 1]},
        {"name": "top", "center": [0, 0, 0], "eye": [0, 0.5, 0], "up": [0, 0, -1]}
    ]
    
    os.makedirs(save_folder, exist_ok=True)
    
    for view in views:
        # Set camera parameters
        render.setup_camera(60.0, view["center"], view["eye"], view["up"])
        
        # Render
        img = render.render_to_image()
        
        # Save image
        filename = f"frame_{frame_idx:04d}_{view['name']}.png"
        filepath = os.path.join(save_folder, filename)
        o3d.io.write_image(filepath, img)
        print(f"Saved view: {filepath}")



def compute_nearest_neighbors(source_points, target_points):
    """
    Compute nearest neighbors between source and target points using Open3D KDTree.
    
    Args:
        source_points: Source point array
        target_points: Target point array
        
    Returns:
        Array of distance vectors
    """
    try:
        # Create a KD-tree from the source points
        source_pcd = o3d.geometry.PointCloud()
        source_pcd.points = o3d.utility.Vector3dVector(source_points)
        source_kd_tree = o3d.geometry.KDTreeFlann(source_pcd)
        distances = []
        for target_point in target_points:
            [_, idx, _] = source_kd_tree.search_knn_vector_3d(target_point, 1)
            nearest_source_point = source_points[idx[0]]
            distances.append(nearest_source_point - target_point)
        return np.array(distances)
    except:
        # Fallback to simple distance computation if Open3D fails
        distances = []
        for target_point in target_points:
            dists = np.linalg.norm(source_points - target_point, axis=1)
            nearest_idx = np.argmin(dists)
            distances.append(source_points[nearest_idx] - target_point)
        return np.array(distances)


def objective_function(translation, source_points, target_points):
    """
    Objective function for translation optimization.
    
    Args:
        translation: Translation vector to optimize
        source_points: Source point array
        target_points: Target point array
        
    Returns:
        Sum of squared distances
    """
    translated_target_points = target_points + translation
    distances = compute_nearest_neighbors(source_points, translated_target_points)
    return np.sum(np.linalg.norm(distances, axis=1) ** 2)


def optimize_translation(source, target):
    """
    Optimize translation between source and target point clouds.
    
    Args:
        source: Source Open3D PointCloud
        target: Target Open3D PointCloud
        
    Returns:
        Optimized translation vector
    """
    try:
        # Load point clouds
        source_points = np.asarray(source.points)
        target_points = np.asarray(target.points)
        # Initial guess for translation
        initial_translation = np.zeros(3)
        # Optimize translation
        result = minimize(objective_function, initial_translation, args=(source_points, target_points))
        return result.x
    except:
        # Fallback to simple centroid difference
        source_points = np.asarray(source.points)
        target_points = np.asarray(target.points)
        return np.mean(target_points, axis=0) - np.mean(source_points, axis=0)


def perform_direct_3d_alignment(hand_mesh, hand_pcd, keypoints_3d, wrist_pose, frame_idx, wrist_keypoint_idx=0):
    """
    Align HaMeR hand mesh directly using 3D coordinates.
    
    This function assumes both keypoints_3d and hand_pcd are in the same coordinate frame
    (RGB camera frame) and performs direct 3D alignment by matching wrist positions.
    
    Args:
        hand_mesh (trimesh.Trimesh): The source HaMeR hand mesh
        hand_pcd (np.ndarray): Target point cloud of the hand in RGB camera frame
        keypoints_3d (np.ndarray): 3D keypoints from HaMeR in RGB camera frame (21 keypoints)
        wrist_pose (np.ndarray): 4x4 wrist pose matrix
        wrist_keypoint_idx (int): Index of wrist keypoint (default: 0)
        
    Returns:
        Tuple of (aligned_mesh, aligned_keypoints_3d, aligned_wrist_pose, depth_translation)
    """
    try:
        print("Performing direct 3D alignment using keypoints and point cloud...")
        
        if hand_pcd is None or len(hand_pcd) < 10:
            print("Warning: Insufficient hand point cloud data for alignment")
            return hand_mesh, keypoints_3d, wrist_pose, np.zeros(3)
        
                
        # Get depth statistics from hand point cloud
        pcd_depths = hand_pcd[:, 2]  # z-coordinates
        valid_depths = pcd_depths[np.isfinite(pcd_depths)]
        median_depth = np.median(valid_depths)
        std_depth = np.std(valid_depths)

        # Filter outliers (points beyond 2 standard deviations)
        valid_mask = np.abs(pcd_depths - median_depth) < (2 * std_depth)
        hand_pcd_filtered = hand_pcd[valid_mask]
        # Get HaMeR wrist position (3D coordinates in RGB camera frame)
        hamer_wrist_3d = keypoints_3d[wrist_keypoint_idx]  # [x, y, z]
        print(f"HaMeR wrist 3D position: ({hamer_wrist_3d[0]:.3f}, {hamer_wrist_3d[1]:.3f}, {hamer_wrist_3d[2]:.3f})")
        
        # Calculate distances from all hand points to HaMeR wrist position
        distances = np.linalg.norm(hand_pcd_filtered - hamer_wrist_3d, axis=1)
        closest_idx = np.argmin(distances)
        closest_point = hand_pcd_filtered[closest_idx]

        print(f"Closest point cloud point: ({closest_point[0]:.3f}, {closest_point[1]:.3f}, {closest_point[2]:.3f})")
        print(f"Distance to HaMeR wrist: {distances[closest_idx]:.3f}m")
        
        # Find all points within a reasonable distance of the wrist
        wrist_radius = 0.13  # 8cm radius around wrist
        nearby_mask = distances < wrist_radius
        nearby_points = hand_pcd_filtered[nearby_mask]
        if len(nearby_points) == 0:
            print(f"Warning: No points found within {wrist_radius}m of wrist. Using closest point.")
            target_wrist_z = closest_point[2]
        else:
            # take Z value only of centroid
            target_wrist_z = np.mean(nearby_points[:,2])

        print(f"Target wrist 3D position: {target_wrist_z:.3f}")

        # Calculate translation needed to align wrist positions
        translation_vector = np.zeros_like(hamer_wrist_3d)
        
        # Calculate Z translation with validation
        z_translation = target_wrist_z - hamer_wrist_3d[2]
        
        # Validate the translation makes sense (sanity checks)
        print(f"HaMeR wrist Z: {hamer_wrist_3d[2]:.3f}, Target Z: {target_wrist_z:.3f}")
        print(f"Proposed Z translation: {z_translation:.3f}")
        
        # Check for unrealistic translations (more than 50cm)
        if abs(z_translation) > 0.5:
            print(f"Warning: Large Z translation ({z_translation:.3f}m) detected!")
            print("This might indicate coordinate frame mismatch or outliers.")
            
            max_translation = 0.2  # 20cm max
            z_translation = np.sign(z_translation) * min(abs(z_translation), max_translation)
            print(f"Capped translation to: {z_translation:.3f}m")
        
        # Check coordinate frame consistency by comparing depth ranges
        hamer_depth_range = [np.min(hand_mesh.vertices[:, 2]), np.max(hand_mesh.vertices[:, 2])]
        pcd_depth_range = [np.min(hand_pcd_filtered[:, 2]), np.max(hand_pcd_filtered[:, 2])]
        
        print(f"HaMeR mesh Z range: [{hamer_depth_range[0]:.3f}, {hamer_depth_range[1]:.3f}]")
        print(f"Point cloud Z range: [{pcd_depth_range[0]:.3f}, {pcd_depth_range[1]:.3f}]")
        
        # Check if ranges have reasonable overlap (accounting for translation)
        translated_hamer_range = [hamer_depth_range[0] + z_translation, hamer_depth_range[1] + z_translation]
        overlap_check = not (translated_hamer_range[1] < pcd_depth_range[0] or 
                            translated_hamer_range[0] > pcd_depth_range[1])
        
        if not overlap_check:
            print("Warning: Translated mesh and point cloud Z ranges don't overlap!")
            print("This suggests possible coordinate frame inconsistency.")
        
        # Apply translation only in Z direction (keeping X,Y unchanged for now)
        translation_vector[0] = 0.0  # No X translation
        translation_vector[1] = 0.0  # No Y translation  
        translation_vector[2] = z_translation
        
        print(f"Translation vector: ({translation_vector[0]:.3f}, {translation_vector[1]:.3f}, {translation_vector[2]:.3f})")
        
        # Apply translation to mesh
        aligned_mesh = copy.deepcopy(hand_mesh)
        aligned_mesh.vertices += translation_vector
        
        # Apply translation to keypoints
        aligned_keypoints_3d = keypoints_3d + translation_vector
        
        # Apply translation to wrist pose
        aligned_wrist_pose = copy.deepcopy(wrist_pose)
        aligned_wrist_pose[:3, 3] += translation_vector
        
        # Verification
        final_wrist_3d = aligned_keypoints_3d[wrist_keypoint_idx]
        print(f"Final aligned wrist position: ({final_wrist_3d[0]:.3f}, {final_wrist_3d[1]:.3f}, {final_wrist_3d[2]:.3f})")
        
        
        # Clean up temporary variables to free memory
        cleanup_variables(hand_pcd_filtered, nearby_points)
        
        return aligned_mesh, aligned_keypoints_3d, aligned_wrist_pose, translation_vector
        
    except Exception as e:
        print(f"Direct 3D alignment failed: {e}")
        # Check if it's a CUDA memory error
        
        # Clean up any allocated variables before returning
        cleanup_variables(hand_pcd_filtered if 'hand_pcd_filtered' in locals() else None,
                         nearby_points if 'nearby_points' in locals() else None,
                         translation_vector if 'translation_vector' in locals() else None)
        
        return hand_mesh, keypoints_3d, wrist_pose, np.zeros(3)


def perform_simplified_depth_alignment(hand_mesh, keypoints_3d, wrist_pose, fs_depth, keypoints_2d, frame_idx,
                                      crop_info=None, wrist_keypoint_idx=0):
    """
    Simplified depth alignment using fs_depth directly.
    
    Since fs_depth is already aligned with RGB camera frame (same as HaMeR keypoints),
    we can directly lookup depth values without complex coordinate transformations.
    
    When crop_info is provided, keypoints_2d are transformed from full image coordinates
    to cropped image coordinates before depth lookup.
    
    Args:
        hand_mesh (trimesh.Trimesh): The source HaMeR hand mesh
        keypoints_3d (np.ndarray): 3D keypoints from HaMeR (21 keypoints)
        wrist_pose (np.ndarray): 4x4 wrist pose matrix
        fs_depth (np.ndarray): FoundationStereo depth map aligned with RGB image
        keypoints_2d (np.ndarray): 2D keypoints in RGB image coordinates
        frame_idx (int): Current frame index for debugging
        crop_info (dict): Optional crop information for coordinate transformation
        wrist_keypoint_idx (int): Index of wrist keypoint (default: 0)
        
    Returns:
        Tuple of (aligned_mesh, aligned_keypoints_3d, aligned_wrist_pose, depth_translation)
    """
 
    try:
        # Get wrist keypoint 2D coordinates (in full RGB image coordinates)
        wrist_2d_full = keypoints_2d[wrist_keypoint_idx]  # [x, y]
        
        # Transform to cropped coordinates if crop_info is available
        if crop_info is not None:
            # Transform from full image coordinates to cropped coordinates
            crop_x_offset = crop_info['offset'][0]
            crop_y_offset = crop_info['offset'][1]
            
            wrist_2d_cropped = np.array([
                wrist_2d_full[0] - crop_x_offset,
                wrist_2d_full[1] - crop_y_offset
            ])
            
            # Check if the transformed wrist is within the cropped region
            crop_height, crop_width = fs_depth.shape
            if (wrist_2d_cropped[0] < 0 or wrist_2d_cropped[0] >= crop_width or
                wrist_2d_cropped[1] < 0 or wrist_2d_cropped[1] >= crop_height):
                print(f"Warning: Wrist keypoint ({wrist_2d_full[0]:.1f}, {wrist_2d_full[1]:.1f}) "
                      f"is outside cropped region after transformation to "
                      f"({wrist_2d_cropped[0]:.1f}, {wrist_2d_cropped[1]:.1f})")
                print(f"Crop region: {crop_width}x{crop_height}, offset: ({crop_x_offset}, {crop_y_offset})")
                return hand_mesh, keypoints_3d, wrist_pose, np.zeros(3)
            
            wrist_2d = wrist_2d_cropped
            keypoints_2d_viz = keypoints_2d.copy()
            keypoints_2d_viz[:, 0] -= crop_info['offset'][0]
            keypoints_2d_viz[:, 1] -= crop_info['offset'][1]
        else:
            # No cropping, use coordinates directly
            wrist_2d = wrist_2d_full
            print(f"Using full image wrist coordinates: ({wrist_2d[0]:.1f}, {wrist_2d[1]:.1f})")
            
        
        # Direct depth lookup (much simpler!)
        wrist_x = int(np.round(wrist_2d[0]))
        wrist_y = int(np.round(wrist_2d[1]))
        height, width = fs_depth.shape[:2]
        
        # Sample depth in neighborhood for robustness
        neighborhood_size = 10
        half_size = neighborhood_size // 2
        
        y_min = max(0, wrist_y - half_size)
        y_max = min(height, wrist_y + half_size + 1)
        x_min = max(0, wrist_x - half_size)
        x_max = min(width, wrist_x + half_size + 1)
        
        neighborhood_depths = fs_depth[y_min:y_max, x_min:x_max]
        
        # Filter valid depths
        valid_depths = neighborhood_depths[
            (neighborhood_depths > 0) & 
            (neighborhood_depths < 5.0) &  # Reasonable hand distance
            np.isfinite(neighborhood_depths)
        ]

        if len(valid_depths) > 0:
            target_depth = np.median(valid_depths)
            
            # Get current HaMeR wrist depth
            hamer_wrist_depth = keypoints_3d[wrist_keypoint_idx, 2]
            
            # Calculate translation
            depth_translation = target_depth - hamer_wrist_depth
            translation_vector = np.array([0, 0, depth_translation])
            
            print(f"Direct depth alignment: {hamer_wrist_depth:.3f} -> {target_depth:.3f} (Δ: {depth_translation:.3f})")
            
            # Sanity check - reject unrealistic translations
            if abs(depth_translation) > 0.5:  # More than 50cm
                print(f"Warning: Large depth translation ({depth_translation:.3f}m) detected, capping to ±20cm")
                max_translation = 0.2
                depth_translation = np.sign(depth_translation) * min(abs(depth_translation), max_translation)
                translation_vector = np.array([0, 0, depth_translation])
            
            # Apply translation
            aligned_mesh = copy.deepcopy(hand_mesh)
            aligned_mesh.vertices += translation_vector
            
            aligned_keypoints_3d = keypoints_3d + translation_vector
            aligned_wrist_pose = copy.deepcopy(wrist_pose)
            aligned_wrist_pose[:3, 3] += translation_vector
            
            # print(f"Applied simplified depth correction: {translation_vector}")
            
            # Clean up temporary variables
            cleanup_variables(neighborhood_depths, valid_depths)
            
            return aligned_mesh, aligned_keypoints_3d, aligned_wrist_pose, translation_vector
        
        else:
            print("No valid depth values found in wrist neighborhood")
            # Clean up temporary variables
            cleanup_variables(neighborhood_depths, valid_depths if 'valid_depths' in locals() else None)
            return hand_mesh, keypoints_3d, wrist_pose, np.zeros(3)
        
    except Exception as e:
        print(f"Simplified depth alignment failed: {e}")
        
        # Clean up any allocated variables before returning
        cleanup_variables(neighborhood_depths if 'neighborhood_depths' in locals() else None,
                         valid_depths if 'valid_depths' in locals() else None)
        
        return hand_mesh, keypoints_3d, wrist_pose, np.zeros(3)


def perform_depth_based_alignment(hand_mesh, hand_pcd, keypoints_3d, wrist_pose, wrist_keypoint_idx=0, 
                                fs_depth=None, keypoints_2d=None):
    """
    Align HaMeR hand mesh to FoundationStereo depth using wrist keypoint location.
    
    This function uses the FoundationStereo depth map to get the depth value at the wrist keypoint
    location in the camera frame, then translates the HaMeR mesh to match this depth.
    
    Args:
        hand_mesh (trimesh.Trimesh): The source HaMeR hand mesh
        hand_pcd (np.ndarray): Target point cloud of the hand from FoundationStereo (fallback)
        keypoints_3d (np.ndarray): 3D keypoints from HaMeR (21 keypoints)
        wrist_pose (np.ndarray): 4x4 wrist pose matrix
        wrist_keypoint_idx (int): Index of wrist keypoint (default: 0)
        fs_depth (np.ndarray): FoundationStereo depth map aligned with RGB image
        keypoints_2d (np.ndarray): 2D keypoints in image coordinates
        
    Returns:
        Tuple of (aligned_mesh, aligned_keypoints_3d, aligned_wrist_pose, depth_translation)
    """
    try:
        print("Performing wrist-based depth alignment...")
        
        target_depth = None
        
        # Method 1: Use FoundationStereo depth at wrist keypoint location (preferred)
        if fs_depth is not None and keypoints_2d is not None:
            try:
                # Get wrist keypoint 2D coordinates in RGB camera frame
                wrist_2d_rgb = keypoints_2d[wrist_keypoint_idx]  # [x, y] in RGB camera
                
                print(f"Original wrist 2D coordinates (RGB frame): ({wrist_2d_rgb[0]:.1f}, {wrist_2d_rgb[1]:.1f})")
                
                # Transform from RGB camera frame to infrared camera frame
                # First convert 2D RGB point to 3D ray using RGB intrinsics
                fx_rgb, fy_rgb = COLOR_INTRINSIC[0, 0], COLOR_INTRINSIC[1, 1]
                cx_rgb, cy_rgb = COLOR_INTRINSIC[0, 2], COLOR_INTRINSIC[1, 2]
                
                # Create a 3D ray in RGB camera frame (at unit depth)
                ray_rgb = np.array([
                    (wrist_2d_rgb[0] - cx_rgb) / fx_rgb,
                    (wrist_2d_rgb[1] - cy_rgb) / fy_rgb,
                    1.0
                ])
                
                # Convert extrinsics from [x,y,z,qx,qy,qz,qw] to 4x4 transformation matrix
                t_rgb_to_ir = EXTRINSICS[:3]  # translation
                q_rgb_to_ir = EXTRINSICS[3:]  # quaternion [qx, qy, qz, qw]
                
                # Convert quaternion to rotation matrix
                def quaternion_to_rotation_matrix(q):
                    """Convert quaternion [qx, qy, qz, qw] to 3x3 rotation matrix."""
                    qx, qy, qz, qw = q
                    R = np.array([
                        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
                        [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
                        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
                    ])
                    return R
                
                R_rgb_to_ir = quaternion_to_rotation_matrix(q_rgb_to_ir)
                
                # Create 4x4 transformation matrix from RGB to IR
                T_rgb_to_ir = np.eye(4)
                T_rgb_to_ir[:3, :3] = R_rgb_to_ir
                T_rgb_to_ir[:3, 3] = t_rgb_to_ir
                
                # Transform ray from RGB frame to IR frame
                ray_rgb_homo = np.append(ray_rgb, 1)  # homogeneous coordinates
                ray_ir_homo = T_rgb_to_ir @ ray_rgb_homo
                ray_ir = ray_ir_homo[:3]
                
                # Project ray back to 2D in IR camera frame using DEPTH_INTRINSIC
                fx_ir, fy_ir = DEPTH_INTRINSIC[0, 0], DEPTH_INTRINSIC[1, 1]
                cx_ir, cy_ir = DEPTH_INTRINSIC[0, 2], DEPTH_INTRINSIC[1, 2]
                
                # Project to 2D IR coordinates
                wrist_x_ir = (ray_ir[0] / ray_ir[2]) * fx_ir + cx_ir
                wrist_y_ir = (ray_ir[1] / ray_ir[2]) * fy_ir + cy_ir
                
                # Convert to integer pixel coordinates
                wrist_x = int(np.round(wrist_x_ir))
                wrist_y = int(np.round(wrist_y_ir))
                
                # Ensure coordinates are within image bounds
                height, width = fs_depth.shape[:2]
                wrist_x = np.clip(wrist_x, 0, width - 1)
                wrist_y = np.clip(wrist_y, 0, height - 1)
                
                print(f"Transformed wrist 2D coordinates (IR frame): ({wrist_x}, {wrist_y})")
                print(f"FoundationStereo depth map shape: {fs_depth.shape}")
                
                # Sample depth in a small neighborhood around the wrist to get robust estimate
                neighborhood_size = 10  # 10x10 pixel neighborhood
                half_size = neighborhood_size // 2
                
                # Define neighborhood bounds
                y_min = max(0, wrist_y - half_size)
                y_max = min(height, wrist_y + half_size + 1)
                x_min = max(0, wrist_x - half_size)
                x_max = min(width, wrist_x + half_size + 1)
                
                # Extract depth values in neighborhood
                neighborhood_depths = fs_depth[y_min:y_max, x_min:x_max]
                
                # Filter out invalid depth values (typically 0 or NaN)
                valid_depths = neighborhood_depths[
                    (neighborhood_depths > 0) & 
                    (neighborhood_depths < 10.0) &  # Remove unrealistic far depths
                    np.isfinite(neighborhood_depths)
                ]
                
                if len(valid_depths) > 0:
                    # Use median for robustness against outliers
                    depth_value = np.median(valid_depths)
                    print(f"Found depth value at wrist location: {depth_value:.3f}m")
                    
                    # Convert 2D IR coordinates + depth to 3D camera coordinates in IR frame
                    # Using infrared camera intrinsics to project to 3D space
                    fx_ir, fy_ir = DEPTH_INTRINSIC[0, 0], DEPTH_INTRINSIC[1, 1]
                    cx_ir, cy_ir = DEPTH_INTRINSIC[0, 2], DEPTH_INTRINSIC[1, 2]
                    
                    # Convert to 3D IR camera coordinates
                    wrist_3d_x_ir = (wrist_x - cx_ir) * depth_value / fx_ir
                    wrist_3d_y_ir = (wrist_y - cy_ir) * depth_value / fy_ir
                    wrist_3d_z_ir = depth_value;
                    
                    print(f"3D wrist coordinates in IR frame: ({wrist_3d_x_ir:.3f}, {wrist_3d_y_ir:.3f}, {wrist_3d_z_ir:.3f})")
                    
                    # Transform back to RGB camera frame for alignment with HaMeR keypoints
                    # Use inverse transformation (IR to RGB)
                    T_ir_to_rgb = np.linalg.inv(T_rgb_to_ir)
                    wrist_3d_ir_homo = np.array([wrist_3d_x_ir, wrist_3d_y_ir, wrist_3d_z_ir, 1])
                    wrist_3d_rgb_homo = T_ir_to_rgb @ wrist_3d_ir_homo
                    wrist_3d_rgb = wrist_3d_rgb_homo[:3]
                    
                    target_depth = wrist_3d_rgb[2]  # Z coordinate in RGB camera frame
                    
                    print(f"3D wrist coordinates in RGB frame: ({wrist_3d_rgb[0]:.3f}, {wrist_3d_rgb[1]:.3f}, {wrist_3d_rgb[2]:.3f})")
                    print(f"Target depth (Z-coordinate in RGB frame): {target_depth:.3f}m")
                else:
                    print("No valid depth values found at wrist location")
                    
            except Exception as e:
                print(f"Error extracting depth at wrist location: {e}")
        
        # Method 2: Fallback to average hand point cloud depth
        if target_depth is None and hand_pcd is not None:
            print("Falling back to hand point cloud average depth...")
            
            if len(hand_pcd) < 10:
                print("Warning: Hand point cloud has too few points for reliable depth estimation")
                return hand_mesh, keypoints_3d, wrist_pose, np.zeros(3)

            # Get depth statistics from hand point cloud
            pcd_depths = hand_pcd[:, 2]  # z-coordinates
            avg_depth = np.mean(pcd_depths)
            median_depth = np.median(pcd_depths)
            std_depth = np.std(pcd_depths)
            
            # Filter outliers (points beyond 2 standard deviations)
            valid_mask = np.abs(pcd_depths - median_depth) < (2 * std_depth)
            filtered_depths = pcd_depths[valid_mask]
            
            if len(filtered_depths) > 5:
                target_depth = np.mean(filtered_depths)
            else:
                target_depth = avg_depth
            
            print(f"Point cloud depth stats - Mean: {avg_depth:.3f}, Median: {median_depth:.3f}, Target: {target_depth:.3f}")
        
        # Method 3: No depth information available
        if target_depth is None:
            print("Warning: No depth information available for alignment")
            return hand_mesh, keypoints_3d, wrist_pose, np.zeros(3)
        
        # Get current wrist depth from HaMeR prediction
        hamer_wrist_depth = keypoints_3d[wrist_keypoint_idx, 2]
        print(f"HaMeR wrist depth: {hamer_wrist_depth:.3f}")
        print(f"Target wrist depth: {target_depth:.3f}")
        
        # Calculate depth translation needed
        depth_translation = target_depth - hamer_wrist_depth
        translation_vector = np.array([0, 0, depth_translation])
        
        print(f"Applying depth translation: {translation_vector}")
        
        # Apply translation to mesh
        aligned_mesh = copy.deepcopy(hand_mesh)
        aligned_mesh.vertices += translation_vector
        
        # Apply translation to keypoints
        aligned_keypoints_3d = keypoints_3d + translation_vector
        
        # Apply translation to wrist pose
        aligned_wrist_pose = copy.deepcopy(wrist_pose)
        aligned_wrist_pose[:3, 3] += translation_vector
        
        # Print verification
        print(f"Original wrist depth: {keypoints_3d[wrist_keypoint_idx, 2]:.3f}")
        print(f"Aligned wrist depth: {aligned_keypoints_3d[wrist_keypoint_idx, 2]:.3f}")
        print(f"Target depth: {target_depth:.3f}")
        print(f"Depth translation applied: {depth_translation:.3f}")

        # Visualize alignment if possible
        try:
            visualize_alignment_headless(aligned_mesh, hand_mesh, hand_pcd if hand_pcd is not None else np.array([]))
        except Exception as viz_e:
            print(f"Visualization failed: {viz_e}")
        
        return aligned_mesh, aligned_keypoints_3d, aligned_wrist_pose, translation_vector
        
    except Exception as e:
        print(f"Depth-based alignment failed: {e}")
        return hand_mesh, keypoints_3d, wrist_pose, np.zeros(3)



# ============================================================================
# CROPPED STEREO PROCESSING FUNCTIONS (OPTIONAL OPTIMIZATION)
# ============================================================================

def crop_images_for_stereo(left_ir, right_ir, rgb_img, bbox, padding=50):
    """
    Crop stereo images to a specific bounding box for faster processing.
    
    Args:
        left_ir: Left infrared image
        right_ir: Right infrared image  
        rgb_img: RGB image (for reference)
        bbox: Bounding box [x_min, y_min, x_max, y_max] in RGB coordinates
        padding: Additional padding around the bounding box
        
    Returns:
        Tuple of (cropped_left, cropped_right, cropped_rgb, crop_info)
    """
    try:
        # Add padding to bounding box
        x_min, y_min, x_max, y_max = bbox
        x_min = max(0, x_min - padding)
        y_min = max(0, y_min - padding)
        x_max = min(rgb_img.shape[1], x_max + padding)
        y_max = min(rgb_img.shape[0], y_max + padding)
        
        # Since StereoDepthProcessor.process_images() handles coordinate transformations internally,
        # we can use the same crop region for all images (RGB and IR are roughly aligned)
        # The processor will handle any misalignment between RGB and IR cameras
        
        # Ensure crop coordinates are valid for IR images too
        ir_x_min = max(0, min(x_min, left_ir.shape[1] - 1))
        ir_y_min = max(0, min(y_min, left_ir.shape[0] - 1))
        ir_x_max = max(ir_x_min + 1, min(x_max, left_ir.shape[1]))
        ir_y_max = max(ir_y_min + 1, min(y_max, left_ir.shape[0]))
        
        # Crop images using the same region
        cropped_left = left_ir[ir_y_min:ir_y_max, ir_x_min:ir_x_max]
        cropped_right = right_ir[ir_y_min:ir_y_max, ir_x_min:ir_x_max]
        cropped_rgb = rgb_img[y_min:y_max, x_min:x_max]
        
        # Store crop information for intrinsic adjustment
        crop_info = {
            'offset': (x_min, y_min),  # Same offset for RGB and IR since they're aligned
            'bbox': (x_min, y_min, x_max, y_max)
        }
        
        # print(f"Cropped images - RGB: {cropped_rgb.shape}, IR: {cropped_left.shape}")
        
        return cropped_left, cropped_right, cropped_rgb, crop_info
        
    except Exception as e:
        print(f"Error cropping images: {e}")
        # Return original images if cropping fails
        return left_ir, right_ir, rgb_img, None


def adjust_camera_intrinsics_for_crop(crop_info):
    """
    Adjust camera intrinsics for cropped images.
    Since RGB and IR cameras are roughly aligned and StereoDepthProcessor handles
    coordinate transformations, we adjust both intrinsics by the same offset.
    
    Args:
        crop_info: Crop information from crop_images_for_stereo
        
    Returns:
        Tuple of (adjusted_color_intrinsic, adjusted_depth_intrinsic)
    """
    if crop_info is None:
        return COLOR_INTRINSIC, DEPTH_INTRINSIC
    
    # Use the same offset for both cameras since they're roughly aligned
    # and StereoDepthProcessor handles fine alignment internally
    x_offset, y_offset = crop_info['offset']
    
    # Adjust RGB camera intrinsics
    adjusted_color_intrinsic = COLOR_INTRINSIC.copy()
    adjusted_color_intrinsic[0, 2] -= x_offset  # cx
    adjusted_color_intrinsic[1, 2] -= y_offset  # cy
    
    # Adjust IR camera intrinsics by the same amount
    adjusted_depth_intrinsic = DEPTH_INTRINSIC.copy()
    adjusted_depth_intrinsic[0, 2] -= x_offset  # cx
    adjusted_depth_intrinsic[1, 2] -= y_offset  # cy
    
    return adjusted_color_intrinsic, adjusted_depth_intrinsic


# ============================================================================
# CAMERA CALIBRATION CONSTANTS (loaded from config)
# ============================================================================

# These will be initialized in main() from the configuration file
COLOR_INTRINSIC = None
DEPTH_INTRINSIC = None
EXTRINSICS = None
EXTRINSICS_wTc_VEC = None
BASELINE = None
LIGHT_BLUE = None

def initialize_camera_constants(cfg):
    """Initialize camera calibration constants from configuration."""
    global COLOR_INTRINSIC, DEPTH_INTRINSIC, EXTRINSICS, EXTRINSICS_wTc_VEC, BASELINE, LIGHT_BLUE
    
    # Convert config lists to numpy arrays
    COLOR_INTRINSIC = np.array(cfg.camera.color_intrinsic)
    DEPTH_INTRINSIC = np.array(cfg.camera.depth_intrinsic)
    EXTRINSICS = np.array(cfg.camera.extrinsics)
    EXTRINSICS_wTc_VEC = np.array(cfg.camera.extrinsics_wtc_vec)
    BASELINE = cfg.camera.baseline
    LIGHT_BLUE = tuple(cfg.visualization.light_blue)
    
    print("Camera calibration constants loaded from configuration:")
    print(f"COLOR_INTRINSIC shape: {COLOR_INTRINSIC.shape}")
    print(f"DEPTH_INTRINSIC shape: {DEPTH_INTRINSIC.shape}")
    print(f"EXTRINSICS shape: {EXTRINSICS.shape}")
    print(f"BASELINE: {BASELINE}")


# ============================================================================
# CORE CLASSES
# ============================================================================

class GlovePoseTracker:
    """
    Main class for tracking hand poses from glove and visual data.
    
    This class integrates multiple components:
    - HaMeR model for 3D hand pose estimation
    - SAM/ViT for hand detection
    - Stereo depth processing
    - Data synchronization between cameras and glove sensors
    """

    def __init__(self, filepath: str, out_folder: str, hamer_ckpt: str, cfg: DictConfig):
        """
        Initialize the glove pose tracker.
        
        Args:
            filepath: Path to the input data file
            out_folder: Output directory for processed data
            hamer_ckpt: Path to HaMeR model checkpoint
            cfg: Configuration object containing processing parameters
        """
        self.out_folder = out_folder
        self.filepath = filepath
        self.hamer_ckpt = hamer_ckpt
        self.cfg = cfg
        
        self.start_idx = cfg.get('start_index')
        self.end_idx = cfg.get('end_index')
        # Initialize tracking statistics
        self.data_stats = [
            ["Filename", "Realsense Frames", "Hand Frames", "No Hand Frames",
             "Hand Error Loss", "Bowie Frames", "Key Error Frames", "Key Error Loss"]
        ]
        
        # Initialize model components
        self.hamer_model = None
        self.hamer_cfg = None
        self.device = None
        self.hamer_detector = None
        self.hamer_renderer = None
        self.cpm = None
        self.langsam = None
        self.processor = None
        self.default_dir = os.getcwd()
        print(f"GlovePoseTracker starting directory: {self.default_dir}")

        # Initialize memory monitoring
        print("Initial memory state:")
        check_cuda_memory()

        self.open_bowie_sync_pkl(self.filepath)
        # Initialize all components
        self._init_langsam()
        self._init_hamer()
        self._init_stereo_processor()
        
        # Report final detection method status
        print("\n" + "="*50)
        print("DETECTION METHOD STATUS:")
        print(f"Configured bb_model: {cfg.get('bb_model', 'unknown')}")
        print(f"LangSAM available: {self.langsam is not None}")
        if cfg.get('bb_model') == 'sam' and self.langsam is None:
            print("⚠️  WARNING: SAM detection requested but LangSAM failed to initialize!")
            print("⚠️  Will fall back to ViT detection instead.")
        elif cfg.get('bb_model') == 'sam' and self.langsam is not None:
            print("✅ SAM detection ready and available")
        elif cfg.get('bb_model') == 'vit':
            print("✅ ViT detection configured and will be used")
        print("="*50 + "\n")

    def open_bowie_sync_pkl(self, path, verbose=False):
        #one trial for now, for rosbag
        """
        input: path to ros2 bag file folder that contains extracted pkl files from .db3 file
        outputs: realsense_ts, realsense_color, realsense_depth, realsense_ir1, realsense_ir2, bowie_mag_data
        """
        # rs_ts = np.asarray(pickle.load(open(path+ "/realsense_ts.pkl", "rb")))
        self.rs_color = np.asarray(pickle.load(open(path + "rgbs_aligned.pkl", "rb")))
        # self.rs_depth = np.asarray(pickle.load(open(path + "depth_aligned.pkl", "rb")))
        self.left_ir = np.asarray(pickle.load(open(path + "left_ir_aligned.pkl", "rb")))
        self.right_ir = np.asarray(pickle.load(open(path + "right_ir_aligned.pkl", "rb")))
        bowie_data = np.asarray(pickle.load(open(path + "synced_mags_aligned.pkl", "rb")), dtype="object")
        bowie_data = bowie_data.reshape((bowie_data.shape[0],10,4))
        self.bowie_data = bowie_data[:,:,1:]
        # self.bowie_data = bowie_data[:,1] 
    
    
    def _init_hamer(self):
        """Initialize HaMeR model, detector, and renderer components."""
        print("Initializing HaMeR model...")
        
        # Store current directory and change to HaMeR directory
        original_dir = os.getcwd()
        hamer_dir = os.path.dirname(os.path.dirname(os.path.abspath(hamer.__file__)))
        os.chdir(hamer_dir)
        
        try:
            # Download and load HaMeR model
            download_models(self.cfg.paths.CACHE_DIR_HAMER)
            model, model_cfg = load_hamer(self.hamer_ckpt)
            
            # Setup device and model
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            self.hamer_model = model.to(self.device)
            self.hamer_model.eval()
            self.hamer_cfg = model_cfg
            
            # Setup ViTDet body detector
            cfg_path = Path(hamer.__file__).parent / "configs" / "cascade_mask_rcnn_vitdet_h_75ep.py"
            detectron2_cfg = LazyConfig.load(str(cfg_path))
            detectron2_cfg.train.init_checkpoint = (
                "https://dl.fbaipublicfiles.com/detectron2/ViTDet/COCO/"
                "cascade_mask_rcnn_vitdet_h/f328730692/model_final_f05665.pkl"
            )
            
            # Set detection thresholds
            for i in range(3):
                detectron2_cfg.model.roi_heads.box_predictors[i].test_score_thresh = 0.25
            
            self.hamer_detector = DefaultPredictor_Lazy(detectron2_cfg)
            
            # Setup keypoint detector and renderer
            self.cpm = ViTPoseModel(self.device)
            self.hamer_renderer = Renderer(model_cfg, faces=model.mano.faces)
            
            print("HaMeR initialization complete")
            
        except Exception as e:
            print(f"Error initializing HaMeR: {e}")
            raise
        finally:
            # Always return to original directory
            os.chdir(original_dir)
            print(f"Restored working directory to: {os.getcwd()}")
    
    def _init_langsam(self):
        """Initialize LangSAM model for semantic segmentation."""
        # Ensure we're in the correct directory for LangSAM initialization
        current_dir = os.getcwd()
        os.chdir(self.default_dir)
        
        print(f"Initializing LangSAM from directory: {os.getcwd()}")
        
        try:
            from lang_sam import LangSAM
            print("Initializing LangSAM...")
            
            # Try different initialization approaches
            initialization_methods = [
                lambda: LangSAM(sam_type="sam2.1_hiera_small"),
                lambda: LangSAM(sam_type="sam2.1_hiera_base"),  
                lambda: LangSAM(sam_type="sam2_hiera_small"),
                lambda: LangSAM(),  # Default
            ]
            
            for i, init_method in enumerate(initialization_methods):
                try:
                    print(f"Trying LangSAM initialization method {i+1}...")
                    self.langsam = init_method()
                    print(f"LangSAM initialized successfully with method {i+1}: {self.langsam}")
                    print("LangSAM initialization complete")
                    return
                except Exception as e:
                    print(f"LangSAM initialization method {i+1} failed: {e}")
                    continue
            
            # If all methods fail
            raise RuntimeError("All LangSAM initialization methods failed")
            
        except Exception as e:
            print(f"Error initializing LangSAM: {e}")
            print("LangSAM initialization failed - setting to None")
            self.langsam = None
            # Don't raise the exception, just continue without LangSAM
            
        finally:
            # Always restore the directory
            os.chdir(current_dir)

    def _init_stereo_processor(self):
        """Initialize stereo depth processor."""
        try:
            print("Initializing stereo depth processor...")
            self.processor = StereoDepthProcessor(
                COLOR_INTRINSIC, DEPTH_INTRINSIC, EXTRINSICS, BASELINE
            )
            print("Stereo processor initialization complete")
        except Exception as e:
            print(f"Warning: Stereo processor initialization failed: {e}")
            self.processor = None
    
    def _detect_hands_vit(self, img_cv2: np.ndarray) -> tuple:
        """
        Detect hands using ViT-based human detection and keypoint estimation.
        
        Args:
            img_cv2: Input image in BGR format
            
        Returns:
            Tuple of (bounding_boxes, is_right_hand_flags) or ([], []) if no hands detected
        """
        # Detect humans in image
        det_out = self.hamer_detector(img_cv2)
        det_instances = det_out["instances"]
        
        # Filter for humans with sufficient confidence
        valid_idx = (det_instances.pred_classes == 0) & (det_instances.scores > 0.5)
        pred_bboxes = det_instances.pred_boxes.tensor[valid_idx].cpu().numpy()
        pred_scores = det_instances.scores[valid_idx].cpu().numpy()
        
        if len(pred_bboxes) == 0:
            return [], []
        
        # Detect human keypoints
        vitposes_out = self.cpm.predict_pose(
            img_cv2, [np.concatenate([pred_bboxes, pred_scores[:, None]], axis=1)]
        )
        
        bboxes = []
        is_right = []
        
        # Extract hand bounding boxes from keypoint detections
        for vitposes in vitposes_out:
            # Focus on right hand keypoints (ignoring left hands)
            right_hand_keyp = vitposes["keypoints"][-21:]
            
            # Check if enough valid keypoints are detected
            valid = right_hand_keyp[:, 2] > 0.5
            if sum(valid) > 3:
                # Calculate bounding box from keypoints
                valid_keyp = right_hand_keyp[valid]
                x_min, y_min = np.min(valid_keyp[:, :2], axis=0)
                x_max, y_max = np.max(valid_keyp[:, :2], axis=0)
                
                # Add padding to bounding box
                padding = 20
                bbox = [
                    max(0, x_min - padding),
                    max(0, y_min - padding),
                    min(img_cv2.shape[1], x_max + padding),
                    min(img_cv2.shape[0], y_max + padding)
                ]
                
                bboxes.append(bbox)
                is_right.append(True)  # Assuming right hand
        
        return bboxes, is_right
    
    def _detect_hands_sam(self, img_cv2: np.ndarray) -> tuple:
        """
        Detect hands using SAM-based semantic segmentation.
        
        Args:
            img_cv2: Input image in BGR format
            
        Returns:
            Tuple of (bounding_boxes, is_right_hand_flags, mask) or ([], [], None) if no hands detected
        """
        if self.langsam is None:
            if self.allow_detection_fallback:
                print("ERROR: LangSAM not available but bb_model='sam' was requested!")
                print("This means LangSAM initialization failed during startup.")
                print("Falling back to ViT detection, but you should fix LangSAM setup for proper SAM detection.")
                return self._detect_hands_vit(img_cv2)
            else:
                raise RuntimeError(
                    "LangSAM is not available but bb_model='sam' was requested and "
                    "allow_detection_fallback=False. Please either:\n"
                    "1. Fix LangSAM initialization issues, or\n"
                    "2. Set bb_model='vit' in config, or\n"
                    "3. Set allow_detection_fallback=True in config"
                )
        
        try:
            # Convert image to RGB for SAM
            results = self.langsam.predict(
                [Image.fromarray(cv2.cvtColor(img_cv2, cv2.COLOR_BGR2RGB))], ["glove."]
            )
            
            if len(results) > 0 and np.asarray(results[0]["masks"]).any():
                # Extract mask and compute bounding box
                mask = (results[0]["masks"].transpose(1, 2, 0).squeeze() * 255).astype(np.uint8)
                jidx, iidx = np.where(results[0]["masks"][0] == 1)
                
                if len(iidx) > 0 and len(jidx) > 0:
                    bbox = [np.min(iidx), np.min(jidx), np.max(iidx), np.max(jidx)]
                    return [bbox], [True], mask  # Assuming right hand
            
            print("No glove mask found")
            return [], [], None
            
        except Exception as e:
            print(f"Error in SAM detection: {e}")
            return [], [], None
    
    def _run_hamer_inference(self, img_cv2: np.ndarray, boxes: np.ndarray, 
                           right_flags: np.ndarray, scaled_focal_length: float) -> dict:
        """
        Run HaMeR inference on detected hand regions.
        
        Args:
            img_cv2: Input image in BGR format
            boxes: Hand bounding boxes [N, 4]
            right_flags: Right hand flags [N]
            scaled_focal_length: Scaled focal length for camera
            
        Returns:
            Dictionary containing prediction results
        """
        # Create dataset for HaMeR
        dataset = ViTDetDataset(self.hamer_cfg, img_cv2, boxes, right_flags, rescale_factor=2)
        dataloader = torch.utils.data.DataLoader(dataset, batch_size=8, shuffle=False, num_workers=0)
        results = {
            'vertices': [],
            'keypoints_2d': [],
            'keypoints_3d': [],
            'wrist_poses': [],
            'meshes': [],
            'camera_translations': []
        }
        
        for batch in dataloader:
            batch = recursive_to(batch, self.device)
            with torch.no_grad():
                out = self.hamer_model(batch)
            
            # Process camera parameters
            multiplier = 2 * batch["right"] - 1
            pred_cam = out["pred_cam"]
            pred_cam[:, 1] = multiplier * pred_cam[:, 1]
            
            # Convert camera coordinates to full image coordinates
            box_center = batch["box_center"].float()
            box_size = batch["box_size"].float()
            img_size = batch["img_size"].float()
            
            pred_cam_t_full = cam_crop_to_full(
                pred_cam, box_center, box_size, img_size, scaled_focal_length
            ).detach().cpu().numpy()
            
            # Process each detection in the batch
            batch_size = batch["img"].shape[0]
            for n in range(batch_size):
                # Extract vertices and apply hand orientation
                verts = out["pred_vertices"][n].detach().cpu().numpy()
                is_right = batch["right"][n].cpu().numpy()
                verts[:, 0] = (2 * is_right - 1) * verts[:, 0]
                
                cam_t = pred_cam_t_full[n]
                
                # Extract 2D keypoints in original image coordinates
                keypoints_2d = out["pred_keypoints_2d"][0].float().cpu().numpy()
                keypoints_2d = keypoints_2d * box_size.cpu().numpy() + box_center.cpu().numpy()
                
                # Extract 3D keypoints in camera space
                keypoints_3d = out["pred_keypoints_3d"].cpu().numpy()[0]
                keypoints_3d = keypoints_3d + pred_cam_t_full
                
                # Extract wrist pose
                wrist_rotation = out["pred_mano_params"]["global_orient"].cpu().numpy()[0, 0, :, :]
                wrist_t = keypoints_3d[0, np.newaxis]
                wrist_pose = np.hstack((wrist_rotation, wrist_t.T))
                wrist_pose = np.vstack((wrist_pose, np.array([0, 0, 0, 1])))
                
                # Create mesh
                camera_translation = cam_t.copy()
                tmesh = self.hamer_renderer.vertices_to_trimesh(
                    verts, camera_translation, LIGHT_BLUE, is_right=is_right
                )
                
                # Adjust mesh coordinates (flip y and z)
                tmesh_adjusted = copy.deepcopy(tmesh)
                tmesh_adjusted.vertices[:, 1:] *= -1
                
                # Store results
                results['vertices'].append(verts)
                results['keypoints_2d'].append(keypoints_2d)
                results['keypoints_3d'].append(keypoints_3d)
                results['wrist_poses'].append(wrist_pose)
                results['meshes'].append(tmesh_adjusted)
                results['camera_translations'].append(camera_translation)
        
        return results

    def _process_stereo_with_optional_cropping(self, frame_idx, img_cv2, rs_ir1, rs_ir2):
        """
        Process stereo depth with optional cropping optimization.
        
        Args:
            frame_idx: Current frame index
            img_cv2: RGB image
            rs_ir1: Left infrared image
            rs_ir2: Right infrared image
            
        Returns:
            Tuple of (fs_depth, pointcloud) or (None, None) if processing fails
        """
        if self.processor is None or rs_ir1 is None or rs_ir2 is None:
            return None, None, None, None, None
            
        # Check if cropped stereo processing is enabled
        use_cropped = self.cfg.get('use_cropped_stereo', False)
        
        if not use_cropped:
            # Original full-image processing
            try:
                # For full processing, we need to get bounding boxes separately
                bb_model = self.cfg.get('bb_model', 'sam')
                mask = None
                
                if bb_model == "vit":
                    boxes, right_flags = self._detect_hands_vit(img_cv2)
                elif bb_model == "sam":
                    boxes, right_flags, mask = self._detect_hands_sam(img_cv2)
                else:
                    boxes, right_flags = [], []
                
                fs_depth, pointcloud = self.processor.process_images(rs_ir1, rs_ir2, img_cv2)
                return fs_depth, boxes, right_flags, mask, None  # No crop_info for full processing
            except Exception as e:
                print(f"Full stereo processing failed for frame {frame_idx}: {e}")
                return None, None, None, None, None
        
        # Cropped stereo processing
        try:
            # First, we need to get a hand bounding box for cropping
            # We'll do a quick hand detection first
            crop_padding = self.cfg.get('crop_padding', 100)
            
            # Detect hands to get bounding box
            mask = None
            bb_model = self.cfg.get('bb_model', 'sam')
            
            if bb_model == "vit":
                boxes, right_flags = self._detect_hands_vit(img_cv2)
            elif bb_model == "sam":
                boxes, right_flags, mask = self._detect_hands_sam(img_cv2)
            else:
                # Fallback to full processing
                try:
                    fs_depth, pointcloud = self.processor.process_images(rs_ir1, rs_ir2, img_cv2)
                    return fs_depth, boxes, right_flags, mask, None  # No crop_info for full processing
                except Exception as e:
                    print(f"Fallback full stereo processing failed: {e}")
                    return None, None, None, None, None
            
            if len(boxes) == 0:
                print(f"Frame {frame_idx}: No hands detected, falling back to full stereo processing")
                try:
                    fs_depth, pointcloud = self.processor.process_images(rs_ir1, rs_ir2, img_cv2)
                    return fs_depth, boxes, right_flags, mask, None  # No crop_info for full processing
                except Exception as e:
                    print(f"Fallback full stereo processing failed: {e}")
                    return None, None, None, None, None
            
            # Use the first detected hand bounding box for cropping
            hand_bbox = boxes[0]  # [x_min, y_min, x_max, y_max]
            
            # Crop images around hand region
            cropped_left, cropped_right, cropped_rgb, crop_info = crop_images_for_stereo(
                rs_ir1, rs_ir2, img_cv2, hand_bbox, padding=crop_padding
            )
            
            if crop_info is None:
                print(f"Frame {frame_idx}: Cropping failed, falling back to full processing")
                try:
                    fs_depth, pointcloud = self.processor.process_images(rs_ir1, rs_ir2, img_cv2)
                    return fs_depth, boxes, right_flags, mask, None  # No crop_info for full processing
                except Exception as e:
                    print(f"Fallback full stereo processing failed: {e}")
                    return None, None, None, None, None
            
            # Adjust camera intrinsics for cropped images
            adj_color_intrinsic, adj_depth_intrinsic = adjust_camera_intrinsics_for_crop(crop_info)
            
            # Process cropped images
            cropped_fs_depth, cropped_pointcloud = self.processor.process_images(
                cropped_left, cropped_right, cropped_rgb, adj_color_intrinsic, adj_depth_intrinsic
            )
            
            return cropped_fs_depth, boxes, right_flags, mask, crop_info
            
        except Exception as e:
            print(f"Frame {frame_idx}: Cropped stereo processing failed: {e}")
            print(f"Frame {frame_idx}: Falling back to full stereo processing")
            try:
                fs_depth, pointcloud = self.processor.process_images(rs_ir1, rs_ir2, img_cv2)
                return fs_depth, boxes, right_flags, mask, None  # No crop_info for full processing
            except Exception as e2:
                print(f"Frame {frame_idx}: Full stereo processing also failed: {e2}")
                return None, None, None, None, None

    def _process_single_frame(self, frame_idx, img_cv2: np.ndarray, boxes, right_flags, mask, 
                             fs_depth: np.ndarray = None, crop_info: dict = None) -> tuple:
        """
        Process a single frame to extract hand pose.
        
        Args:
            img_cv2: Input image in BGR format
            bb_model: Bounding box detection model ("sam" or "vit")
            pointcloud: Optional point cloud data for registration
            fs_depth: Optional stereo depth data
            
        Returns:
            Tuple of (mesh, overlay_image, keypoints_2d, keypoints_3d, wrist_pose, mask)
        """
        # Calculate scaled focal length
        img_size = max(img_cv2.shape[:2])
        scaled_focal_length = COLOR_INTRINSIC[1, 1]  # from realsense camera
        
        # Convert to numpy arrays
        boxes = np.array(boxes)
        # hand_bbox = np.array(hand_bbox).reshape(1, 4)  # Single box
        right_flags = np.array(right_flags)
        
        # Run HaMeR inference
        results = self._run_hamer_inference(img_cv2, boxes, right_flags, scaled_focal_length)
        
        # Extract first detection results
        if len(results['meshes']) > 0:
            mesh = results['meshes'][0]
            keypoints_2d = results['keypoints_2d'][0]
            keypoints_3d = results['keypoints_3d'][0]
            wrist_pose = results['wrist_poses'][0]
            uncorrected_keypoints = copy.deepcopy(keypoints_3d)
            
            # Perform alignment using available data
            # Priority: 1) Direct 3D alignment with point cloud, 2) Depth alignment based on config, 3) Original predictions
            depth_alignment_method = self.cfg.get('depth_alignment_method', 'simplified')
            pointcloud=None
            # Method 1: Direct 3D alignment if point cloud is available (simplest and most robust)
            if pointcloud is not None and mask is not None:
                try:
                    # Extract hand point cloud
                    hand_pcd = self._extract_hand_pointcloud(pointcloud, mask)
                    
                    if hand_pcd is not None and len(hand_pcd) > 50:
                        print(f"Using direct 3D alignment with {len(hand_pcd)} hand points")
                        
                        # Use direct 3D alignment (both keypoints_3d and hand_pcd should be in RGB camera frame)
                        mesh, keypoints_3d, wrist_pose, translation = perform_direct_3d_alignment(
                            mesh, hand_pcd, keypoints_3d, wrist_pose, frame_idx, wrist_keypoint_idx=0
                        )
                        
                        print(f"Applied direct 3D translation: {translation}")
                    else:
                        print("Insufficient hand points for direct 3D alignment")
                        
                except Exception as e:
                    print(f"Direct 3D alignment failed: {e}")
                    
            # Method 2: Depth alignment using fs_depth (if enabled and available)
            elif fs_depth is not None and keypoints_2d is not None and depth_alignment_method != "disabled":
                try:
                    if depth_alignment_method == "simplified":
                        # print(f"Using simplified FoundationStereo depth alignment")
                        
                        # Use the simplified depth-based alignment (no complex coordinate transformations)
                        mesh, keypoints_3d, wrist_pose, depth_translation = perform_simplified_depth_alignment(
                            mesh, keypoints_3d, wrist_pose, fs_depth, keypoints_2d, frame_idx, 
                            crop_info=crop_info, wrist_keypoint_idx=0
                        )
                        print(self.out_folder)
                        
                        # print(f"Applied simplified depth translation: {depth_translation}")
                        
                    elif depth_alignment_method == "complex":
                        print(f"Using complex FoundationStereo depth alignment")
                        
                        # Extract hand point cloud for fallback (if mask is available)
                        hand_pcd = None
                        if pointcloud is not None and mask is not None:
                            hand_pcd = self._extract_hand_pointcloud(pointcloud, mask)

                        # Use the complex FoundationStereo depth-based alignment
                        mesh, keypoints_3d, wrist_pose, depth_translation = perform_depth_based_alignment(
                            mesh, hand_pcd, keypoints_3d, wrist_pose, 
                            wrist_keypoint_idx=0, fs_depth=fs_depth, keypoints_2d=keypoints_2d
                        )
                        
                        print(f"Applied complex depth translation: {depth_translation}")
                    
                except Exception as e:
                    print(f"{depth_alignment_method.capitalize()} depth alignment failed: {e}")
                    
            else:
                if depth_alignment_method == "disabled":
                    print("Depth alignment disabled by configuration - using original HaMeR predictions")
                else:
                    print("No alignment data available - using original HaMeR predictions")
            

            overlay_image = img_cv2.astype(np.float32)[:, :, ::-1] / 255.0
            
            return mesh, overlay_image, keypoints_2d, keypoints_3d, wrist_pose, mask, uncorrected_keypoints
        
        # Return empty results if inference failed
        empty_mesh = trimesh.Trimesh()
        empty_image = np.zeros(img_cv2.shape, dtype=np.float32)
        return empty_mesh, empty_image, None, None, None, mask

    def _extract_hand_pointcloud(self, pointcloud: dict, mask: np.ndarray) -> np.ndarray:
        """
        Extract hand point cloud from full point cloud using mask.
        
        Args:
            pointcloud: Dictionary containing 'points' and 'rgb' arrays
            mask: Binary mask indicating hand regions
            
        Returns:
            Filtered point cloud array or None if extraction fails
        """
        try:
            # Resize mask if necessary to match pointcloud dimensions
            if mask.size != pointcloud['points'].shape[0]:
                # Assuming standard camera resolution ratios
                target_height = pointcloud['points'].shape[0] // 1280
                target_width = pointcloud['points'].shape[0] // 720
                resized_mask = cv2.resize(mask, (target_width, target_height))
                mask_bool = resized_mask.reshape(-1).astype(bool)
            else:
                mask_bool = mask.reshape(-1).astype(bool)
            
            # Extract hand points using mask
            if mask_bool.shape[0] == pointcloud['points'].shape[0]:
                hand_pcd = pointcloud['points'][mask_bool]
                print(f"Extracted {len(hand_pcd)} hand points from point cloud")
                return hand_pcd
            else:
                print(f"Warning: Could not match mask shape {mask_bool.shape} with pointcloud shape {pointcloud['points'].shape}")
                return None
                
        except Exception as e:
            print(f"Error extracting hand point cloud: {e}")
            return None
    
    def run_hamer(self, bb_model: str = "sam"):
        """
        Run HaMeR hand pose estimation on all input files.
        
        Args:
            bb_model: Bounding box detection model ("sam" or "vit")
        """
        print(f"Starting HaMeR processing with {bb_model} detection...")
        
        filepath = self.filepath
        filename = "processed.pkl"
        
        print(f"Processing File: {filename}")
        print(f"BowieSyncData Filepath: {filepath}")
        
        try:
            rs_ir1 = self.left_ir
            rs_ir2 = self.right_ir
            # rs_depth = self.rs_depth
            # Process frames
            hand_objs = []
            img_overlays = []
            keypoints = []
            hand_masks = []
            all_uncorrected_keypoints = []
            
            print(f"Processing {len(self.rs_color)} frames...")
            for frame_idx in tqdm(range(self.start_idx,len(self.rs_color)+self.end_idx)):
                try:
                    img = self.rs_color[frame_idx]
                    
                    # Process stereo depth with optional cropping optimization
                    pointcloud = None
                    fs_depth = None
                    
                    # Initialize stereo processor if not already done
                    if self.processor is None:
                        self._init_stereo_processor()
                    
                    # Process stereo depth (with optional cropping)
                    fs_depth, boxes, right_flags, mask, crop_info = self._process_stereo_with_optional_cropping(
                        frame_idx, img, rs_ir1[frame_idx] if rs_ir1 is not None else None,
                        rs_ir2[frame_idx] if rs_ir2 is not None else None
                    )
                
                    # Process frame
                    mesh, overlay, kp_2d, kp_3d, wrist, mask, uncorrected_keypoints = self._process_single_frame(
                        frame_idx, img, boxes, right_flags, mask, fs_depth=fs_depth, crop_info=crop_info
                    )

                    # Store results
                    hand_objs.append(mesh)
                    img_overlays.append(overlay)
                    hand_masks.append(mask)
                    all_uncorrected_keypoints.append(uncorrected_keypoints)

                    if kp_2d is not None and kp_3d is not None and wrist is not None:
                        keypoints.append((kp_2d, kp_3d, wrist))
                    else:
                        keypoints.append((None, None, None))
                    
                    # Clean up frame-specific variables to reduce memory usage
                    cleanup_variables(fs_depth, pointcloud, crop_info)
                    
                    # Force garbage collection every 20 frames
                    if frame_idx % 20 == 0:
                        cleanup_memory()
                        
                except Exception as e:
                    print(f"Error processing frame {frame_idx}: {e}")
                
                    
                    # Add empty results for failed frame
                    hand_objs.append(trimesh.Trimesh())
                    img_overlays.append(np.zeros_like(self.rs_color[0], dtype=np.float32))
                    hand_masks.append(None)
                    all_uncorrected_keypoints.append(None)
                    keypoints.append((None, None, None))
            
            # Save processed data
            self.save_data(self.rs_color, rs_ir1, rs_ir2,
                            keypoints, all_uncorrected_keypoints, hand_objs, hand_masks, self.bowie_data, filename)
            
            
        except KeyboardInterrupt:
            print("Processing interrupted by user")
        except Exception as e:
            print(f"Error processing file {filepath}: {e}")
       
    
    def save_data(self, rs_color: np.ndarray,
                  rs_ir1: np.ndarray, rs_ir2: np.ndarray, keypoints: list, uncorrected_keypoints: list,
                  hand_objs: list, hand_masks: list, bowie, filename: str):
        """
        Save processed data to pickle file.
        
        Args:
            rs_time: RealSense timestamps
            rs_color: Color images
            rs_depth: Depth images
            rs_ir1: Left infrared images
            rs_ir2: Right infrared images
            keypoints: List of (2D_keypoints, 3D_keypoints, wrist_pose) tuples
            uncorrected_keypoints: List of 3D keypoints without depth correction in z-axis
            hand_objs: List of hand mesh objects
            hand_masks: List of hand masks
            bowie: Bowie glove data object
            filename: Output filename
        """
        print(f"Saving processed data to {filename}...")
        
        data_dict = {
            "rs_color": self.rs_color,
            "fs_depth": None,  # Placeholder for FoundationStereo depth
            "crop_info": None,  # Placeholder for cropping info
            "pred_keypoints_2d": np.array([item[0] for item in keypoints]),
            "pred_keypoints_3d": np.array([item[1] for item in keypoints]),
            "uncorrected_keypoints": uncorrected_keypoints,
            "wrist": np.array([item[2] for item in keypoints]),
            "glove": self.bowie_data,
        }
        
        # Save to file (overwrites existing data)
        savepath = os.path.join(self.out_folder, filename)
        with open(savepath, "wb") as f:
            pickle.dump(data_dict, f)
        
        print(f"Data saved successfully to {savepath}")
        
        # Print convenience stats
        no_hand_frames = sum(1 for mesh in hand_objs if mesh.is_empty)
        hand_frames = len(self.rs_color) - no_hand_frames
        print(f"Statistics: {len(self.rs_color)} total frames, {hand_frames} with hands, {no_hand_frames} without hands")

        import matplotlib.pyplot as plt
        #corrected vs uncorrected wrist keypoints
        uk = np.asarray(uncorrected_keypoints)
        ck = np.array([item[1] for item in keypoints])
        plt.figure(figsize=(10, 5))
        plt.plot(uk[:, 0, 2], label='Uncorrected Wrist Z', color='r')
        plt.plot(ck[:, 0, 2], label='Corrected Wrist Z', color='g')
        plt.xlabel('Frame Index')
        plt.ylabel('Z Coordinate (m)')
        plt.title('Wrist Keypoint Z Coordinate: Corrected vs Uncorrected')
        plt.legend()
        plt.grid()
        plt.savefig(os.path.join(self.out_folder, "wrist_z_comparison.png"))
        plt.close()
    
    def cleanup(self):
        """Clean up GPU memory and model references."""
        print("Cleaning up GlovePoseTracker...")
        
        # Clear data arrays
        cleanup_variables(getattr(self, 'rs_color', None), 
                         getattr(self, 'left_ir', None), 
                         getattr(self, 'right_ir', None), 
                         getattr(self, 'bowie_data', None))
        
        # Clear model references
        for attr in ['hamer_model', 'hamer_detector', 'cpm', 'hamer_renderer', 'langsam', 'processor']:
            if hasattr(self, attr) and getattr(self, attr) is not None:
                delattr(self, attr)
        
        # Clear CUDA cache
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
            torch.cuda.synchronize()
        
        gc.collect()
        print("Cleanup complete - Final memory state:")
        check_cuda_memory()
    


# ============================================================================
# MAIN EXECUTION
# ============================================================================

def main() -> None:
    """
    Main function for running the HaMeR hand pose estimation pipeline.

    """
    print("=" * 60)
    print("HaMeR Hand Pose Estimation Pipeline")
    print("=" * 60)

    parser = argparse.ArgumentParser(
                    prog='',
                    description='',
                    epilog='')

    parser.add_argument('data_paths', type=str)
    args = parser.parse_args()

    with hydra.initialize(version_base=None, config_path="../config"):
        cfg = hydra.compose(config_name="config_extract_hamer.yaml")
    cfg['paths']['data_path'] = args.data_paths.split(',')
    # Initialize camera calibration constants from configuration
    initialize_camera_constants(cfg)
    # Extract configuration paths
    print(cfg.paths)
    all_raw_data_paths = cfg.paths.raw_data_path
    hamer_ckpt = cfg.paths.hamer_ckpt_path
    
    print(f"Processing {len(all_raw_data_paths)} data directories...")
    print(f"Using detection model: {cfg.bb_model}")
    
    # Process each data directory
    for i, raw_data_path in enumerate(all_raw_data_paths):
        print(f"\n[{i+1}/{len(all_raw_data_paths)}] Processing: {raw_data_path}")
        try:
            out_folder = raw_data_path
            
            # Initialize and run tracker
            print("raw data path", raw_data_path)
            print("out folder", out_folder)
            glove_tracker = GlovePoseTracker(
                raw_data_path, out_folder=out_folder, hamer_ckpt=hamer_ckpt, cfg=cfg
            )
            
            glove_tracker.run_hamer(bb_model=cfg.bb_model)
            
            # Cleanup
            print(f"Cleaning up memory after processing {raw_data_path}")
            glove_tracker.cleanup()
            del glove_tracker
            
            # Force garbage collection
            gc.collect()
            
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
                torch.cuda.synchronize()
                print(f"GPU memory allocated: {torch.cuda.memory_allocated() / 1024**3:.2f} GB")
            
            print("-" * 50)
            
        except Exception as e:
            print(f"Error processing {raw_data_path}: {e}")        
            
            # Cleanup on error
            if 'glove_tracker' in locals():
                try:
                    glove_tracker.cleanup()
                    del glove_tracker
                except:
                    pass
            
            # Emergency memory cleanup
            cleanup_memory()
            
            continue
    
    print("\nProcessing complete!")


if __name__ == "__main__":
    main()