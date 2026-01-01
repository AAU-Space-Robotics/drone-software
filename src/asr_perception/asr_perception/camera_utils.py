#!/usr/bin/env python3
"""
Camera Utilities Module
Geometric operations for camera projection, depth conversion, and coordinate transforms.
Pure geometric logic - no ROS dependencies.
"""

import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass


@dataclass
class CameraIntrinsics:
    """Camera intrinsic parameters."""
    fx: float  # Focal length x
    fy: float  # Focal length y
    cx: float  # Principal point x
    cy: float  # Principal point y
    width: int  # Image width
    height: int  # Image height
    distortion: Optional[np.ndarray] = None  # Distortion coefficients
    
    def to_matrix(self) -> np.ndarray:
        """Get 3x3 camera matrix."""
        return np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])


@dataclass
class Transform:
    """Rigid body transform (rotation + translation)."""
    rotation: np.ndarray  # 3x3 rotation matrix
    translation: np.ndarray  # 3x1 translation vector
    
    def to_homogeneous(self) -> np.ndarray:
        """Convert to 4x4 homogeneous transformation matrix."""
        T = np.eye(4)
        T[:3, :3] = self.rotation
        T[:3, 3] = self.translation
        return T
    
    def inverse(self) -> 'Transform':
        """Get inverse transform."""
        R_inv = self.rotation.T
        t_inv = -R_inv @ self.translation
        return Transform(R_inv, t_inv)


class CameraProjection:
    """
    Handles camera projection operations:
    - Pixel + depth → 3D point in camera frame
    - Camera frame → drone/body frame
    - Drone frame → global/world frame
    """
    
    def __init__(self, intrinsics: CameraIntrinsics, 
                 camera_to_body_transform: Optional[Transform] = None):
        """
        Initialize camera projection.
        
        Args:
            intrinsics: Camera intrinsic parameters
            camera_to_body_transform: Transform from camera to drone body frame
        """
        self.intrinsics = intrinsics
        self.K = intrinsics.to_matrix()
        self.K_inv = np.linalg.inv(self.K)
        
        # Default: camera frame = body frame (identity transform)
        if camera_to_body_transform is None:
            camera_to_body_transform = Transform(np.eye(3), np.zeros(3))
        
        self.camera_to_body = camera_to_body_transform
        self.T_cam_to_body = camera_to_body_transform.to_homogeneous()
    
    def pixel_to_camera_frame(self, u: float, v: float, depth: float) -> np.ndarray:
        """
        Convert pixel coordinates + depth to 3D point in camera frame.
        
        Args:
            u: Pixel x coordinate
            v: Pixel y coordinate
            depth: Depth value (meters)
            
        Returns:
            3D point in camera frame [x, y, z]
        """
        if depth <= 0:
            return None
        
        # Back-project to camera frame
        pixel_hom = np.array([u, v, 1.0])
        point_cam = self.K_inv @ pixel_hom * depth
        
        return point_cam
    
    def depth_to_pointcloud(self, u: float, v: float, depth: float) -> Optional[np.ndarray]:
        """
        Convert pixel + depth to 3D point in camera frame.
        Alias for pixel_to_camera_frame for compatibility.
        
        Returns:
            3D point [x, y, z] or None if invalid depth
        """
        return self.pixel_to_camera_frame(u, v, depth)
    
    def camera_to_body_frame(self, point_camera: np.ndarray) -> np.ndarray:
        """
        Transform point from camera frame to drone body frame.
        
        Args:
            point_camera: Point in camera frame [x, y, z]
            
        Returns:
            Point in body frame [x, y, z]
        """
        point_hom = np.append(point_camera, 1.0)
        point_body_hom = self.T_cam_to_body @ point_hom
        return point_body_hom[:3]
    
    def body_to_global_frame(self, point_body: np.ndarray, 
                            position: np.ndarray, orientation: np.ndarray) -> np.ndarray:
        """
        Transform point from drone body frame to global frame.
        
        Args:
            point_body: Point in body frame [x, y, z]
            position: Drone position in global frame [x, y, z]
            orientation: Drone orientation as quaternion [x, y, z, w] or rotation matrix
            
        Returns:
            Point in global frame [x, y, z]
        """
        # Convert orientation to rotation matrix if quaternion
        if orientation.shape == (4,):
            R_body_to_global = self.quaternion_to_matrix(orientation)
        else:
            R_body_to_global = orientation
        
        # Transform: p_global = R * p_body + t
        point_global = R_body_to_global @ point_body + position
        
        return point_global
    
    def pixel_to_global(self, u: float, v: float, depth: float,
                       drone_position: np.ndarray, drone_orientation: np.ndarray) -> Optional[np.ndarray]:
        """
        Complete pipeline: pixel + depth → global 3D position.
        
        Args:
            u, v: Pixel coordinates
            depth: Depth value (meters)
            drone_position: Drone position in global frame [x, y, z]
            drone_orientation: Drone orientation (quaternion or rotation matrix)
            
        Returns:
            3D point in global frame [x, y, z] or None if invalid
        """
        # Step 1: Pixel + depth → camera frame
        point_cam = self.pixel_to_camera_frame(u, v, depth)
        if point_cam is None:
            return None
        
        # Step 2: Camera frame → body frame
        point_body = self.camera_to_body_frame(point_cam)
        
        # Step 3: Body frame → global frame
        point_global = self.body_to_global_frame(point_body, drone_position, drone_orientation)
        
        return point_global
    
    @staticmethod
    def quaternion_to_matrix(q: np.ndarray) -> np.ndarray:
        """
        Convert quaternion to rotation matrix.
        
        Args:
            q: Quaternion [x, y, z, w]
            
        Returns:
            3x3 rotation matrix
        """
        x, y, z, w = q
        
        R = np.array([
            [1 - 2*y*y - 2*z*z,  2*x*y - 2*z*w,      2*x*z + 2*y*w],
            [2*x*y + 2*z*w,      1 - 2*x*x - 2*z*z,  2*y*z - 2*x*w],
            [2*x*z - 2*y*w,      2*y*z + 2*x*w,      1 - 2*x*x - 2*y*y]
        ])
        
        return R
    
    @staticmethod
    def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
        """
        Convert rotation matrix to quaternion.
        
        Args:
            R: 3x3 rotation matrix
            
        Returns:
            Quaternion [x, y, z, w]
        """
        trace = np.trace(R)
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        return np.array([x, y, z, w])
    
    @staticmethod
    def create_from_params(fx: float, fy: float, cx: float, cy: float,
                          width: int, height: int,
                          camera_to_body_matrix: Optional[np.ndarray] = None) -> 'CameraProjection':
        """
        Factory method to create CameraProjection from parameters.
        
        Args:
            fx, fy: Focal lengths
            cx, cy: Principal point
            width, height: Image dimensions
            camera_to_body_matrix: 4x4 or 3x4 transform matrix (optional)
            
        Returns:
            CameraProjection instance
        """
        intrinsics = CameraIntrinsics(fx, fy, cx, cy, width, height)
        
        transform = None
        if camera_to_body_matrix is not None:
            if camera_to_body_matrix.shape == (4, 4):
                R = camera_to_body_matrix[:3, :3]
                t = camera_to_body_matrix[:3, 3]
            elif camera_to_body_matrix.shape == (3, 4):
                R = camera_to_body_matrix[:3, :3]
                t = camera_to_body_matrix[:, 3]
            else:
                raise ValueError(f"Invalid transform matrix shape: {camera_to_body_matrix.shape}")
            
            transform = Transform(R, t)
        
        return CameraProjection(intrinsics, transform)
