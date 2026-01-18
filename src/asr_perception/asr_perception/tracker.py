#!/usr/bin/env python3
"""
Object Tracking Module
Multi-object tracking with Kalman filtering and data association.
Pure tracking logic - no ROS dependencies.
"""

import numpy as np
from filterpy.kalman import KalmanFilter
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist
from typing import List, Tuple, Optional
from dataclasses import dataclass
import time


@dataclass
class TrackedObject:
    """Represents a tracked object with state estimation."""
    track_id: int
    position: np.ndarray  # [x, y, z]
    velocity: np.ndarray  # [vx, vy, vz]
    confidence: float
    class_name: str
    age: int  # Number of frames since creation
    hits: int  # Number of successful updates
    time_since_update: int  # Frames since last detection match
    covariance: np.ndarray  # Position covariance
    
    def get_position(self) -> Tuple[float, float, float]:
        """Get 3D position as tuple."""
        return tuple(self.position)
    
    def is_confirmed(self, min_hits: int = 3) -> bool:
        """Check if track is confirmed (enough updates)."""
        return self.hits >= min_hits


class Track:
    """Internal track representation with Kalman filter."""
    
    def __init__(self, track_id: int, initial_pos: np.ndarray, 
                 confidence: float, class_name: str):
        """
        Initialize a new track.
        
        Args:
            track_id: Unique track identifier
            initial_pos: Initial 3D position [x, y, z]
            confidence: Initial detection confidence
            class_name: Object class name
        """
        self.track_id = track_id
        self.class_name = class_name
        self.age = 0
        self.hits = 1
        self.time_since_update = 0
        
        # Initialize Kalman filter
        # State: [x, y, z, vx, vy, vz]
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        
        # State vector
        self.kf.x = np.array([
            initial_pos[0], initial_pos[1], initial_pos[2],  # position
            0.0, 0.0, 0.0  # velocity (initially zero)
        ])
        
        # State transition matrix (constant velocity model)
        dt = 1.0  # Will be updated in predict()
        self.kf.F = np.array([
            [1, 0, 0, dt,  0,  0],
            [0, 1, 0,  0, dt,  0],
            [0, 0, 1,  0,  0, dt],
            [0, 0, 0,  1,  0,  0],
            [0, 0, 0,  0,  1,  0],
            [0, 0, 0,  0,  0,  1]
        ])
        
        # Measurement matrix (observe position only)
        self.kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])
        
        # Measurement noise (tune based on sensor accuracy)
        self.kf.R = np.eye(3) * 0.1  # Position measurement noise
        
        # Process noise (tune based on expected dynamics)
        self.kf.Q = np.eye(6) * 0.01  # Process noise
        
        # Initial covariance (high uncertainty initially)
        self.kf.P = np.eye(6) * 1000.0
        
        # Confidence tracking
        self.confidence_sum = confidence
        self.confidence_count = 1
    
    def predict(self, dt: float):
        """
        Predict next state using Kalman filter.
        
        Args:
            dt: Time delta since last update (seconds)
        """
        # Update state transition matrix with current dt
        self.kf.F[0, 3] = dt
        self.kf.F[1, 4] = dt
        self.kf.F[2, 5] = dt
        
        # Predict
        self.kf.predict()
        self.age += 1
        self.time_since_update += 1
    
    def update(self, measurement: np.ndarray, confidence: float):
        """
        Update track with new measurement.
        
        Args:
            measurement: Measured position [x, y, z]
            confidence: Detection confidence
        """
        self.kf.update(measurement)
        self.hits += 1
        self.time_since_update = 0
        
        # Update confidence
        self.confidence_sum += confidence
        self.confidence_count += 1
    
    def get_state(self) -> TrackedObject:
        """Get current track state as TrackedObject."""
        pos = self.kf.x[:3]
        vel = self.kf.x[3:]
        avg_confidence = self.confidence_sum / self.confidence_count
        covariance = self.kf.P[:3, :3]  # Position covariance only
        
        return TrackedObject(
            track_id=self.track_id,
            position=pos.copy(),
            velocity=vel.copy(),
            confidence=avg_confidence,
            class_name=self.class_name,
            age=self.age,
            hits=self.hits,
            time_since_update=self.time_since_update,
            covariance=covariance.copy()
        )


class ObjectTracker:
    """
    Multi-object tracker using Kalman filtering and Hungarian algorithm for data association.
    """
    
    def __init__(self, max_age: int = 10, min_hits: int = 3, 
                 distance_threshold: float = 0.5):
        """
        Initialize object tracker.
        
        Args:
            max_age: Maximum frames to keep track without updates before deletion
            min_hits: Minimum hits before track is confirmed
            distance_threshold: Maximum distance for association (meters)
        """
        self.max_age = max_age
        self.min_hits = min_hits
        self.distance_threshold = distance_threshold
        
        self.tracks: List[Track] = []
        self.next_id = 1
        self.last_time = None
    
    def update(self, detections: List[Tuple[np.ndarray, float, str]], 
               current_time: Optional[float] = None) -> List[TrackedObject]:
        """
        Update tracker with new detections.
        
        Args:
            detections: List of (position, confidence, class_name) tuples
            current_time: Current timestamp (seconds). If None, uses system time.
            
        Returns:
            List of currently tracked objects (only confirmed tracks)
        """
        # Calculate dt
        if current_time is None:
            current_time = time.time()
        
        dt = 0.2  # Default 5 Hz
        if self.last_time is not None:
            dt = max(current_time - self.last_time, 0.01)  # Avoid zero dt
        self.last_time = current_time
        
        # Predict all tracks
        for track in self.tracks:
            track.predict(dt)
        
        # Data association
        matched, unmatched_detections, unmatched_tracks = self._associate(detections)
        
        # Update matched tracks
        for det_idx, track_idx in matched:
            position, confidence, class_name = detections[det_idx]
            self.tracks[track_idx].update(position, confidence)
        
        # Create new tracks for unmatched detections
        for det_idx in unmatched_detections:
            position, confidence, class_name = detections[det_idx]
            new_track = Track(self.next_id, position, confidence, class_name)
            self.tracks.append(new_track)
            self.next_id += 1
        
        # Remove old tracks
        self.tracks = [t for t in self.tracks if t.time_since_update < self.max_age]
        
        # Return confirmed tracks only
        return self.get_confirmed_tracks()
    
    def _associate(self, detections: List[Tuple[np.ndarray, float, str]]) \
            -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
        """
        Associate detections with tracks using Hungarian algorithm.
        
        Returns:
            matched: List of (detection_idx, track_idx) pairs
            unmatched_detections: List of detection indices
            unmatched_tracks: List of track indices
        """
        if len(self.tracks) == 0:
            return [], list(range(len(detections))), []
        
        if len(detections) == 0:
            return [], [], list(range(len(self.tracks)))
        
        # Build cost matrix (Euclidean distance)
        det_positions = np.array([d[0] for d in detections])
        track_positions = np.array([t.kf.x[:3] for t in self.tracks])
        
        cost_matrix = cdist(det_positions, track_positions, metric='euclidean')
        
        # Apply distance threshold
        cost_matrix[cost_matrix > self.distance_threshold] = 1e9
        
        # Hungarian algorithm
        det_indices, track_indices = linear_sum_assignment(cost_matrix)
        
        # Filter out invalid matches (exceed threshold)
        matched = []
        for det_idx, track_idx in zip(det_indices, track_indices):
            if cost_matrix[det_idx, track_idx] < 1e9:
                matched.append((det_idx, track_idx))
        
        # Find unmatched
        matched_det_indices = set(m[0] for m in matched)
        matched_track_indices = set(m[1] for m in matched)
        
        unmatched_detections = [i for i in range(len(detections)) 
                                if i not in matched_det_indices]
        unmatched_tracks = [i for i in range(len(self.tracks)) 
                           if i not in matched_track_indices]
        
        return matched, unmatched_detections, unmatched_tracks
    
    def get_all_tracks(self) -> List[TrackedObject]:
        """Get all tracks (including unconfirmed)."""
        return [track.get_state() for track in self.tracks]
    
    def get_confirmed_tracks(self) -> List[TrackedObject]:
        """Get only confirmed tracks."""
        return [track.get_state() for track in self.tracks 
                if track.hits >= self.min_hits]
    
    def reset(self):
        """Reset tracker (clear all tracks)."""
        self.tracks = []
        self.next_id = 1
        self.last_time = None
