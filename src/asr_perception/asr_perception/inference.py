#!/usr/bin/env python3
"""
YOLO Inference Module
Handles object detection inference with YOLO models (PyTorch/TensorRT).
Pure inference logic - no ROS dependencies.
"""

from ultralytics import YOLO
import numpy as np
from typing import List, Tuple, Optional
import os


class Detection:
    """Represents a single object detection."""
    def __init__(self, bbox: Tuple[float, float, float, float], 
                 confidence: float, class_id: int, class_name: str,
                 mask: Optional[np.ndarray] = None):
        self.x, self.y, self.w, self.h = bbox  # Bounding box (center_x, center_y, width, height)
        self.confidence = confidence
        self.class_id = class_id
        self.class_name = class_name
        self.mask = mask  # Segmentation mask if available
        
    def get_center(self) -> Tuple[float, float]:
        """Get center point of bounding box."""
        return (self.x, self.y)
    
    def get_corners(self) -> Tuple[float, float, float, float]:
        """Get bounding box corners (x1, y1, x2, y2)."""
        x1 = self.x - self.w / 2
        y1 = self.y - self.h / 2
        x2 = self.x + self.w / 2
        y2 = self.y + self.h / 2
        return (x1, y1, x2, y2)


class YOLOInference:
    """
    YOLO inference wrapper supporting both PyTorch and TensorRT models.
    """
    
    def __init__(self, model_path: str, confidence_threshold: float = 0.5,
                 device: str = 'cuda:0', half_precision: bool = False):
        """
        Initialize YOLO model for inference.
        
        Args:
            model_path: Path to YOLO model (.pt, .engine, .onnx)
            confidence_threshold: Minimum confidence for detections
            device: Device to run inference on ('cuda:0', 'cpu')
            half_precision: Use FP16 precision (for TensorRT)
        """
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model not found: {model_path}")
        
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.device = device
        self.half_precision = half_precision
        
        # Load YOLO model
        self.model = YOLO(model_path)
        
        # Determine model type
        self.is_tensorrt = model_path.endswith('.engine')
        self.is_segmentation = 'seg' in model_path.lower() or self._check_segmentation()
        
        print(f"Loaded YOLO model: {model_path}")
        print(f"  - Type: {'TensorRT' if self.is_tensorrt else 'PyTorch'}")
        print(f"  - Segmentation: {self.is_segmentation}")
        print(f"  - Device: {device}")
    
    def _check_segmentation(self) -> bool:
        """Check if model supports segmentation."""
        try:
            # Try a dummy prediction to check model capabilities
            dummy_img = np.zeros((640, 640, 3), dtype=np.uint8)
            results = self.model.predict(dummy_img, verbose=False)
            return hasattr(results[0], 'masks') and results[0].masks is not None
        except:
            return False
    
    def detect(self, image: np.ndarray, verbose: bool = False) -> List[Detection]:
        """
        Run inference on an image.
        
        Args:
            image: Input image (numpy array, HxWx3, BGR format)
            verbose: Print detection info
            
        Returns:
            List of Detection objects
        """
        # Run inference
        results = self.model.predict(
            image,
            conf=self.confidence_threshold,
            device=self.device,
            half=self.half_precision,
            verbose=verbose
        )
        
        # Parse results
        detections = self._parse_results(results[0])
        
        if verbose:
            print(f"Detected {len(detections)} objects")
        
        return detections
    
    def _parse_results(self, result) -> List[Detection]:
        """Parse YOLO results into Detection objects."""
        detections = []
        
        if result.boxes is None or len(result.boxes) == 0:
            return detections
        
        boxes = result.boxes.xywh.cpu().numpy()  # Center format
        confidences = result.boxes.conf.cpu().numpy()
        class_ids = result.boxes.cls.cpu().numpy().astype(int)
        
        # Get masks if available
        masks = None
        if self.is_segmentation and result.masks is not None:
            masks = result.masks.data.cpu().numpy()
        
        # Create Detection objects
        for i in range(len(boxes)):
            class_name = result.names[class_ids[i]]
            mask = masks[i] if masks is not None else None
            
            detection = Detection(
                bbox=tuple(boxes[i]),
                confidence=float(confidences[i]),
                class_id=int(class_ids[i]),
                class_name=class_name,
                mask=mask
            )
            detections.append(detection)
        
        return detections
    
    def visualize(self, image: np.ndarray, detections: List[Detection]) -> np.ndarray:
        """
        Draw detections on image.
        
        Args:
            image: Input image
            detections: List of detections to draw
            
        Returns:
            Image with visualizations
        """
        import cv2
        
        vis_image = image.copy()
        
        for det in detections:
            x1, y1, x2, y2 = det.get_corners()
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            # Draw bounding box
            cv2.rectangle(vis_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw label
            label = f"{det.class_name}: {det.confidence:.2f}"
            cv2.putText(vis_image, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Draw mask if available
            if det.mask is not None:
                mask_resized = cv2.resize(det.mask, (image.shape[1], image.shape[0]))
                mask_binary = (mask_resized > 0.5).astype(np.uint8)
                colored_mask = np.zeros_like(image)
                colored_mask[mask_binary == 1] = [0, 255, 0]
                vis_image = cv2.addWeighted(vis_image, 1.0, colored_mask, 0.3, 0)
        
        return vis_image
