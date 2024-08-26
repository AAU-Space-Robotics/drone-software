#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import cv2
import numpy as np

class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_publisher')

        # Create a publisher for the image topic
        self.publisher_ = self.create_publisher(Image, 'realsense/image', 10)

        # Initialize the RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(config)

        # Set the loop rate to 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Initialize the CvBridge
        self.bridge = CvBridge()

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            return

        # Convert the RealSense frame to a NumPy array
        color_image = np.asanyarray(color_frame.get_data())

        # Convert the NumPy array to a ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')

        # Publish the image
        self.publisher_.publish(ros_image)

    def destroy(self):
        # Stop the pipeline
        self.pipeline.stop()

        super().destroy()

def main(args=None):
    rclpy.init(args=args)

    realsense_publisher = RealSensePublisher()

    try:
        rclpy.spin(realsense_publisher)
    except KeyboardInterrupt:
        pass

    realsense_publisher.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
