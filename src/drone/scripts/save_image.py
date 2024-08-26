#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')

        # Create a subscriber for the image topic
        self.subscription = self.create_subscription(
            Image,
            '/realsense/image',  # Replace with your image topic
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Set up directory for saving images
        self.image_dir = 'saved_images'
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)

        self.image_count = 0  # Counter for naming saved images

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Save the image to disk
        image_filename = os.path.join(self.image_dir, f'image_{self.image_count:04d}.png')
        cv2.imwrite(image_filename, cv_image)
        self.get_logger().info(f'Saved {image_filename}')

        self.image_count += 1  # Increment the image counter

def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()

    try:
        rclpy.spin(image_saver)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown ROS 2
        image_saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
