#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.msg import MotionCapturePose
import time
import socket

class MotionCaptureNode(Node):
    def __init__(self):
        super().__init__('motion_capture_node')
        
        # Set quality of service
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # Create publisher with MotionCapturePose message type
        self.publisher_ = self.create_publisher(MotionCapturePose, 'thyra/in/motion_capture_pose', qos)
        # 110 Hz publishing rate
        self.timer = self.create_timer(1/110, self.publish_message)
        
        # Socket configuration
        self.HOST = '192.168.10.86'  # Motion capture system IP
        self.PORT = 12345           # Port number
        
        # Initialize socket connection
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.bind((self.HOST, self.PORT))
            self.server_socket.listen(1)
            self.get_logger().info(f"Server listening on {self.HOST}:{self.PORT}")
            
            self.conn, self.addr = self.server_socket.accept()
            self.get_logger().info(f"Connection established from {self.addr}")
            
        except Exception as e:
            self.get_logger().error(f"Socket initialization failed: {str(e)}")
            raise

    
    def publish_message(self):
        msg = MotionCapturePose()
        skip_next = False  # Toggle to skip every other message
        
        try:
            # Buffer to store incoming data
            buffer = ""
            while True:
                # Receive data from motion capture system
                data = self.conn.recv(1024).decode()
                if not data:
                    self.get_logger().warn("No data received from socket")
                    return
                
                # Append received data to buffer
                buffer += data
                
                # Check if we have a complete message (ends with newline)
                if '\n' in buffer:
                    # Split buffer by newline to get complete messages
                    messages = buffer.split('\n')
                    # The last element might be incomplete; save it for the next iteration
                    buffer = messages[-1]
                    
                    # Process all complete messages
                    for message in messages[:-1]:
                        if not message:
                            continue
                        received_data = message.split(',')
                        
                        if len(received_data) == 6:
                            if skip_next:
                                skip_next = False
                                continue  # Skip this message
                            
                            x, y, z, roll, pitch, yaw = map(float, received_data)
                            
                            # Populate message fields
                            msg.timestamp = time.time()
                            msg.y = x / 1000.0
                            msg.x = y / 1000.0
                            msg.z = -(z / 1000.0)
                            msg.roll = roll
                            msg.pitch = pitch
                            msg.yaw = yaw
                            
                            # Publish the message
                            self.publisher_.publish(msg)
                            self.get_logger().info(f"Published: x={x:.3f}, y={y:.3f}, z={z:.3f}, "
                                                f"roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}")
                            
                            skip_next = True  # Skip the next valid message
                        else:
                            self.get_logger().warning(f"Received unexpected data format: {received_data}")
                    break  # Exit after processing complete messages
                    
        except Exception as e:
            self.get_logger().error(f"Error in publish_message: {str(e)}")
            
    def destroy_node(self):
        """Clean up resources when shutting down"""
        if hasattr(self, 'conn'):
            self.conn.close()
        if hasattr(self, 'server_socket'):
            self.server_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        motion_capture_node = MotionCaptureNode()
        rclpy.spin(motion_capture_node)
    except Exception as e:
        print(f"Error in main: {str(e)}")
    finally:
        motion_capture_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()