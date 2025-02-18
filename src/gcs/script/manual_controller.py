#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.msg import ManualControlInput
import pygame

class ManualController(Node):
    def __init__(self):
        super().__init__('manual_controller')
        # ROS2 QoS settings
        qos_settings = rclpy.qos.QoSProfile(depth=10)
        qos_settings.reliability = rclpy.qos.ReliabilityPolicy.BEST_EFFORT
        qos_settings.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        


        self.publisher_ = self.create_publisher(ManualControlInput, 'drone/in/manual_input', qos_settings)
        self.timer = self.create_timer(0.01, self.publish_control)  # 100Hz
        self.drone_cmd = ManualControlInput()


       
        # Initialize pygame for joystick input
        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.get_logger().info(f'Connected to joystick: {self.joystick.get_name()}')
        else:
            self.joystick = None
            self.get_logger().warn('No joystick detected!')
    
    def read_controller_input(self):
        """Reads input from the PS4 controller and updates the drone_cmd."""
        if self.joystick is None:
            return
        
        pygame.event.pump()  # Process pygame events
        
        # Read joystick axes (values range from -1 to 1)
        self.drone_cmd.roll = -self.joystick.get_axis(0) if abs(self.joystick.get_axis(0)) > 0.05 else 0.0  # Left stick X-axis
        self.drone_cmd.pitch = -self.joystick.get_axis(1) if abs(self.joystick.get_axis(1)) > 0.05 else 0.0  # Left stick Y-axis (inverted for correct mapping)
        self.drone_cmd.yaw_velocity = -self.joystick.get_axis(3) if abs(self.joystick.get_axis(3)) > 0.05 else 0.0  # Right stick X-axis
        self.drone_cmd.thrust = self.joystick.get_axis(4) if abs(self.joystick.get_axis(4)) > 0.05 else 0.0  # Right stick Y-axis, mapped from [-1,1] to [0,-1]
        
    def publish_control(self):
        self.read_controller_input()
        self.publisher_.publish(self.drone_cmd)
        self.get_logger().info(
            f'Publishing: Roll={self.drone_cmd.roll:.2f}, Pitch={self.drone_cmd.pitch:.2f}, '
            f'Yaw={self.drone_cmd.yaw_velocity:.2f}, Thrust={self.drone_cmd.thrust:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = ManualController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == '__main__':
    main()
