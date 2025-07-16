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
        
        self.publisher_ = self.create_publisher(ManualControlInput, 'thyra/in/manual_input', qos_settings)
        self.timer = self.create_timer(0.01, self.publish_control)  # 100Hz
        self.drone_cmd = ManualControlInput()

        # Initialize pygame for keyboard input
        pygame.init()
        self.screen = pygame.display.set_mode((300, 200))  # Small window to capture events
        pygame.display.set_caption("Drone Controller")

    def read_keyboard_input(self):
        """Reads input from the arrow keys and updates the drone_cmd."""
        keys = pygame.key.get_pressed()
        
        # Arrow keys for pitch and roll
        self.drone_cmd.pitch = -1.0 if keys[pygame.K_UP] else (1.0 if keys[pygame.K_DOWN] else 0.0)
        self.drone_cmd.roll = -1.0 if keys[pygame.K_LEFT] else (1.0 if keys[pygame.K_RIGHT] else 0.0)
        
        # Additional keys for yaw and thrust
        self.drone_cmd.yaw_velocity = -1.0 if keys[pygame.K_a] else (1.0 if keys[pygame.K_d] else 0.0)
        self.drone_cmd.thrust = 1.0 if keys[pygame.K_w] else (-1.0 if keys[pygame.K_s] else 0.0)

    def publish_control(self):
        pygame.event.pump()  # Process pygame events
        self.read_keyboard_input()
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
