#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.msg import DroneState  # Change to your actual package name
import time

class DummyDronePublisher(Node):
    def __init__(self):
        super().__init__('dummy_drone_publisher')

        self.publisher_ = self.create_publisher(DroneState, '/drone/out/state', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.counter = 0.0

        self.get_logger().info('Dummy DroneState Publisher Started')

    def timer_callback(self):
        msg = DroneState()
        self.counter += 0.1

        # Rising values
        msg.position = [self.counter, self.counter + 1.0, self.counter + 2.0]
        msg.velocity = [0.5 * self.counter, 0.3 * self.counter, 0.1 * self.counter]
        msg.orientation = [0.1 * self.counter, 0.2 * self.counter, 0.3 * self.counter]
        msg.battery_voltage = 12.0 - 0.01 * self.counter  # slowly dropping battery

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyDronePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
