#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from drone_interfaces.srv import Clock

class Clock_server(Node):
    """Clock server node"""
    def __init__(self):
        # Initialize the node
        super().__init__('clock_sync_server')
        self.srv = self.create_service(Clock, '/sync_clock', self.get_server_time)

    def get_server_time(self, request, response):
        self.get_logger().info(f'Request received: {request.request_value}')
        response.server_time = time.time()
        self.get_logger().info('Sending response')
        # Return the response
        return response

def main(args=None):
    rclpy.init(args=args)
    clock = Clock_server()
    clock.get_logger().info('Clock server is running')
    rclpy.spin(clock)   

    rclpy.shutdown()

if __name__ == '__main__':
    main()