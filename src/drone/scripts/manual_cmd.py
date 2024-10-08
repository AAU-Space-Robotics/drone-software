#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import pygame
import numpy as np
from drone_interfaces.msg import DroneCommand
import time

# Mode dictionary
mode_dict = {0: 'Manual_Control', 1: 'Autonomous', 2: 'Reboot'}

class JoystickControlNode(Node):
    def __init__(self):
        time.time() 
        super().__init__('joystick_control')
        self.publisher_ = self.create_publisher(DroneCommand, '/cmd_fc', 10)
        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        
    def send_control_command(self, DroneCommand):
        self.publisher_.publish(DroneCommand)
        self.get_logger().info(f"Armed={DroneCommand.cmd_arm}, Eland={DroneCommand.cmd_eland}, Estop={DroneCommand.cmd_estop}, mode={mode_dict[DroneCommand.cmd_mode]}, Timestamp={DroneCommand.timestamp},Roll={DroneCommand.cmd_roll}, Pitch={DroneCommand.cmd_pitch}, Thrust={DroneCommand.cmd_thrust}, Yaw={DroneCommand.cmd_yaw}")


def get_mode(node):
    pygame.event.pump()  # Process event queue
    return int(2) if node.controller.get_axis(5) > 0 else int(node.controller.get_axis(5))+1  # SC 3-way switch

def control_loop(node):
    rate = node.create_rate(100)
    log = True
    reboot_time = 5
    while True:
        pygame.event.pump()  # Process event queue
        ax0 = node.controller.get_axis(0)  # Thrust
        ax3 = node.controller.get_axis(3)  # Yaw
        ax2 = node.controller.get_axis(2)  # Pitch
        ax1 = node.controller.get_axis(1)  # Roll

        
        arm = int(node.controller.get_axis(6))+1  # SF 3-way switch 
        estop = float(node.controller.get_axis(4))+1  # SG 3-way switch 0 is normal operation, above 0 is Emergency stop

        Drone_cmd = DroneCommand()

        Drone_cmd.identifier = int(0)
        Drone_cmd.timestamp = float(node.get_clock().now().nanoseconds) / 1e9
        Drone_cmd.cmd_mode = get_mode(node)
        Drone_cmd.cmd_arm = True if arm == 1 else False

        if estop == 1:
            Drone_cmd.cmd_eland = True 
            Drone_cmd.cmd_estop = False
        elif estop > 1:
            Drone_cmd.cmd_eland = False
            Drone_cmd.cmd_estop = True
        else:
            Drone_cmd.cmd_eland = False
            Drone_cmd.cmd_estop = False

        if Drone_cmd.cmd_mode == 1:
            # Send nan values to the drone
            Drone_cmd.cmd_pitch = 0 #int('nan')
            Drone_cmd.cmd_roll = 0 #int('nan')
            Drone_cmd.cmd_thrust = 0 #int('nan')
            Drone_cmd.cmd_yaw = 0 #int('nan')
            time.sleep(0.2)
        elif Drone_cmd.cmd_mode == 0:
            # Map values.   
            Drone_cmd.cmd_pitch = int(np.interp(ax2, (-1, -0.1, 0.1, 1), (-1000, 0, 0, 1000)))  # Roll
            Drone_cmd.cmd_roll = int(np.interp(ax1, (-1, -0.1, 0.1, 1), (-1000, 0, 0, 1000)))   # Pitch
            Drone_cmd.cmd_thrust = int(np.interp(ax0, (-1, -0.1, 0.1, 1), (-1000, 0, 0, 1000)))  # Throttle
            Drone_cmd.cmd_yaw = int(np.interp(ax3, (-1, -0.1, 0.1, 1), (-1000, 0, 0, 1000)))  # Yaw  
        elif Drone_cmd.cmd_mode == 2:
            Drone_cmd.cmd_pitch = int(np.interp(ax2, (-1, -0.1, 0.1, 1), (-1000, 0, 0, 1000)))  # Roll
            Drone_cmd.cmd_roll = int(np.interp(ax1, (-1, -0.1, 0.1, 1), (-1000, 0, 0, 1000)))   # Pitch
            Drone_cmd.cmd_thrust = int(np.interp(ax0, (-1, -0.1, 0.1, 1), (-1000, 0, 0, 1000)))  # Throttle
            Drone_cmd.cmd_yaw = int(np.interp(ax3, (-1, -0.1, 0.1, 1), (-1000, 0, 0, 1000)))  # Yaw  
        
        node.send_control_command(Drone_cmd)

    
        rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    joystick_control_node = JoystickControlNode()
    control_thread = threading.Thread(target=control_loop, args=(joystick_control_node,))
    control_thread.start()
    rclpy.spin(joystick_control_node)
    control_thread.join()
    joystick_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
