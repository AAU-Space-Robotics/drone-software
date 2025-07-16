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
        self.controller_type = 1 #to choose controller. 1 for TX16S and 0 for PS4


        self.publisher_ = self.create_publisher(ManualControlInput, 'thyra/in/manual_input', qos_settings)
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
            self.get_logger().warn('No joystick '
            'detected!')
    def controller_choice(self):
        if not(self.controller_type):
            """Reads input from the PS4 controller and updates the drone_cmd."""
            self.read_PS4()
        elif (self.controller_type):
            """Reads input from the TX16S controller and updates the drone_cmd."""
            self.read_TX16S()
    def read_TX16S(self):
        """Reads input from the TX16S controller and updates the drone_cmd."""
        if self.joystick is None:
            return
        
        pygame.event.pump()  # Process pygame events
        #axis 0 = right stick x-axis (curretly roll)
        #axis 1 = right stick y-axis (currently pitch)
        #axis 2 = left stick y-axis (currently thrust)
        #axis 3 = left stick x-axis (currently yaw)
        
        #axis 4 = SF (currently arm)
        #axis 5 = SB
        #axis 6 = SC
        
        #button 0 = SE
        #button 1 = SA 
        #button 2 = SD (currently estop)
        #button 3 = SH (This is a hold down switch?) 
        

        #not used right now |
        #                   V
        #left_trigger = ((self.joystick.get_axis(2) + 1.0)/2.0)
        #right_trigger = (self.joystick.get_axis(5) + 1.0)/2.0
        #yaw_value = right_trigger - left_trigger
       
      
        
        self.drone_cmd.roll = self.joystick.get_axis(0) if abs(self.joystick.get_axis(0)) > 0.05 else 0.0 # Left stick X-axis
        self.drone_cmd.pitch = -self.joystick.get_axis(1)# if abs(self.joystick.get_axis(1)) > 0.05 else 0.0 # Left stick Y-axis (inverted for correct mapping)
        self.drone_cmd.yaw_velocity =self.joystick.get_axis(3) if abs(self.joystick.get_axis(3)) > 0.05 else 0.0 # Right stick X-axis
        #self.drone_cmd.yaw_velocity =yaw_value if abs(yaw_value) > 0.02 else 0.0 # Right stick X-axis
        self.drone_cmd.thrust = -self.joystick.get_axis(2) if abs(self.joystick.get_axis(2)) > 0.05 else 0.0 # Right stick Y-axis, mapped from [-1,1] to [0,-1]
        
        #if(int(abs(self.joystick.get_button(3)))): #to flip the direction of arm to match controller
        #    self.drone_cmd.arm = 0
        #else:
        #    self.drone_cmd.arm = 1
        self.drone_cmd.arm = int(abs(self.joystick.get_button(3)))
     
        self.drone_cmd.estop = self.joystick.get_button(2)
        
        #self.hello = self.joystick.get_button(23) #command to test which buttons
        
        self.drone_cmd.estop = self.joystick.get_button(2)
        #note to self, yaw left = -1 yaw right = 1
        # thrust up = -1 thrust down = 1
        # roll left -1 roll right 1
        # pitch up -1 down 1

    def read_PS4(self):
        """Reads input from the PS4 controller and updates the drone_cmd."""
        if self.joystick is None:
            return
        
        pygame.event.pump()  # Process pygame events
        #axis 2 is the left trigger
        #axis 5 is the right trigger
        
        left_trigger = ((self.joystick.get_axis(2) + 1.0)/2.0)
        right_trigger = (self.joystick.get_axis(5) + 1.0)/2.0
        yaw_value = right_trigger - left_trigger
        
        # Read joystick axes (values range from -1 to 1)
        self.drone_cmd.roll = self.joystick.get_axis(0) if abs(self.joystick.get_axis(0)) > 0.05 else 0.0  # Left stick X-axis
        self.drone_cmd.pitch = self.joystick.get_axis(1) if abs(self.joystick.get_axis(1)) > 0.05 else 0.0  # Left stick Y-axis (inverted for correct mapping)
        #self.drone_cmd.yaw_velocity =self.joystick.get_axis(3) if abs(self.joystick.get_axis(3)) > 0.05 else 0.0  # Right stick X-axis
        self.drone_cmd.yaw_velocity =yaw_value if abs(yaw_value) > 0.02 else 0.0  # Right stick X-axis
        self.drone_cmd.thrust = self.joystick.get_axis(4) if abs(self.joystick.get_axis(4)) > 0.05 else 0.0  # Right stick Y-axis, mapped from [-1,1] to [0,-1]
 

    def publish_control(self):
        self.controller_choice()
        self.publisher_.publish(self.drone_cmd)
        self.get_logger().info(
            f'Publishing: Roll={self.drone_cmd.roll:.2f}, Pitch={self.drone_cmd.pitch:.2f}, '
            f'Yaw={self.drone_cmd.yaw_velocity:.2f}, Thrust={self.drone_cmd.thrust:.2f}, '
            f'Arm={self.drone_cmd.arm}, estop={self.drone_cmd.estop}, '
            #print(self.hello)
        )
def start_ros():
    rclpy.init()
    node = DroneGuiNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
def main(args=None):
    rclpy.init(args=args)
    node = ManualController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == '__main__':
    main()