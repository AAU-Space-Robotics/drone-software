#!/usr/bin/env python3

import glfw
import warnings
from glfw import GLFWError
warnings.filterwarnings("ignore", category=GLFWError, message=".*Wayland: The platform does not support setting the window position.*")
import imgui
from imgui.integrations.glfw import GlfwRenderer
import OpenGL.GL as gl
from PIL import Image
from OpenGL.GL import *

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import threading
from threading import Thread
from interfaces.msg import DroneState
from interfaces.msg import GcsHeartbeat
from interfaces.action import DroneCommand
from dataclasses import dataclass
from decimal import Decimal
import os

import ament_index_python.packages

import rclpy
from rclpy.node import Node
from interfaces.msg import ManualControlInput
import pygame
from datetime import datetime
from collections import deque

import cv2
import threading
import queue
import numpy as np
from OpenGL.GL import *



class DroneData:
    position: list = (0.0, 0.0, 0.0)
    velocity: list = (0.0, 0.0, 0.0)
    orientation: list = (0.0, 0.0, 0.0)
    battery_voltage: float = 0.0

class ImGuiLogger:
    def __init__(self, max_messages=1000):
        self.messages = deque(maxlen=max_messages)
        self.lock = threading.Lock()
    
    def log(self, level, message):
        with self.lock:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            self.messages.append({
                'level': level,
                'message': message,
                'timestamp': timestamp
            })
    def info(self, message):
        self.log('INFO', message)
    
    def warn(self, message):
        self.log('WARN', message)
    
    def error(self, message):
        self.log('ERROR', message)
    
    def debug(self, message):
        self.log('DEBUG', message)

class GUIButton():
    def button1(x, y, font, label, command, node):
        imgui.set_cursor_pos((x, y))
        imgui.push_style_var(imgui.STYLE_FRAME_ROUNDING, 12.0)
        imgui.push_style_color(imgui.COLOR_BUTTON, *(0.0, 0.5, 0.0))
        imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, *(0.0, 0.8, 0.0))
        imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, *(0.0, 0.2, 0.0)) 
        with imgui.font(font):
            if imgui.button(label, width=150, height=50):
                node.send_command(command, [-1.0])  
        imgui.pop_style_color(3)
        imgui.pop_style_var() 
    def button2(x, y, font, label, command, node, coordinates):
        imgui.set_cursor_pos((x, y))
        imgui.push_style_var(imgui.STYLE_FRAME_ROUNDING, 12.0)
        imgui.push_style_color(imgui.COLOR_BUTTON, *(0.0, 0.5, 0.0))
        imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, *(0.0, 0.8, 0.0))
        imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, *(0.0, 0.2, 0.0)) 
        with imgui.font(font):
            if imgui.button(label, width=150, height=50):
                node.send_command(command, [coordinates])  
        imgui.pop_style_color(3)
        imgui.pop_style_var() 
class graphs():
    def motor_speed_graph(start_x, width, start_y, height):
        color = imgui.get_color_u32_rgba(0.8, 0.8, 0.8, 1.0)
        draw_list = imgui.get_window_draw_list()
        draw_list.add_rect(start_x, width,start_y,height,color, rounding=1.0,flags=15,thickness=3)
    def battery_graph(x_start, width, y_start, height, x_start2, width2, y_start2, height2):
        draw_list = imgui.get_window_draw_list()
        color = imgui.get_color_u32_rgba(0.8, 0.8, 0.8, 1.0) 
        draw_list.add_rect(x_start,width,y_start,height,color, rounding=1.0,flags=15,thickness=3)   #shifted all 320 for fullscreen
        draw_list.add_rect(x_start2, width2, y_start2, height2,color, rounding=1.0,flags=3,thickness=3)  

drone_data = DroneData()

button_color = (0.0,0.5,0.0)
killbutton_color = (0.8,0.0,0.0)
text_buffer = ""
speed_buffer = ""
current_item = 0
position_x, position_y, position_z = 0, 0, 0
target_position_x, target_position_y, target_position_z = 0, 0, 0
roll, pitch, yaw_velocity = 0, 0, 0
velocity_x, velocity_y, velocity_z = 0, 0, 0
thrust = 0
drone_kill = True
test_slider = 0
battery_state_timestamp = 0
position_timestamp = 0
velocity_timestamp = 0
takeoff_time = 0
battery_voltage, battery_current, battery_percentage = 0.0, 0.0, 0.0
battery_discharge_rate, battery_average_current = 0.0, 0.0
arming_state = 0
estop = 0
drone_state = False
GUI_console_logs = [""]
GUI_Heartbeat = 0
actuator_speeds = [0, 0, 0, 0] # Placeholder for actuator speeds
yaw = 0.0

class DroneGuiNode(Node):
    def __init__(self):
        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        super().__init__('thyra_gui_node')
        self.subscription = self.create_subscription(
            DroneState,
            "thyra/out/drone_state",
            self.state_callback,
            10
            
        )
        self.publisher_ = self.create_publisher(
           GcsHeartbeat, 
           "/thyra/in/gcs_heartbeat",
           qos
        )

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.heartbeat_timer = self.create_timer(0.5, self.send_heartbeat)  # 2 Hz
        self.counter = 0.0
        self.get_logger().info('GUI Publisher Started')
        self.imgui_logger = ImGuiLogger()
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.manual_control_publisher = self.create_publisher(ManualControlInput, '/thyra/in/manual_control_input', 10)
        self._action_client = ActionClient(self, DroneCommand, '/thyra/in/drone_command')   
        self.imgui_logger.info('DroneCommand client initialized, waiting for action server...')
        self._action_client.wait_for_server()
        self.goal_handle = None 
        self.log_filters = {
            'show_info': True,
            'show_warn': True,
            'show_error': True,
            'show_debug': False
        }
    
    
    def timer_callback(self):
        # Example of different log levels
        if hasattr(self, 'timer_count'):
            self.timer_count += 1
        else:
            self.timer_count = 1
            
        #if self.timer_count % 50 == 0:  # Every 5 seconds
        #    self.imgui_logger.info(f"System heartbeat - {self.timer_count}")
        
        # Log ROS2 messages to ImGui as well
        #msg = "Timer callback executed"
        #self.get_logger().info(msg)
        #self.imgui_logger.debug(msg)
        


    def state_callback(self, msg):
        global position_x, position_y, position_z, position_timestamp
        global target_position_x, target_position_y, target_position_z
        global roll, pitch, yaw_velocity
        global velocity_x, velocity_y, velocity_z, velocity_timestamp
        global battery_voltage, battery_state_timestamp, battery_current, battery_percentage, battery_discharge_rate, battery_average_current
        global arming_state, flight_mode, takeoff_time
        global GUI_console_logs
        global actuator_speed

        if hasattr(self, 'last_arming_state') and self.last_arming_state != msg.arming_state:
            self.imgui_logger.warn(f"Arming state changed: {self.last_arming_state} -> {msg.arming_state}")
        
        if hasattr(self, 'last_flight_mode') and self.last_flight_mode != msg.flight_mode:
            self.imgui_logger.warn(f"Flight mode changed: {self.last_flight_mode} -> {msg.flight_mode}")
        
        # Store previous states
        self.last_arming_state = msg.arming_state
        self.last_flight_mode = msg.flight_mode


        position_timestamp = msg.position_timestamp
        if len(msg.position) >= 3:
            #Barbre was here....
            position_x = msg.position[0]
            position_y = msg.position[1]
            position_z = msg.position[2]
        velocity_timestamp = msg.velocity_timestamp

        if len(msg.velocity) >= 3:
            velocity_x = msg.velocity[0]
            velocity_y = msg.velocity[1]
            velocity_z = msg.velocity[2]
        if len(msg.orientation) >= 3:
            roll = msg.orientation[0]
            pitch = msg.orientation[1]
            yaw_velocity = msg.orientation[2]
        if len(msg.target_position) >= 3:
            target_position_x = msg.target_position[0]
            target_position_y = msg.target_position[1]
            target_position_z = msg.target_position[2]

        battery_state_timestamp = msg.battery_state_timestamp
        battery_voltage = msg.battery_voltage
        battery_current = msg.battery_current
        battery_percentage = msg.battery_percentage
        battery_discharge_rate = msg.battery_discharged_mah
        battery_average_current = msg.battery_average_current

        if len(msg.actuator_speeds) >= 4:
            actuator_speeds[0] = msg.actuator_speeds[0]  
            actuator_speeds[1] = msg.actuator_speeds[1]  
            actuator_speeds[2] = msg.actuator_speeds[2]  
            actuator_speeds[3] = msg.actuator_speeds[3]  
       



        arming_state = msg.arming_state
        #self.get_logger().info(f"Arming state: {arming_state}")
        flight_mode = msg.flight_mode
        GUI_console_logs[0] = str(self.get_logger())
        #takeoff_time = msg.takeoff_time
        #self.imgui_logger.info(f"Takeoff time: {takeoff_time}")


    def send_command(self, command_type, target_pose=None, yaw=None):
        goal_msg = DroneCommand.Goal()
        goal_msg.command_type = command_type
        if target_pose is not None:
            goal_msg.target_pose = target_pose
        if yaw is not None:
            goal_msg.yaw = yaw
        log_msg = f'Sending command: {command_type}, target_pose: {target_pose} yaw: {yaw}'
        self.get_logger().info(log_msg)
        self.imgui_logger.info(log_msg)
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            msg = 'Goal was rejected by the server'
            self.get_logger().warn(msg)
            self.imgui_logger.warn(msg)
            return
        msg = 'Goal accepted by server, waiting for result...'
        self.get_logger().info(msg)
        self.imgui_logger.info(msg)
        self.goal_handle.get_result_async().add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        msg = f'Feedback received: {feedback_msg.feedback}'
        self.get_logger().info(msg)
        self.imgui_logger.info(msg)

    def result_callback(self, future):
        result = future.result().result
        msg = f'Action completed with result: success={result.success}, message={result.message}'
        self.get_logger().info(msg)
        if result.success:
            self.imgui_logger.info(msg)
        else:
            self.imgui_logger.error(msg)
        self.goal_handle = None
    def send_manual_control(self, roll, pitch, yaw_velocity, thrust):
        msg = ManualControlInput()
        msg.roll = float(roll)
        msg.pitch = float(pitch)
        msg.yaw_velocity = float(yaw_velocity)
        msg.thrust = float(thrust)
        self.manual_control_publisher.publish(msg)
    def send_heartbeat(self):
        msg = GcsHeartbeat()
        msg.timestamp = float(self.get_clock().now().nanoseconds) / 1e9
        msg.gcs_nominal = 1
        self.publisher_.publish(msg)
        

def Arm_Button(node):
    global button_color, drone_kill, drone_state
    global arming_state
    imgui.set_cursor_pos((450, 30))
    button_color = (0.0, 0.5, 0.0) if drone_kill else (1.0, 0.0, 0.0)
    button_text = "Press to Arm!" if drone_kill else "Press to Disarm"
    imgui.push_style_var(imgui.STYLE_FRAME_ROUNDING, 12.0)
    imgui.push_style_color(imgui.COLOR_BUTTON, *button_color)
    imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, *button_color)
    imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, *button_color)
    if arming_state == 1:
        drone_kill = False
        drone_state = True
        button_text = "Press to Disarm"
    elif arming_state == 0:
        drone_kill = True
        drone_state = False
        button_text = "Press to Arm!"
    with imgui.font(font_small):
        if imgui.button(button_text):
            if drone_kill:
                node.send_command("arm")
                drone_kill = False
                drone_state = True
            else:
                node.send_command("disarm")
                drone_kill = True
                drone_state = False
    imgui.pop_style_color(3)
    imgui.pop_style_var()
    imgui.set_cursor_pos((710, 30))
    with imgui.font(font):
        if drone_kill:
            imgui.text("THYRA IS DISARMED!")
        else:
            imgui.text("THYRA IS ARMED!")
    draw_list = imgui.get_window_draw_list()
    start_x, start_y = 450, 70
    end_x, end_y = 1100, 70
    color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5)
    draw_list.add_line(start_x, start_y, end_x, end_y, color, 5.0)

def Kill_command(node):
    global killbutton_color, drone_kill
    imgui.set_cursor_pos((1110, 30))  # Changed x-coordinate to 1130
    imgui.push_style_var(imgui.STYLE_FRAME_ROUNDING, 12.0)
    imgui.push_style_color(imgui.COLOR_BUTTON, *killbutton_color)
    imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, 1.0, 0.0, 0.0, 1.0)  # Fixed: Added alpha
    imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, 0.2, 0.0, 0.0, 1.0)   # Fixed: Added alpha
    with imgui.font(font_large):
        if imgui.button("Kill", width=150, height=100):
            node.send_command("estop")
            drone_kill = True
    imgui.pop_style_color(3)
    imgui.pop_style_var()
    return drone_kill

def Goto_field(node):
    global text_buffer, yaw
    text_field = ""
    imgui.set_cursor_pos((1250,770)) #moved 100 down y-axis for fullscreen
    with imgui.font(font):
        imgui.text("Target Pose (x y -z yaw):")
    imgui.set_cursor_pos((1250,835))
    imgui.set_next_item_width(300)
    imgui.set_window_font_scale(2.0) 
    changed, text_field= imgui.input_text("##goto_input", text_buffer, 64)
    if changed:
        text_buffer = text_field
    imgui.set_window_font_scale(1.0)
 
  

    imgui.set_cursor_pos((1570,825))
    imgui.push_style_var(imgui.STYLE_FRAME_ROUNDING, 12.0)
    imgui.push_style_color(imgui.COLOR_BUTTON, *(0.0, 0.5, 0.0))
    imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, *(0.0,0.8,0.0))
    imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, *(0.0,0.2,0.0)) 
    with imgui.font(font_small):
        if imgui.button("Send",width=70, height=50):
            try:
                try:
                    x, y, z, yaw = map(float, text_buffer.strip().split())
                    node.send_command("goto", [x, y, z], yaw)
                    node.imgui_logger.info(f"Going to position: x={x}, y={y}, z={z}, yaw={yaw}")

                except ValueError:
                    x, y, z =map(float, text_buffer.strip().split())
                    node.send_command("goto", [x, y, z], yaw)
                    node.imgui_logger.info(f"Going to position: x={x}, y={y}, z={z}")
            except:
                node.get_logger().warn("Invalid input for goto, please enter x y z yaw values")
                node.imgui_logger.warn("Invalid input for goto, please enter x y z yaw values")

            
                
    imgui.pop_style_color(3)
    imgui.pop_style_var()

def speed_field(node):
    global speed_buffer
    speed_text_field = ""
    imgui.set_cursor_pos((1250, 650))  
    with imgui.font(font):
        imgui.text("Speed (m/s):")
    imgui.set_cursor_pos((1250, 715))
    imgui.set_next_item_width(300)
    imgui.set_window_font_scale(2.0)
    changed, speed_text_field = imgui.input_text("##speed_input", speed_buffer, 64)
    if changed:
        speed_buffer = speed_text_field
    imgui.set_window_font_scale(1.0)
    imgui.set_cursor_pos((1570, 705))
    imgui.push_style_var(imgui.STYLE_FRAME_ROUNDING, 12.0)
    imgui.push_style_color(imgui.COLOR_BUTTON, *(0.0, 0.5, 0.0))
    imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, *(0.0, 0.8, 0.0))
    imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, *(0.0, 0.2, 0.0))
    with imgui.font(font_small):
        if imgui.button("Send S", width=70, height=50):
            try:
                speed = float(speed_buffer.strip())
                node.send_command("set_linear_speed", [speed])  
                node.get_logger().info(f"Speed set to {speed}")
                node.imgui_logger.info(f"Speed set to {speed}")
            except ValueError as e:
                node.get_logger().warn(f"Invalid speed input: {e}")
                node.imgui_logger.warn(f"Invalid speed input: {e}")
    imgui.pop_style_color(3)
    imgui.pop_style_var()

    
def Dropdown_Menu():
    global current_item

    items = ["No Controller", "PS4", "TX16S"]
    imgui.set_cursor_pos((450,90))
    imgui.set_next_item_width(300)

    with imgui.font(font_small):
        changed, current_item = imgui.combo(
            "Controller", current_item, items)
    #imgui.set_cursor_pos((10,570))
    #imgui.text(f"{str(items[current_item])}")

def XYZ_Text_Field(msg):
    #Drawing a square kek
    global position_x, position_y, position_z
    global target_position_x, target_position_y, target_position_z
    global position_timestamp
    #position_x, position_y, position_z = drone_data.position
    #position_x = int(msg.position[0])
    #print(f"Position: {position_x}, {position_y}, {position_z}")
   
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5) 
                                                                #flags is for rounding different corners
    draw_list.add_rect_filled(20,90, 420,235,color,rounding =10.0, flags=10)
    
    with imgui.font(font_small):
        imgui.set_cursor_pos((23,50)); imgui.text("Positon:| Current:| Target: ")
        imgui.set_cursor_pos((60,95)); imgui.text("X = ")
        imgui.set_cursor_pos((60,145)); imgui.text("Y = ")
        imgui.set_cursor_pos((60,195)); imgui.text("Z = ")
        imgui.set_cursor_pos((170,93)); imgui.text(f"{Decimal(position_x).quantize(Decimal('0.000'))}")
        imgui.set_cursor_pos((170,143)); imgui.text(f"{Decimal(position_y).quantize(Decimal('0.000'))}")
        imgui.set_cursor_pos((170,193)); imgui.text(f"{(Decimal(position_z).quantize(Decimal('0.000')))}")

        imgui.set_cursor_pos((313,93)); imgui.text(f"{Decimal(target_position_x).quantize(Decimal('0.000'))}")
        imgui.set_cursor_pos((313,143)); imgui.text(f"{Decimal(target_position_y).quantize(Decimal('0.000'))}")
        imgui.set_cursor_pos((313,193)); imgui.text(f"{(Decimal(target_position_z).quantize(Decimal('0.000')))}")
    with imgui.font(font_for_meter):
        imgui.set_cursor_pos((240,240)); imgui.text("*measure in meters")
        imgui.set_cursor_pos((405,42)); imgui.text("*")

            # Ending point of the line (x, y)
        draw_list = imgui.get_window_draw_list()
        start_x, start_y = 145, 90  # Starting point of the line (x, y)
        end_x, end_y = 145, 235      # Ending point of the line (x, y)
        color = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 1.0) 
        draw_list.add_line(start_x,start_y, end_x, end_y, color, 5.0)

        draw_list = imgui.get_window_draw_list()
        start_x, start_y = 287, 90  # Starting point of the line (x, y)
        end_x, end_y = 287, 235      # Ending point of the line (x, y)
        color = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 1.0) 
        draw_list.add_line(start_x,start_y, end_x, end_y, color, 5.0)
    imgui.set_cursor_pos((23, 265)); imgui.text(f" Position Timestamp {position_timestamp}")
    
def RPY_Text_Field():
    
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5)
    draw_list.add_rect_filled(20,565, 250,710,color,rounding =10.0, flags=10)
    thrust = -(int(test_slider))

    with imgui.font(font_small):
        imgui.set_cursor_pos((23,525)); imgui.text("Orientation:")
    with imgui.font(font_small):
        imgui.set_cursor_pos((30,570)); imgui.text("Roll  = ")
        imgui.set_cursor_pos((30,620)); imgui.text("Pitch = ")
        imgui.set_cursor_pos((30,670)); imgui.text("Yaw   = ")
        imgui.set_cursor_pos((150,568)); imgui.text(f"{Decimal(roll).quantize(Decimal('0.00'))}")
        imgui.set_cursor_pos((150,618)); imgui.text(f"{Decimal(pitch).quantize(Decimal('0.00'))}")
        imgui.set_cursor_pos((150,668)); imgui.text(f"{Decimal(yaw_velocity).quantize(Decimal('0.00'))}")

def XYZVelocity_Text_Field():

        global velocity_x, velocity_y, velocity_z, velocity_timestamp

        draw_list = imgui.get_window_draw_list()
        color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5)
     
        draw_list.add_rect_filled(20,335, 250,480,color,rounding =10.0, flags=10)

        with imgui.font(font_small):
            imgui.set_cursor_pos((23,295)); imgui.text("Velocity:")
            imgui.push_style_color(imgui.COLOR_TEXT, 1.0, 0.0, 0.0, 0.9)
            imgui.set_cursor_pos((30,340)); imgui.text("X = ")
            imgui.set_cursor_pos((100,338)); imgui.text(f"{Decimal(velocity_x).quantize(Decimal('0.000'))}")
            imgui.set_cursor_pos((200,338)); imgui.text("m/s")
            imgui.pop_style_color()
            imgui.push_style_color(imgui.COLOR_TEXT, 0.0, 1.0, 0.0, 0.9)
            imgui.set_cursor_pos((30,395)); imgui.text("Y = ")
            imgui.set_cursor_pos((100,393)); imgui.text(f"{Decimal(velocity_y).quantize(Decimal('0.000'))}")
            imgui.set_cursor_pos((200,393)); imgui.text("m/s")
            imgui.pop_style_color()
            imgui.push_style_color(imgui.COLOR_TEXT, 0.0, 0.0, 0.5, 0.9)
            imgui.set_cursor_pos((30,445)); imgui.text("Z = ")
            imgui.set_cursor_pos((100,443)); imgui.text(f"{-(Decimal(velocity_z).quantize(Decimal('0.000')))}")
            imgui.set_cursor_pos((200,443)); imgui.text("m/s")
            imgui.pop_style_color()
        imgui.set_cursor_pos((23, 490)); imgui.text(f" Velocity Timestamp {velocity_timestamp}")
    
def batteryGraph():
    global battery_voltage, battery_current, battery_percentage, battery_average_current
    battery_progressbar = map_value(battery_percentage, 0, 1, 109, 44)
    #draw_list = imgui.get_window_draw_list()
    #color = imgui.get_color_u32_rgba(0.8, 0.8, 0.8, 1.0) 
    #draw_list.add_rect(1845,40,1885,110,color, rounding=1.0,flags=15,thickness=3)   #shifted all 320 for fullscreen
    #draw_list.add_rect(1852,31,1877,38,color, rounding=1.0,flags=3,thickness=3)  
    graphs.battery_graph(1845, 40, 1885, 110, 1852, 31, 1877, 38)
    if(battery_percentage > 0.5):
        battery_color = imgui.get_color_u32_rgba(0.0, 1.0, 0.0, 1.0)
        
    elif(battery_percentage > 0.25):
        battery_color = imgui.get_color_u32_rgba(1.0, 1.0, 0.0, 1.0)
    else:
        battery_color = imgui.get_color_u32_rgba(1.0, 0.0, 0.0, 1.0)
    imgui.set_cursor_pos((1844, 116)); imgui.text(f"{Decimal(100*battery_percentage).quantize(Decimal('0.00'))} %")
    #draw_list.add_rect_filled(1465,31,1432+(battery_percentage*25),38,color, rounding=1.0,flags=3)
    #print(battery_voltage)
    draw_list = imgui.get_window_draw_list()
    draw_list.add_rect_filled(1848,106,1882,(battery_progressbar),battery_color, rounding=1.0,flags=15)

    imgui.set_cursor_pos((1640, 30)); imgui.text(f"Voltage:          {Decimal(battery_voltage).quantize(Decimal('0.0'))} V")
    imgui.set_cursor_pos((1640, 60)); imgui.text(f"Current:          {-(Decimal(battery_current).quantize(Decimal('0.0')))} A")
    imgui.set_cursor_pos((1640, 90)); imgui.text(f"Discharge rate:   {Decimal(battery_discharge_rate).quantize(Decimal('0.0'))} mAh")
    imgui.set_cursor_pos((1640, 120)); imgui.text(f"Average current:  {-(Decimal(battery_average_current).quantize(Decimal('0.0')))} A")

    draw_list = imgui.get_window_draw_list()
    start_x, start_y = 1830, 30  # Starting point of the line (x, y)
    end_x, end_y = 1830, 132      # Ending point of the line (x, y)
    color = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 1.0) 
    draw_list.add_line(start_x,start_y, end_x, end_y, color, 2.0)

    draw_list = imgui.get_window_draw_list()
    start_x, start_y = 1759, 30  # Starting point of the line (x, y)
    end_x, end_y = 1759, 132      # Ending point of the line (x, y)
    color = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 1.0) 
    draw_list.add_line(start_x,start_y, end_x, end_y, color, 2.0)

    draw_list = imgui.get_window_draw_list()
    start_x, start_y = 1630, 30  # Starting point of the line (x, y)
    end_x, end_y = 1630, 132      # Ending point of the line (x, y)
    color = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 1.0) 
    draw_list.add_line(start_x,start_y, end_x, end_y, color, 2.0)

    imgui.set_cursor_pos((1620, 175)); imgui.text(f" Battery Timestamp  {battery_state_timestamp}") 

def map_value(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def drone_visualization():
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5) 
                                                                #flags is for rounding different corners
    draw_list.add_rect(450,150, 1100,560,color,rounding =10.0, flags=15,thickness=6)
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 0.9) 
                                                                #flags is for rounding different corners
    draw_list.add_rect_filled(453,153, 1097,567,color,rounding =10.0, flags=15)
          
def Arrows():
    slider_value = 0.0  # default

    # Set size of the slider
    imgui.set_cursor_pos((450, 600)); imgui.set_next_item_width(300)

    # Slider with range from 0 to 10
    #changed, slider_value = imgui.slider_float("Scale Me", slider_value, 0.0, 100.0)

    # Show the current value

    #Arrows 90 PÅ X ASKEN
    #z-axis
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.0, 0.0, 1.0, 0.9)
    #draw_list.add_triangle_filled(660, (-velocity_z * 10)+325, 685, (-velocity_z * 10)+325, 672.5, (-velocity_z * 10)+300, color)
    if(-velocity_z) > 0:
        draw_list.add_triangle_filled(660, (velocity_z * 10)+325, 685, (velocity_z * 10)+325, 672.5, (velocity_z * 10)+300, color)
        draw_list.add_rect_filled(670, 325, 675, (velocity_z * 10)+305, color, rounding=2.0)
    else:
        draw_list.add_triangle_filled(660, (velocity_z * 10)+325, 685, (velocity_z * 10)+325, 672.5, (velocity_z * 10)+350, color)
        draw_list.add_rect_filled(670, 325, 675, (velocity_z * 10)+345, color, rounding=2.0)

    
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.0, 0.0, 1.0, 0.9)
    
    #x-axis
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(1.0, 0.0, 0.0, 0.9)
    #draw_list.add_triangle_filled((velocity_x* 10)+830, 450, (velocity_x* 10)+830, 475, (velocity_x* 10)+855, 462.5, color)
    #draw_list = imgui.get_window_draw_list()
    #color = imgui.get_color_u32_rgba(1.0, 0.0, 0.0, 0.9)
    #draw_list.add_rect_filled(805, 460, (velocity_x* 10)+830, 465, color, rounding=2.0)

    if velocity_x > 0:
        draw_list.add_triangle_filled((velocity_x* 10)+830, 450, (velocity_x* 10)+830, 475, (velocity_x* 10)+855, 462.5, color)
        draw_list.add_rect_filled(830, 460, (velocity_x* 10)+835, 465, color, rounding=2.0)
    else:
        draw_list.add_triangle_filled((velocity_x* 10)+830, 450, (velocity_x* 10)+830, 475, (velocity_x* 10)+805, 462.5, color)
        draw_list.add_rect_filled(830, 460, (velocity_x* 10)+825, 465, color, rounding=2.0)
    
    
    
    #y-axis
    # Arrow shaft (thin diagonal rectangle or just a line)
    # Arrowhead (triangle at the end)
    # Position the triangle to point diagonally
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.0, 1.0, 0.0, 0.9)  # Green arrow
    
    if velocity_y > 0:
        start_x, start_y = 813, 334
        end_x, end_y = 830 +(velocity_y * 10), 320 - (velocity_y * 10)
        draw_list.add_line(start_x, start_y, end_x, end_y, color, 5.0)
        draw_list.add_triangle_filled(
        end_x+3, end_y-3,          # tip
        end_x - 23, end_y + 5,  # base left
        end_x - 5, end_y + 23,  # base right
        color
        )
    else:  
        start_x, start_y = 813, 334
        end_x, end_y = 810 -(-velocity_y * 10), 340 +(-velocity_y * 10)
        draw_list.add_line(start_x, start_y, end_x, end_y, color, 5.0)
        draw_list.add_triangle_filled(
        end_x-3, end_y+3,          # tip
        end_x + 23, end_y - 5,  # base left
        end_x + 5, end_y - 23,  # base right
        color
        )

def return_to_home_button(node):
    imgui.set_cursor_pos((1650, 580))
    imgui.push_style_var(imgui.STYLE_FRAME_ROUNDING, 12.0)
    imgui.push_style_color(imgui.COLOR_BUTTON, *(0.0, 0.5, 0.0))
    imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, *(0.0, 0.8, 0.0))
    imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, *(0.0, 0.2, 0.0))  
    with imgui.font(font_small):
        if imgui.button("Return", width=150, height=50):
            node.send_command("goto", [float(0.0), float(0.0), float(-1.5)]) 
    imgui.pop_style_color(3)
    imgui.pop_style_var()

def manual(node):
    imgui.set_cursor_pos((1400, 200))
    button_color = (0.5, 0.5, 0.5)
    hover_color = (0.5, 0.8, 0.5)
    active_color = (0.5, 1, 0.5)
    imgui.push_style_var(imgui.STYLE_FRAME_ROUNDING, 12.0)
    imgui.push_style_color(imgui.COLOR_BUTTON, *button_color)
    imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, *hover_color)
    imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, *active_color)  
    with imgui.font(font_small):
        if imgui.button("Manual", width=200, height=50):
            try :
                # Check if joystick is connected
                if pygame.joystick.get_count() > 0:
                    button_color = (0.0, 0.5, 0.0)  # Green color for connected joystick
                    hover_color = (0.0, 0.8, 0.0)  # Lighter green for hover
                    active_color = (0.0, 0.2, 0.0)
                    node.send_command("manual")
                    GuiConsoleLogger("Manual mode activated.")
                else:
                    print("Joystick not connected or not initialized.")
            except AttributeError:
                print("Joystick not initialized or not available.")
              
   
    imgui.pop_style_color(3)
    imgui.pop_style_var()
    flight_mode_text = ""
    if flight_mode == -2:
        flight_mode_text = "Landed"
    elif flight_mode == -1:
        flight_mode_text = "Standby"
    elif flight_mode == 0:
        flight_mode_text = "Manual"
    elif flight_mode == 1:
        flight_mode_text = "Manual Aided"
    elif flight_mode == 2:
        flight_mode_text = "Position"
    elif flight_mode == 3:
        flight_mode_text = "Safetyland Blind "
    elif flight_mode == 4:
        flight_mode_text = "Begin land position"
    elif flight_mode == 5:
        flight_mode_text = "Land position"
    
    imgui.set_cursor_pos((1250, 710))
    with imgui.font(font_small):
        imgui.text("Flight Mode: " + str(flight_mode_text))

def start_ros(node):
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

def start_joystick(node):
    global roll, pitch, yaw_velocity, thrust, arming_state
    global drone_kill, drone_state
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick connected.")
        return
    
    node.joystick = pygame.joystick.Joystick(0)
    node.joystick.init()

    print(f"Initialized joystick: {node.joystick.get_name()}")
    
    if not node.joystick.get_init():
        print("Joystick initialization failed.")
        return
    
  

    clock = pygame.time.Clock()
    DEAD_ZONE = 0.05
    prev_axis_state = None

    try:
        while rclpy.ok():
            pygame.event.pump()  # Process internal queue
            #while arming_state == 7:
            #print(f"Flight mode: {flight_mode}")
            roll = node.joystick.get_axis(0) if abs(node.joystick.get_axis(0)) > DEAD_ZONE else 0.0
            pitch = -node.joystick.get_axis(1) if abs(node.joystick.get_axis(1)) > DEAD_ZONE else 0.0
            yaw_velocity = node.joystick.get_axis(2) if abs(node.joystick.get_axis(2)) > DEAD_ZONE else 0.0
            thrust = -node.joystick.get_axis(3) if abs(node.joystick.get_axis(3)) > DEAD_ZONE else 0.0
            current_axis_state = int(abs(node.joystick.get_axis(4)))
            if prev_axis_state is None or current_axis_state != prev_axis_state:
                if current_axis_state == 1:
                    node.send_command("disarm")
                    #print("Arming state changed to: Disarmed (0)")  # Debug
                elif current_axis_state == 0:
                    node.send_command("arm")
                    #print("Arming state changed to: Armed (1)")  # Debug
                prev_axis_state = current_axis_state
            #arming_state = -int(abs(node.joystick.get_axis(4)))
            #print(f"Arming state: {arming_state}")
            #print(int(abs(node.joystick.get_axis(4))))
        
            if( node.joystick.get_button(3) == 1):
                node.send_command("estop")
                drone_kill = True
        
            node.send_manual_control(roll, pitch, yaw_velocity, thrust)
            clock.tick(20)  # 20 Hz update rate
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()

def GuiConsoleLogger(node):
    global font_small
    
    # Position the logger window in the bottom right area
    imgui.set_cursor_pos((450, 600))
    
    # Create a child window for the logger
    imgui.begin_child("LoggerWindow", 750, 280, border=True)
    
    # Header
    with imgui.font(font_small):
        imgui.text("Console Logger")
    
    imgui.separator()
    
    # Filter checkboxes
    imgui.text("Filters:")
    imgui.same_line()
    
    # Use persistent filter states from the node
    node.log_filters['show_info'] = imgui.checkbox("INFO", node.log_filters['show_info'])[1]
    imgui.same_line()
    node.log_filters['show_warn'] = imgui.checkbox("WARN", node.log_filters['show_warn'])[1]
    imgui.same_line()
    node.log_filters['show_error'] = imgui.checkbox("ERROR", node.log_filters['show_error'])[1]
    imgui.same_line()
    node.log_filters['show_debug'] = imgui.checkbox("DEBUG", node.log_filters['show_debug'])[1]
    
    # Clear button
    imgui.same_line()
    if imgui.button("Clear"):
        with node.imgui_logger.lock:
            node.imgui_logger.messages.clear()
    
    imgui.separator()
    
    # Scrollable log area
    imgui.begin_child("ScrollingRegion", 0, 0, border=False)
    
    # Display log messages
    with node.imgui_logger.lock:
        for msg in node.imgui_logger.messages:
            # Filter by level
            if (msg['level'] == 'INFO' and not node.log_filters['show_info']) or \
               (msg['level'] == 'WARN' and not node.log_filters['show_warn']) or \
               (msg['level'] == 'ERROR' and not node.log_filters['show_error']) or \
               (msg['level'] == 'DEBUG' and not node.log_filters['show_debug']):
                continue
            
            # Format the message
            formatted_msg = f"[{msg['timestamp']}] {msg['level']}: {msg['message']}"
            
            # Color based on log level
            if msg['level'] == 'ERROR':
                imgui.text_colored(formatted_msg, 1.0, 0.4, 0.4, 1.0)
            elif msg['level'] == 'WARN':
                imgui.text_colored(formatted_msg, 1.0, 0.8, 0.0, 1.0)
            elif msg['level'] == 'INFO':
                imgui.text_colored(formatted_msg, 0.4, 1.0, 0.4, 1.0)
            elif msg['level'] == 'DEBUG':
                imgui.text_colored(formatted_msg, 0.7, 0.7, 0.7, 1.0)
            else:
                imgui.text(formatted_msg)
    
    # Auto-scroll to bottom if we're at the bottom
    if imgui.get_scroll_y() >= imgui.get_scroll_max_y():
        imgui.set_scroll_here_y(1.0)
    
    imgui.end_child()
    imgui.end_child()

def motor_speed():
    global actuator_speeds   
    actuator_speeds_slider_bar1 = map_value(actuator_speeds[0], 0, 1000, 106, 54)
    actuator_speeds_slider_bar2 = map_value(actuator_speeds[1], 0, 1000, 106, 54)
    actuator_speeds_slider_bar3 = map_value(actuator_speeds[2], 0, 1000, 106, 54)
    actuator_speeds_slider_bar4 = map_value(actuator_speeds[3], 0, 1000, 106, 54)
    graphs.motor_speed_graph(1300, 50, 1360, 110) 
    graphs.motor_speed_graph(1380, 50, 1440, 110)
    graphs.motor_speed_graph(1460, 50, 1520, 110)
    graphs.motor_speed_graph(1540, 50, 1600, 110) 
    standardcolor = imgui.get_color_u32_rgba(0.8, 0.8, 0.8, 1.0)
    color1 = standardcolor
    color2 = standardcolor
    color3 = standardcolor
    color4 = standardcolor
    if actuator_speeds[0] > 900:
        color1 = imgui.get_color_u32_rgba(1.0, 0.0, 0.0, 1.0)
    if actuator_speeds[1] > 900:
        color2 = imgui.get_color_u32_rgba(1.0, 0.0, 0.0, 1.0)
    if actuator_speeds[2] > 900:
        color3 = imgui.get_color_u32_rgba(1.0, 0.0, 0.0, 1.0)
    if actuator_speeds[3] > 900:
        color4 = imgui.get_color_u32_rgba(1.0, 0.0, 0.0, 1.0)
    draw_list = imgui.get_window_draw_list()
    draw_list.add_rect_filled(1303,106,1357,actuator_speeds_slider_bar1,color1, rounding=1.0,flags=15)   
    imgui.set_cursor_pos((1304, 120)); imgui.text(f"{Decimal(actuator_speeds[0]).quantize(Decimal('0.00'))}")
    draw_list = imgui.get_window_draw_list()
    draw_list.add_rect_filled(1383,106,1437,actuator_speeds_slider_bar2,color2, rounding=1.0,flags=15)
    imgui.set_cursor_pos((1384, 120)); imgui.text(f"{Decimal(actuator_speeds[1]).quantize(Decimal('0.00'))}")
    draw_list = imgui.get_window_draw_list()
    draw_list.add_rect_filled(1463,106,1517,actuator_speeds_slider_bar3,color3, rounding=1.0,flags=15)
    imgui.set_cursor_pos((1464, 120)); imgui.text(f"{Decimal(actuator_speeds[2]).quantize(Decimal('0.00'))}")
    draw_list = imgui.get_window_draw_list()
    draw_list.add_rect_filled(1543,106,1597,actuator_speeds_slider_bar4,color4, rounding=1.0,flags=15)
    imgui.set_cursor_pos((1544, 120)); imgui.text(f"{Decimal(actuator_speeds[3]).quantize(Decimal('0.00'))}")
    with imgui.font(font_small):
        imgui.set_cursor_pos((1320, 20)); imgui.text("M1")
        imgui.set_cursor_pos((1400, 20)); imgui.text("M2")
        imgui.set_cursor_pos((1480, 20)); imgui.text("M3")
        imgui.set_cursor_pos((1560, 20)); imgui.text("M4")

    
def main(args=None):
    rclpy.init()
    global font, font_large, font_small, font_for_meter
    node = DroneGuiNode()  # Create node once
    Thread(target=start_ros, args=(node,), daemon=True).start()
    Thread(target=start_joystick, args=(node,), daemon=True).start()
    # Initialize GLFW
    if not glfw.init():
        print("Could not initialize GLFW")
        return
    
    # Set window hints for borderless full-screen mode

    glfw.init()
    monitor = glfw.get_primary_monitor()
    video_mode = glfw.get_video_mode(monitor)
    screen_width = video_mode.size.width
    screen_height = video_mode.size.height
    

    glfw.window_hint(glfw.RESIZABLE, glfw.TRUE)
    glfw.window_hint(glfw.DECORATED, glfw.TRUE)
    glfw.window_hint(glfw.MAXIMIZED, glfw.TRUE)

    window = glfw.create_window(screen_width, screen_height, "THYRA", None, None)
    glfw.set_window_pos(window, 0, 0)
    if not window:
        print("Could not create GLFW window")
        glfw.terminate()  
        return
    
    # Make the OpenGL context current
    glfw.make_context_current(window)

    # Initialize ImGui

    #Not quiet sure what to do about this yet....
    imgui.create_context()
    #io = imgui.get_io()
    #io.config_flags |= imgui.CONFIG_NAV_ENABLE_KEYBOARD
    
    impl = GlfwRenderer(window)
    io = imgui.get_io()
    global font, font_large, font_small, font_for_meter
    # Try to find the font in the ROS package share directory
    
    try:
        font_dir = os.path.join(
            ament_index_python.packages.get_package_share_directory("gcs"),
            "fonts", "source-code-pro"
        )
        font_path = os.path.join(font_dir, "SourceCodePro-Black.otf")
    except Exception as e:
        print(f"Could not find gcs package share directory: {e}")
        glfw.terminate()
        return
    font_path = os.path.normpath(font_path)
    if not os.path.isfile(font_path):
        print(f"Font file not found: {font_path}")
        glfw.terminate()
        return
    font = io.fonts.add_font_from_file_ttf(font_path, 40)
    font_large = io.fonts.add_font_from_file_ttf(font_path, 50)
    font_small = io.fonts.add_font_from_file_ttf(font_path, 30)
    font_for_meter = io.fonts.add_font_from_file_ttf(font_path, 20)
    impl.refresh_font_texture()


    texture_id = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, texture_id)
    
    # Set texture parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
   
    
    # Main loop
    while not glfw.window_should_close(window):
        glfw.poll_events()
        impl.process_inputs()
        clear_color = (0.0, 0.0, 0.12, 0.0)  # RGBA 
        gl.glClearColor(*clear_color)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        imgui.new_frame()
      
        # Create UI without windowed mode
        imgui.set_next_window_position(0, 0)
       
        imgui.set_next_window_size(screen_width, screen_height)
        imgui.begin("wtf", flags=imgui.WINDOW_NO_TITLE_BAR | imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE | imgui.WINDOW_NO_BACKGROUND)
        imgui.set_cursor_pos((20,10))
        with imgui.font(font_small):
            imgui.text("THYRA state monitor")
        #Creating a line for seperation
        draw_list = imgui.get_window_draw_list()
        start_x, start_y = 430, 0  # Starting point of the line (x, y)
        end_x, end_y = 430, 2000      # Ending point of the line (x, y)
        color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5) 
        draw_list.add_line(start_x,start_y, end_x, end_y, color, 5.0)
        #Running different widgets
        drone_visualization()
       
        Arm_Button(node)
        Kill_command(node)
        Goto_field(node)
        speed_field(node)
        Dropdown_Menu()
        XYZ_Text_Field(msg=drone_data)
        RPY_Text_Field()
        XYZVelocity_Text_Field()
        
        batteryGraph()
        motor_speed()
        #Arrows()   
        #takeoff_button(node)
        #land_button(node)
        return_to_home_button(node)
        #manual(node) #to be continued
        GuiConsoleLogger(node)
        GUIButton.button1(1450, 580, font_small, "Land", "land", node)
        GUIButton.button2(1250, 580, font_small, "Takeoff", "takeoff", node, -1.0)
        GUIButton.button1(1250, 490, font_small, "Set Origin", "set_origin", node)
        imgui.end()

        
        # Render
        
        imgui.render()
        impl.render(imgui.get_draw_data())
        glfw.swap_buffers(window)
    
    # Cleanup

    impl.shutdown()
    glfw.terminate()
    node.destroy_node()

if __name__ == "__main__":
    main()
    


    