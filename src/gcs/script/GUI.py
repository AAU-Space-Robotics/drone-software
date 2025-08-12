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
from interfaces.msg import ProbeLocations
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
import time
import cv2
import queue
import numpy as np
from OpenGL.GL import *
import re
import math

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
    def button_Plan(x, y, font, label):
        state = False
        imgui.set_window_font_scale(0.75)
        imgui.set_cursor_pos((x, y))
        imgui.push_style_var(imgui.STYLE_FRAME_ROUNDING, 12.0)
        imgui.push_style_color(imgui.COLOR_BUTTON, *(0.0, 0.5, 0.0))
        imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, *(0.0, 0.8, 0.0))
        imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, *(0.0, 0.2, 0.0)) 
        with imgui.font(font):
            if imgui.button(label, width=120, height=40):
                state = True
                
        imgui.pop_style_color(3)
        imgui.pop_style_var() 
        imgui.set_window_font_scale(1.0)
        return state


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
text_buffer, text_buffer_plan = "", ""
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
mouse_x_buffer, mouse_y_buffer = 0,0
effect1, effect2, effect3 = False, False, False
effect_begin1, effect_begin2, begin_execute, effect_begin3 = False, False, False, False
orange_effect_going = [0,0,0,0,0,0,0,0,0,0]
duration, plan_duration = 0.3, 5 
start_time = None
change_x, change_y_1, change_y_2 = 0,0, 0
permant_y, card_y_buffer = 190, 190
flight_plan = [0,0,0,0,0,0,0,0,0,0]
flight_plan_coord = [[""] for _ in range(10)]
flight_plan_numb, execute_route_numb, visual_card_numb = 0 , 0, 0
current_step, tep_start_time = 0, 0
command_sent, waiting_for_completion = False, False
probes = []
probe_classification = 0
probe_numb = 0
temp_y_arrow, start_y_arrow = 0, 0

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
        self.subscription = self.create_subscription(
            ProbeLocations,
            '/thyra/out/probe_locations_global',
            self.probe_callback,
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
        self.manual_control_publisher = self.create_publisher(ManualControlInput, '/thyra/in/manual_input',qos)
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

    def probe_callback(self, msg):
        global probes
        global probe_numb, probe_classification

        probe_numb = msg.num_probes 

        probe_classification = msg.classification_confidence

        probes = msg.probes



    def send_command(self, command_type, target_pose=None, yaw=None):
        goal_msg = DroneCommand.Goal()
        goal_msg.command_type = command_type
        if target_pose is not None:
            goal_msg.target_pose = target_pose
        if yaw is not None:
            goal_msg.yaw = yaw
        log_msg = f'Sending command: {command_type}, target_pose: {target_pose} yaw: {yaw}'
        self.get_logger().info(log_msg)
        #self.imgui_logger.info(log_msg)
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
    imgui.set_cursor_pos((1270,820)) #moved 100 down y-axis for fullscreen
    with imgui.font(font):
        imgui.text("Target Pose (x y -z yaw):")
    imgui.set_cursor_pos((1270,885))
    imgui.set_next_item_width(300)
    imgui.set_window_font_scale(2.0) 
    changed, text_field= imgui.input_text("##goto_input", text_buffer, 64)
    if changed:
        text_buffer = text_field
    imgui.set_window_font_scale(1.0)
 
  

    imgui.set_cursor_pos((1590,875))
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
                    node.imgui_logger.info(f"Going to position: x = {x}, y = {y}, z = {z}, yaw = {yaw}")

                except ValueError:
                    x, y, z =map(float, text_buffer.strip().split())
                    node.send_command("goto", [x, y, z], yaw)
                    node.imgui_logger.info(f"Going to position: x = {x}, y = {y}, z = {z}")
            except:
                node.get_logger().warn("Invalid input for goto, please enter x y z yaw values")
                node.imgui_logger.warn("Invalid input for goto, please enter x y z yaw values")

            
                
    imgui.pop_style_color(3)
    imgui.pop_style_var()

def speed_field(node):
    global speed_buffer
    speed_text_field = ""
    imgui.set_cursor_pos((1270, 700))  
    with imgui.font(font):
        imgui.text("Speed (m/s):")
    imgui.set_cursor_pos((1270, 765))
    imgui.set_next_item_width(300)
    imgui.set_window_font_scale(2.0)
    changed, speed_text_field = imgui.input_text("##speed_input", speed_buffer, 64)
    if changed:
        speed_buffer = speed_text_field
    imgui.set_window_font_scale(1.0)
    imgui.set_cursor_pos((1590, 755))
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
        imgui.set_cursor_pos((23, 240)); imgui.text(f"[m]")
    #with imgui.font(font_for_meter):
    #    imgui.set_cursor_pos((240,240)); imgui.text("*measure in meters")
    #    imgui.set_cursor_pos((405,42)); imgui.text("*")

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
    imgui.set_cursor_pos((103, 250)); imgui.text(f" TS: {position_timestamp}")

def XYZVelocity_Text_Field():

        global velocity_x, velocity_y, velocity_z, velocity_timestamp

        draw_list = imgui.get_window_draw_list()
        color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5)
         
        # Move down y axis by 25 (all y values + 25)
        draw_list.add_rect_filled(20, 310, 250, 455, color, rounding=10.0, flags=10)

        with imgui.font(font_small):
            imgui.set_cursor_pos((23, 270)); imgui.text("Velocity:")
            imgui.set_cursor_pos((60, 315)); imgui.text("X   = ")
            imgui.set_cursor_pos((150, 313)); imgui.text(f"{Decimal(velocity_x).quantize(Decimal('0.000'))}")
            imgui.set_cursor_pos((60, 365)); imgui.text("Y   = ")
            imgui.set_cursor_pos((150, 363)); imgui.text(f"{Decimal(velocity_y).quantize(Decimal('0.000'))}")
            imgui.set_cursor_pos((60, 415)); imgui.text("Z   = ")
            imgui.set_cursor_pos((150, 413)); imgui.text(f"{-(Decimal(velocity_z).quantize(Decimal('0.000')))}")
            imgui.set_cursor_pos((23, 460)); imgui.text(f"[m/s]")
        imgui.set_cursor_pos((103, 470)); imgui.text(f" TS: {velocity_timestamp}")
    
def RPY_Text_Field():
    
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5)
    # Move down y axis by 20 (all y values + 20)
    draw_list.add_rect_filled(20, 530, 250, 675, color, rounding=10.0, flags=10)

    with imgui.font(font_small):
        imgui.set_cursor_pos((23, 490)); imgui.text("Orientation:")
        imgui.set_cursor_pos((30, 540)); imgui.text("Roll  = ")
        imgui.set_cursor_pos((150, 538)); imgui.text(f"{Decimal(roll).quantize(Decimal('0.00'))}")
        imgui.set_cursor_pos((30, 590)); imgui.text("Pitch = ")
        imgui.set_cursor_pos((150, 588)); imgui.text(f"{Decimal(pitch).quantize(Decimal('0.00'))}")
        imgui.set_cursor_pos((30, 640)); imgui.text("Yaw   = ")
        imgui.set_cursor_pos((150, 638)); imgui.text(f"{Decimal(yaw_velocity).quantize(Decimal('0.00'))}")
        imgui.set_cursor_pos((23, 680)); imgui.text(f"[r]")

def probe_Field(node):
    global probes
    global probe_classification, probe_numb
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5)
    # Move up 30 on y axis (was 780-925, now 750-895)

    
    draw_list.add_rect_filled(20, 750, 420, 895, color, rounding=10.0, flags=10)


    start_x, start_y = 80, 750  # Line after ID
    end_x, end_y = 80, 895      
    color_line = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 1.0) 
    draw_list.add_line(start_x,start_y, end_x, end_y, color_line, 5.0)

    start_x, start_y = 165, 750  # Line after X
    end_x, end_y = 165, 895      
    color_line = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 1.0) 
    draw_list.add_line(start_x,start_y, end_x, end_y, color_line, 5.0)

    start_x, start_y = 250, 750  # Line after Y
    end_x, end_y = 250, 895      
    color_line = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 1.0) 
    draw_list.add_line(start_x,start_y, end_x, end_y, color_line, 5.0)

    start_x, start_y = 335, 750  # Line after Z
    end_x, end_y = 335, 895      
    color_line = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 1.0) 
    draw_list.add_line(start_x,start_y, end_x, end_y, color_line, 5.0)

    with imgui.font(font_small):
        imgui.set_cursor_pos((23, 710)); imgui.text("Probe info:")
        imgui.set_cursor_pos((30, 755)); imgui.text("ID")
        imgui.set_cursor_pos((115, 755)); imgui.text("X")
        imgui.set_cursor_pos((200, 755)); imgui.text("Y")
        imgui.set_cursor_pos((285, 755)); imgui.text("Z")
        imgui.set_cursor_pos((370, 755)); imgui.text("C")
    try:
        i = 0
        if probe_numb == 1:
            plan_card.probe_cards(0,1,probes,i,node)
        if probe_numb == 2:
            plan_card.probe_cards(0,1,probes,i,node)
            plan_card.probe_cards(20,2,probes,i+3,node)
        if  probe_numb == 3:
            plan_card.probe_cards(0,1,probes,i,node)
            plan_card.probe_cards(20,2,probes,i+3,node)
            plan_card.probe_cards(40,3,probes,i+6,node)
        if probe_numb == 4:
            plan_card.probe_cards(0,1,probes,i,node)
            plan_card.probe_cards(20,2,probes,i+3,node)
            plan_card.probe_cards(40,3,probes,i+6,node)
            plan_card.probe_cards(60,4,probes,i+9,node)
        if probe_numb >= 5:
            plan_card.probe_cards(0,1,probes,i,node)
            plan_card.probe_cards(20,2,probes,i+3,node)
            plan_card.probe_cards(40,3,probes,i+6,node)
            plan_card.probe_cards(60,4,probes,i+9,node)
            plan_card.probe_cards(80,5,probes,i+12,node)
        #        imgui.set_cursor_pos((30, 790)); imgui.text("1")
        #        imgui.set_cursor_pos((100, 790)); imgui.text(f"{Decimal(probes[(i)]).quantize(Decimal('0.00'))}")
        #        imgui.set_cursor_pos((185, 790)); imgui.text(f"{Decimal(probes[(i)+1]).quantize(Decimal('0.00'))}")
        #        imgui.set_cursor_pos((270, 790)); imgui.text(f"{Decimal(probes[(i)+2]).quantize(Decimal('0.00'))}")
        #        imgui.set_cursor_pos((365, 790)); imgui.text(f"wack")
        #i = 3
        #imgui.set_cursor_pos((30, 810)); imgui.text("2")
        #imgui.set_cursor_pos((100, 810)); imgui.text(f"{Decimal(probes[(i)]).quantize(Decimal('0.00'))}")
        #imgui.set_cursor_pos((185, 810)); imgui.text(f"{Decimal(probes[(i)+1]).quantize(Decimal('0.00'))}")
        #imgui.set_cursor_pos((270, 810)); imgui.text(f"{Decimal(probes[(i)+2]).quantize(Decimal('0.00'))}")
        #imgui.set_cursor_pos((365, 810)); imgui.text(f"wack")
#
        #i = 6
        #imgui.set_cursor_pos((30, 830)); imgui.text("3")
        #imgui.set_cursor_pos((100, 830)); imgui.text(f"{Decimal(probes[(i)]).quantize(Decimal('0.00'))}")
        #imgui.set_cursor_pos((185, 830)); imgui.text(f"{Decimal(probes[(i)+1]).quantize(Decimal('0.00'))}")
        #imgui.set_cursor_pos((270, 830)); imgui.text(f"{Decimal(probes[(i)+2]).quantize(Decimal('0.00'))}")
        #imgui.set_cursor_pos((365, 830)); imgui.text(f"wack")
#
        #i = 9
        #imgui.set_cursor_pos((30, 850)); imgui.text("4")
        #imgui.set_cursor_pos((100, 850)); imgui.text(f"{Decimal(probes[(i)]).quantize(Decimal('0.00'))}")
        #imgui.set_cursor_pos((185, 850)); imgui.text(f"{Decimal(probes[(i)+1]).quantize(Decimal('0.00'))}")
        #imgui.set_cursor_pos((270, 850)); imgui.text(f"{Decimal(probes[(i)+2]).quantize(Decimal('0.00'))}")
        #imgui.set_cursor_pos((365, 850)); imgui.text(f"wack")
#
        #i = 12
        #imgui.set_cursor_pos((30, 870)); imgui.text("5")
        #imgui.set_cursor_pos((100, 870)); imgui.text(f"{Decimal(probes[(i)]).quantize(Decimal('0.00'))}")
        #imgui.set_cursor_pos((185, 870)); imgui.text(f"{Decimal(probes[(i)+1]).quantize(Decimal('0.00'))}")
        #imgui.set_cursor_pos((270, 870)); imgui.text(f"{Decimal(probes[(i)+2]).quantize(Decimal('0.00'))}")
        #imgui.set_cursor_pos((365, 870)); imgui.text(f"wack")
    except:
        pass

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

    imgui.set_cursor_pos((1720, 175)); imgui.text(f" TS:  {battery_state_timestamp}") 

def map_value(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def drone_visualization():
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5) 
                                                                #flags is for rounding different corners
    draw_list.add_rect(450,140, 1260,620,color,rounding =10.0, flags=15,thickness=6) #earlier 450 150 1100 560 Scale diff = x1.145 y1.101 (only one)
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 0.9) 
                                                                
    draw_list.add_rect_filled(453,143, 1256,624,color,rounding =10.0, flags=15) #e 453 153 1097 567

def grid():
    global position_x, position_y
    dot_position_x = map_value(position_x, 10, -10, 467, 1228) #e 1107
    dot_position_y = map_value(position_y, 10, -10, 158, 599) #e 155 561
    drawlist = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.2, 0.6, 0.9, 0.8)  # Light blue color
    for x in range(467, 1256, 38):
        drawlist.add_line(x, 145, x, 620, color, 1.5) #e 145 563
    for y in range(157, 618, 22): #e 157 560 20
        drawlist.add_line(455, y, 1254, y, color, 1.5) #e 455 1095

    imgui.push_style_color(imgui.COLOR_TEXT, 0.0, 0.0, 0.0, 1.0)
    imgui.set_cursor_pos((462, 610)); imgui.text("10   9    8     7    6     5     4    3     2    1    0    -1    -2   -3   -4   -5    -6    -7    -8    -9   -10") #e 462 570
    j = 10
    for i in range(40, 482, 22): #e 50 470
            imgui.set_cursor_pos((1235, 105+i)); imgui.text(f"{j}")
            j -= 1
    imgui.pop_style_color()
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(1.0, 0.0, 0.0, 1.0)

    if 449 < dot_position_x < 1260 and 155 < dot_position_y < 620: 
            
            draw_list.add_circle_filled(dot_position_x, dot_position_y, 4, color)  # Red dot at the center Center of the grid 778, 358
    elif 449 > dot_position_x:
        if 155 < dot_position_y < 620:
            draw_list.add_triangle_filled(449, dot_position_y-10, 449, dot_position_y+10, 464, dot_position_y, color)
    elif dot_position_x > 1260:
        if 155 < dot_position_y < 620:
            draw_list.add_triangle_filled(1260, dot_position_y-10, 1260, dot_position_y+10, 1245, dot_position_y, color)
    elif 155 > dot_position_y:
        if 449 < dot_position_x < 1260:
            draw_list.add_triangle_filled(dot_position_x -10, 155, dot_position_x+10, 155, dot_position_x, 180, color)
    elif dot_position_y > 620:
        if 449 < dot_position_x < 1260:
            draw_list.add_triangle_filled(dot_position_x -10, 620, dot_position_x+10, 620, dot_position_x, 595, color)
        #draw_list.add_triangle_filled((velocity_x* 10)+830, 450, (velocity_x* 10)+830, 475, (velocity_x* 10)+855, 462.5, color)
    

    # max 449 1107 x
    # max 155 561 y
              
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
    imgui.set_cursor_pos((684, 635))
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
    imgui.set_cursor_pos((1562, 200))
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
                    #GuiConsoleLogger("Manual mode activated.")
                    node.imgui_logger.info("Manual mode activated")
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
    
    imgui.set_cursor_pos((1562, 710))
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
    global current_item
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        #print("No joystick connected.")
        current_item = 0
        return
    
    node.joystick = pygame.joystick.Joystick(0)
    node.joystick.init()
    if node.joystick.get_name() == "Sony Interactive Entertainment Wireless Controller":
        current_item = 1
        #print(f"Initialized joystick: {node.joystick.get_name()}")
        return
    if node.joystick.get_name() == "OpenTX RM TX16S Joystick":
        current_item = 2
        #print(f"Initialized joystick: {node.joystick.get_name()}")
        return
    if not node.joystick.get_init():
        #print("Joystick initialization failed.")
        return
    
  

    clock = pygame.time.Clock()
    DEAD_ZONE = 0.05
    prev_axis_state = None
    match current_item:
        case 1:
            try:
                while rclpy.ok():
                    pygame.event.pump()  # Process internal queue
                    #while arming_state == 7:
                    #print(f"Flight mode: {flight_mode}")
                    left_trigger = ((node.joystick.get_axis(2) + 1.0)/2.0)
                    right_trigger = (node.joystick.get_axis(5) + 1.0)/2.0
                    yaw_value = right_trigger - left_trigger
                    roll_m = node.joystick.get_axis(0) if abs(node.joystick.get_axis(0)) > DEAD_ZONE else 0.0
                    pitch_m = -node.joystick.get_axis(1) if abs(node.joystick.get_axis(1)) > DEAD_ZONE else 0.0
                    yaw_velocity_m = yaw_value if abs(yaw_value) > DEAD_ZONE else 0.0
                    thrust = -node.joystick.get_axis(4) if abs(node.joystick.get_axis(4)) > DEAD_ZONE else 0.0
                    current_axis_state = int(abs(node.joystick.get_axis(4)))


                    #if prev_axis_state is None or current_axis_state != prev_axis_state:
                    #    if current_axis_state == 1:
                    #        node.send_command("disarm")
                    #        #print("Arming state changed to: Disarmed (0)")  # Debug
                    #    elif current_axis_state == 0:
                    #        node.send_command("arm")
                    #        #print("Arming state changed to: Armed (1)")  # Debug
                    #    prev_axis_state = current_axis_state
                    ##arming_state = -int(abs(node.joystick.get_axis(4)))
                    ##print(f"Arming state: {arming_state}")
                    ##print(int(abs(node.joystick.get_axis(4))))
                #
                    #if( node.joystick.get_button(3) == 1):
                    #    node.send_command("estop")
                    #    drone_kill = True

                    node.send_manual_control(roll_m, pitch_m, yaw_velocity_m, thrust)
                    clock.tick(20)  # 20 Hz update rate
            except KeyboardInterrupt:
                pass
            finally:
                pygame.quit()
        case 2:
            try:
                while rclpy.ok():
                    pygame.event.pump()  # Process internal queue
                    #while arming_state == 7:
                    #print(f"Flight mode: {flight_mode}")
                    #left_trigger = ((node.joystick.get_axis(2) + 1.0)/2.0)
                    #right_trigger = (node.joystick.get_axis(5) + 1.0)/2.0
                    #yaw_value = right_trigger - left_trigger
                    roll_m = node.joystick.get_axis(0) if abs(node.joystick.get_axis(0)) > DEAD_ZONE else 0.0
                    pitch_m = -node.joystick.get_axis(1) if abs(node.joystick.get_axis(1)) > DEAD_ZONE else 0.0
                    yaw_velocity_m = node.joystick.get_axis(2) if abs(node.joystick.get_axis(2)) > DEAD_ZONE else 0.0
                    thrust = -node.joystick.get_axis(1) if abs(node.joystick.get_axis(1)) > DEAD_ZONE else 0.0
                    
                    current_axis_state = int(abs(node.joystick.get_axis(4)))


                    if prev_axis_state is None or current_axis_state != prev_axis_state:
                        if current_axis_state == 1:
                            node.send_command("disarm")
                            #print("Arming state changed to: Disarmed (0)")  # Debug
                        elif current_axis_state == 0:
                            node.send_command("arm")
                            #print("Arming state changed to: Armed (1)")  # Debug
                        prev_axis_state = current_axis_state
                    arming_state = -int(abs(node.joystick.get_axis(4)))
                    #print(f"Arming state: {arming_state}")
                    #print(int(abs(node.joystick.get_axis(4))))
                
                    if( node.joystick.get_button(3) == 1):
                        node.send_command("estop")
                        drone_kill = True

                    node.send_manual_control(roll_m, pitch_m, yaw_velocity_m, thrust)
                    clock.tick(20)  # 20 Hz update rate
            except KeyboardInterrupt:
                pass
            finally:
                pygame.quit()

def GuiConsoleLogger(node):
    global font_small
    
    # Position the logger window in the bottom right area
    imgui.set_cursor_pos((450, 690))
    
    # Create a child window for the logger
    imgui.begin_child("LoggerWindow", 810, 240, border=True)
    
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
    actuator_speeds_slider_bar1 = map_value(actuator_speeds[0], 0, 1, 106, 54)
    actuator_speeds_slider_bar2 = map_value(actuator_speeds[1], 0, 1, 106, 54)
    actuator_speeds_slider_bar3 = map_value(actuator_speeds[2], 0, 1, 106, 54)
    actuator_speeds_slider_bar4 = map_value(actuator_speeds[3], 0, 1, 106, 54)
    graphs.motor_speed_graph(1300, 50, 1360, 110) 
    graphs.motor_speed_graph(1380, 50, 1440, 110)
    graphs.motor_speed_graph(1460, 50, 1520, 110)
    graphs.motor_speed_graph(1540, 50, 1600, 110) 
    standardcolor = imgui.get_color_u32_rgba(0.7, 0.9, 1.0, 1.0)
    color1 = standardcolor
    color2 = standardcolor
    color3 = standardcolor
    color4 = standardcolor
    if actuator_speeds[0] > 0.9:
        color1 = imgui.get_color_u32_rgba(1.0, 0.0, 0.0, 1.0)
    if actuator_speeds[1] > 0.9:
        color2 = imgui.get_color_u32_rgba(1.0, 0.0, 0.0, 1.0)
    if actuator_speeds[2] > 0.9:
        color3 = imgui.get_color_u32_rgba(1.0, 0.0, 0.0, 1.0)
    if actuator_speeds[3] > 0.9:
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

def load_image_to_texture(image_path):
    try:
        # Load image using Pillow
        img = Image.open(image_path).convert("RGBA")
        width, height = img.size
        pixels = img.tobytes()  # Get RGBA pixel data

        # Generate OpenGL texture
        texture_id = gl.glGenTextures(1)
        gl.glBindTexture(gl.GL_TEXTURE_2D, texture_id)
        gl.glTexImage2D(
            gl.GL_TEXTURE_2D, 0, gl.GL_RGBA, width, height, 0,
            gl.GL_RGBA, gl.GL_UNSIGNED_BYTE, pixels
        )
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, gl.GL_CLAMP_TO_EDGE)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, gl.GL_CLAMP_TO_EDGE)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)
        gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
        
        return texture_id, width, height
    except Exception as e:
        print(f"Error loading image: {e}")
        return None, 0, 0

def mouse_placement():
    global mouse_x_buffer, mouse_y_buffer
    mouse_x, mouse_y = imgui.get_mouse_pos()
    if 449 < mouse_x < 1260 and 155 < mouse_y < 620:  
        if imgui.is_mouse_clicked(imgui.MOUSE_BUTTON_LEFT):
            mouse_x_buffer = mouse_x
            mouse_y_buffer = mouse_y
            print(f"{mouse_x} and {mouse_y} ")

def send_map_pos(node):
    global mouse_x_buffer, mouse_y_buffer, yaw, position_z, target_position_z
    global effect1
    buffer_z = 0
   
    mouse_placement()
    mouse_x, mouse_y = imgui.get_mouse_pos()
    if 449 < mouse_x < 1260 and 155 < mouse_y < 620:  
        if imgui.is_mouse_clicked(imgui.MOUSE_BUTTON_LEFT):
            mouse_x_buffer = mouse_x
            mouse_y_buffer = mouse_y
            picked_x = map_value(mouse_x_buffer, 468, 1228, 10, -10) #e 
            picked_y = map_value(mouse_y_buffer, 159, 598, 10, -10)
            if imgui.is_mouse_clicked(imgui.MOUSE_BUTTON_LEFT):
                try:
                    x, y , z = float(picked_x), float(picked_y), position_z
                    buffer_z = target_position_z
                    node.send_command("goto", [x, y, buffer_z], yaw)
                    node.imgui_logger.info(f"Going to position: x = {Decimal(x).quantize(Decimal('0.000'))}, y = {Decimal(y).quantize(Decimal('0.000'))}, z = {Decimal(buffer_z).quantize(Decimal('0.000'))}")
                    effect1 = True
                except ValueError:
                    pass
    draw_list = imgui.get_window_draw_list()
    draw_list.add_circle_filled(mouse_x_buffer, mouse_y_buffer, 4, imgui.get_color_u32_rgba(0.0, 1.0, 0.0, 1.0))
    effect_class.circle_effect_green(imgui.get_color_u32_rgba(0.0, 1.0, 0.0, 1.0),mouse_x_buffer,mouse_y_buffer)
    
def route_planner(node):
    global change_y_1, change_y_2, permant_y, card_y_buffer
    global flight_plan, text_buffer_plan, flight_plan_numb, flight_plan_coord, execute_route_numb
    global effect2, current_step, temp_y_arrow, start_y_arrow 
    text_field = ""
    mouse_x, mouse_y = imgui.get_mouse_pos()
    imgui.set_cursor_pos((1690, 260))
    imgui.set_window_font_scale(2.0)
    imgui.set_next_item_width(180)
    changed, text_field= imgui.input_text("##goto_input_plan", text_buffer_plan, 20)
    if changed:
        text_buffer_plan = text_field
    imgui.set_window_font_scale(1.0) 
    color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5) 
    draw_list = imgui.get_window_draw_list()
    draw_list.add_rect_filled(1285,140, 1556, 624,color,rounding =10.0, flags=15) #e 1505
    draw_list.add_line(1285,180,1556, 180,imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 0.9), 5)

    with imgui.font(font_small):
        imgui.set_cursor_pos((1338,145)); imgui.text("Flight Plan")


    if GUIButton.button_Plan(1562, 316, font_small, "Takeoff##plan1"):
        
        for i, val in enumerate(flight_plan):
            if val == 0:
                flight_plan[i] = 1
                flight_plan_numb += 1
                break
    if GUIButton.button_Plan(1562, 376, font_small, "Land##plan2"):
        
        for i, val in enumerate(flight_plan):
            if val == 0:
                flight_plan[i] = 2
                flight_plan_numb += 1
                break
    if GUIButton.button_Plan(1562, 256, font_small, "Goto##plan3"):
        
        for i, val in enumerate(flight_plan):
            if val == 0:
                flight_plan[i] = 3
                try:
                    flight_plan_coord[flight_plan_numb] = [text_buffer_plan]
                    flight_plan_numb += 1
                    
                    text_buffer_plan = ""
                    text_field = ""
                except ValueError:
                     node.imgui_logger.warn("Invalid input for goto, please enter x y z yaw values")
                
                break
    if GUIButton.button_Plan(1436,635,font_small, "Clear##plan4"):
        for i in range(7):
            flight_plan[i] = 0
            flight_plan_numb = 0
            flight_plan_coord[i][0] = ""
            current_step = 0
            temp_y_arrow = 0
            start_y_arrow = 0
            execute_route_numb = 0
   
    
    
    #if imgui.is_mouse_down(imgui.MOUSE_BUTTON_LEFT):
    #    if 1297 < mouse_x < 1565 and 190 < mouse_y < 624:  
    #        card_y_buffer = mouse_y -25
    #        #change_y_1 = mouse_y -40
    #        #change_y_2 = mouse_y + 40
    #        
    #        permant_y = card_y_buffer
    #
    #else:
    #    permant_y = card_y_buffer
            
    #plan_card.goto_card(1287, 400, node)

def is_T_close(target_pos_x, target_pos_y):
    #global position_x, position_y, position_z
    #pos_vector_length = math.sqrt(math.pow(position_x, 2)+ math.pow(position_y,2))
    #t_vector_length = math.sqrt(math.pow(target_pos_x, 2)+ math.pow(target_pos_y,2))
    #e = t_vector_length - pos_vector_length
    #if e <= 0.5:
    #    return True
    #else:
    #    return False
    global position_x, position_y
    dx = target_pos_x - position_x
    dy = target_pos_y - position_y
    distance = math.sqrt(dx * dx + dy * dy)
    return distance <= 0.5
    
def execute_route(node):
    global flight_plan, flight_plan_coord
    global start_time, plan_duration, begin_execute
    global execute_route_numb, effect3
    global position_x, position_y, position_z
    global current_step, step_start_time, command_sent, waiting_for_completion
    #print(execute_route_numb)
    if GUIButton.button_Plan(1285, 635, font_small, "Excecute##plan4"):
        begin_execute = True
        current_step = 0
        step_start_time = 0
        command_sent = False
        waiting_for_completion = False
        

    # If we’re not executing or plan is over
    if not begin_execute or current_step >= len(flight_plan) or flight_plan[current_step] == 0:
        begin_execute = False
        current_step = 0
        step_start_time = 0
        command_sent = False
        waiting_for_completion = False
        return

    current_time = time.time()
    if step_start_time == 0:
        step_start_time = current_time
        command_sent = False
        waiting_for_completion = False

    elapsed = current_time - step_start_time

    step_finished = False  # track if this step finishes in this frame
    #print(current_step)
    match flight_plan[current_step]:
        case 1:  # Arm & takeoff
            if not command_sent:
                node.send_command("arm")
                command_sent = True

            if elapsed >= plan_duration - 3 and not waiting_for_completion:
                node.send_command("takeoff", [-1.0])
                waiting_for_completion = True

            if waiting_for_completion and elapsed >= plan_duration + 5:
                effect3 = True
                execute_route_numb += 1
                step_finished = True

        case 2:  # Land
            
            if not command_sent:
                node.send_command("land", [-1.0])
                command_sent = True

            if elapsed >= 3.0:
                effect3 = True
                step_finished = True

        case 3:  # Goto
            
            if not command_sent:
                try:
                    if execute_route_numb < len(flight_plan_coord) and flight_plan_coord[execute_route_numb]:
                        x, y, z, yaw = map(float, flight_plan_coord[execute_route_numb][0].strip().split())
                        node.send_command("goto", [x, y, z], yaw)
                        node.imgui_logger.info(f"Going to position: x = {x}, y = {y}, z = {z}, yaw = {yaw}")
                        command_sent = True
                        waiting_for_completion = True
                    else:
                        step_finished = True
                except ValueError:
                    node.get_logger().warn("Invalid goto input")
                    node.imgui_logger.warn("Invalid goto input")
                    step_finished = True

            if waiting_for_completion and elapsed > 0.5:
                try:
                    if execute_route_numb < len(flight_plan_coord) and flight_plan_coord[execute_route_numb]:
                        x, y, z, yaw = map(float, flight_plan_coord[execute_route_numb][0].strip().split())
                        if is_T_close(x, y):
                            effect3 = True

                            step_finished = True
                            execute_route_numb += 1
                        #elif elapsed > 50.0:
                        #    effect3 = True
                        #    
                        #    step_finished = True
                        #    waiting_for_completion = False
                except (ValueError, IndexError):
                    step_finished = True
                    execute_route_numb += 1
                    #waiting_for_completion = False

    # If the step finished, move to next one, but don’t process it this frame
    if step_finished:
        waiting_for_completion = False
        command_sent = False
        step_start_time = -1  
        current_step += 1  
        return
    print(execute_route_numb)
    
        
class effect_class:
    def circle_effect_green(color, pos_x, pos_y):
        global effect1,effect_begin1, duration, start_time
        t = 0
        draw_list = imgui.get_window_draw_list()
        if effect1:
            start_time = time.time()
            effect_begin1 = True
            effect1 = False
        if effect_begin1 and start_time is not None:
            elapsed = time.time() - start_time
            t = min(elapsed / duration, 1.0)  # Clamp to [0,1]
            ## Interpolate radius from 30 to 12
            current_radius = 60 - (60 - 12) * (t)
            ## Draw shrinking circle
            draw_list.add_circle(pos_x, pos_y,current_radius,color,thickness=2.0)
         # Stop effect once finished
        if t >= 1.0:
            effect_begin1 = False
            start_time = None
        else:
            draw_list.add_circle(pos_x, pos_y, 12,color, thickness=2.0 ) 
    def circle_effect_orange(color, pos_x, pos_y, numb):
        global effect2, effect_begin2, duration, start_time
        global orange_effect_going
        
        t = 0
        draw_list = imgui.get_window_draw_list()
        if effect2:
            start_time = time.time()
            effect_begin2 = True
            effect2 = False
        if effect_begin2 and start_time is not None:
            elapsed = time.time() - start_time
            t = min(elapsed / duration, 1.0)  # Clamp to [0,1]
            ## Interpolate radius from 30 to 12
            current_radius = 60 - (60 - 12) * (t)
            ## Draw shrinking circle
            draw_list.add_circle(pos_x, pos_y,current_radius,color,thickness=2.0)
         # Stop effect once finished
        if t >= 1.0:
            effect_begin2 = False
            start_time = None
            orange_effect_going[numb] = 2   
    def arrow_effect(numb):
        global effect_begin3, effect3
        global duration, start_time, temp_y_arrow, start_y_arrow
        global flight_plan
        draw_list = imgui.get_window_draw_list()
        color = imgui.get_color_u32_rgba(0.0, 1.0, 0.0, 0.9)
        if flight_plan[numb] != 0:
            t = 0
            draw_list = imgui.get_window_draw_list()
            if effect3:
                start_time = time.time()
                effect_begin3 = True
                effect3 = False
                start_y_arrow = temp_y_arrow 


           
            if effect_begin3 and start_time is not None:
                elapsed = time.time() - start_time
                t = min(elapsed / duration, 1.0)
                temp_y_arrow = start_y_arrow + (52 * t)

                #draw_list.add_triangle_filled(1266, 201+(temp_y), 1266, 231+(temp_y), 1282, 216+(temp_y), color)
            #draw_list.add_triangle_filled(830, 450, 830, 475, 855, 462.5, color) #move y axis 52
            if t >= 1.0:
                effect_begin3 = False
                start_time = None
                temp_y_arrow = start_y_arrow + 52

            draw_list.add_triangle_filled(1266, 201+(temp_y_arrow ), 1266, 231+(temp_y_arrow ), 1282, 216+(temp_y_arrow ), color)
                #draw_list.add_triangle_filled(1266, 201+(temp_y), 1266, 231+(temp_y), 1282, 216+(temp_y), color)
        else:
            draw_list.add_triangle_filled(1266, 201+(temp_y_arrow), 1266, 231+(temp_y_arrow), 1282, 216+(temp_y_arrow), color)

class plan_card:
    def goto_card(card_pos_x, card_pos_y, node, type,text, numb):
        global flight_plan_numb, flight_plan_coord, execute_route_numb
        global effect2, effect_begin2, orange_effect_going
        match type:
            case 0:
                color1 = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 0.9) 
                color2 = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5) 
                draw_list = imgui.get_window_draw_list()
                draw_list.add_rect_filled(card_pos_x,card_pos_y, card_pos_x + 265, card_pos_y+50,color1,rounding =10.0, flags=15)
                draw_list.add_rect_filled(card_pos_x+2,card_pos_y+2, card_pos_x + 263, card_pos_y+48,color2,rounding =10.0, flags=15)
            case 1:
                color = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 0.9) 
                draw_list = imgui.get_window_draw_list()
                draw_list.add_rect_filled(card_pos_x,card_pos_y, card_pos_x + 265, card_pos_y+50,color,rounding =10.0, flags=15)
                imgui.push_style_color(imgui.COLOR_TEXT, 0.0, 0.0, 0.0, 1.0)
                imgui.set_cursor_pos((card_pos_x+35,card_pos_y+20)); imgui.text(f"Takeoff")

                with imgui.font(font):

                    imgui.set_cursor_pos((card_pos_x+230,card_pos_y-6)); imgui.text("-")
                    imgui.set_cursor_pos((card_pos_x+230,card_pos_y+4)); imgui.text("-")
                    imgui.set_cursor_pos((card_pos_x+230,card_pos_y+14)); imgui.text("-")

                imgui.pop_style_color()
            case 2:
                color = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 0.9) 
                draw_list = imgui.get_window_draw_list()
                draw_list.add_rect_filled(card_pos_x,card_pos_y, card_pos_x + 265, card_pos_y+50,color,rounding =10.0, flags=15)
                imgui.push_style_color(imgui.COLOR_TEXT, 0.0, 0.0, 0.0, 1.0)
                imgui.set_cursor_pos((card_pos_x+35,card_pos_y+20)); imgui.text(f"Land")

                with imgui.font(font):

                    imgui.set_cursor_pos((card_pos_x+230,card_pos_y-6)); imgui.text("-")
                    imgui.set_cursor_pos((card_pos_x+230,card_pos_y+4)); imgui.text("-")
                    imgui.set_cursor_pos((card_pos_x+230,card_pos_y+14)); imgui.text("-")

                imgui.pop_style_color()
            case 3:
                text = str(text)
                clean_text = re.sub(r"[^\d.\s\-+]", " ", text)
                clean_text = " ".join(clean_text.split())

                # Draw card background
                color = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 0.9) 
                draw_list = imgui.get_window_draw_list()
                draw_list.add_rect_filled(card_pos_x, card_pos_y, card_pos_x + 265, card_pos_y + 50, color, rounding=10.0, flags=15)

                imgui.push_style_color(imgui.COLOR_TEXT, 0.0, 0.0, 0.0, 1.0)
                imgui.set_cursor_pos((card_pos_x + 35, card_pos_y + 20))
                imgui.text(f"Going to: {clean_text}")

                with imgui.font(font):
                    imgui.set_cursor_pos((card_pos_x + 230, card_pos_y - 6)); imgui.text("-")
                    imgui.set_cursor_pos((card_pos_x + 230, card_pos_y + 4)); imgui.text("-")
                    imgui.set_cursor_pos((card_pos_x + 230, card_pos_y + 14)); imgui.text("-")
                imgui.pop_style_color()
                try:
                    temp_x, temp_y, temp_z, temp_yaw = map(float, flight_plan_coord[numb][0].strip().split())
                    circle_x = map_value(temp_x, 10, -10, 467, 1228)
                    circle_y = map_value(temp_y, 10, -10, 158, 599)
                    if orange_effect_going[numb] == 0 and not effect_begin2: 
                    
                        effect2 = True
                        orange_effect_going[numb] = 1
                        return  # Skip rest of logic for this frame

                    draw_list.add_circle(circle_x, circle_y, 12,imgui.get_color_u32_rgba(0.9, 0.8, 0.0, 1.0), thickness=2.0 ) 
                    draw_list.add_circle_filled(circle_x, circle_y, 4, imgui.get_color_u32_rgba(0.9, 0.8, 0.0, 1.0))
                    if orange_effect_going[numb] == 1:

                        effect_class.circle_effect_orange(imgui.get_color_u32_rgba(0.9, 0.8, 0.0, 1.0),circle_x,circle_y, numb)
                        return  # Wait for effect to finish
                    draw_list.add_circle(circle_x, circle_y, 12,imgui.get_color_u32_rgba(0.9, 0.8, 0.0, 1.0), thickness=2.0 ) 
                    draw_list.add_circle_filled(circle_x, circle_y, 4, imgui.get_color_u32_rgba(0.9, 0.8, 0.0, 1.0))
                except:
                    flight_plan_coord[numb][0] = ""


                
                # --- Effect setup only once ---
    def probe_cards(y_add, id, probes, i, node):
        imgui.set_cursor_pos((30, 790+y_add)); imgui.text(f"{id}")
        imgui.set_cursor_pos((100, 790+y_add)); imgui.text(f"{Decimal(probes[(i)]).quantize(Decimal('0.00'))}")
        imgui.set_cursor_pos((185, 790+y_add)); imgui.text(f"{Decimal(probes[(i)+1]).quantize(Decimal('0.00'))}")
        imgui.set_cursor_pos((270, 790+y_add)); imgui.text(f"{Decimal(probes[(i)+2]).quantize(Decimal('0.00'))}")
        imgui.set_cursor_pos((365, 790+y_add)); imgui.text(f"wack")
               
def drone_image(image_path, texture_id, img_width, img_height):
    #Todo move to function
    global position_x, position_y
    dot_position_x = map_value(position_x, 10, -10, 467, 1228) #e 1107
    dot_position_y = map_value(position_y, 10, -10, 158, 599)
    if 455 < dot_position_x < 1250 and 165 < dot_position_y < 610:  
        if texture_id:

            img = Image.open(image_path).convert("RGBA")

            # Apply a dead zone to yaw_velocity to prevent sudden jumps between 6.28 and 0
            if abs(yaw_velocity) < 0.05 or abs(yaw_velocity - 2 * math.pi) < 0.05:
                angle = 0
            else:
                angle = yaw_velocity * 57.4358  # Convert 0-2π to degrees
            rotated_img = img.rotate(angle, expand=False, center=(img.width/2, img.height/2))
            pixels = rotated_img.tobytes()
            gl.glBindTexture(gl.GL_TEXTURE_2D, texture_id)
            gl.glTexImage2D(
                gl.GL_TEXTURE_2D, 0, gl.GL_RGBA, img_width, img_height, 0,
                gl.GL_RGBA, gl.GL_UNSIGNED_BYTE, pixels
            )
            gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
            imgui.set_cursor_pos((dot_position_x-161.5, dot_position_y-107))  # Position the image
            imgui.image(texture_id, 290, 230)
        else:
            imgui.text("Failed to load droneimage.png")               
def main(args=None):
    rclpy.init()
    global font, font_large, font_small, font_for_meter
    global position_x, position_y, yaw_velocity
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
    # Try to find the font in the ROS package share directory
    
    try:
        font_dir = os.path.join(
            ament_index_python.packages.get_package_share_directory("gcs"),
            "fonts", "source-code-pro"
        
        )
        image_dir = os.path.join(
            ament_index_python.packages.get_package_share_directory("gcs"),
            "images"
        )
        image_path = os.path.join(image_dir, "droneImage.png")
        font_path = os.path.join(font_dir, "SourceCodePro-Black.otf")
    except Exception as e:
        print(f"Could not find gcs package share directory: {e}")
        glfw.terminate()
        return
    #font_path = os.path.normpath(font_path)
    if not os.path.isfile(font_path):
        print(f"Font file not found: {font_path}")
        glfw.terminate()
        return
    font = io.fonts.add_font_from_file_ttf(font_path, 40)
    font_large = io.fonts.add_font_from_file_ttf(font_path, 50)
    font_small = io.fonts.add_font_from_file_ttf(font_path, 30)
    font_for_meter = io.fonts.add_font_from_file_ttf(font_path, 20)
    impl.refresh_font_texture()
    

    #texture_id = glGenTextures(1)
    #glBindTexture(GL_TEXTURE_2D, texture_id)
    #
    ## Set texture parameters
    #glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
    #glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
    #glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    #glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)

    if image_path and os.path.exists(image_path):
        texture_id, img_width, img_height = load_image_to_texture(image_path)
    else:
        print(f"Image file not found at: {image_path}")
        texture_id, img_width, img_height = None, 0, 0
    # Main loop
    while not glfw.window_should_close(window):
        glfw.poll_events()
        impl.process_inputs()
        clear_color = (0.0, 0.0, 0.12, 0.0)  # Background color
        gl.glClearColor(*clear_color)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        imgui.new_frame()
      
        # Create UI without windowed mode
        imgui.set_next_window_position(0, 0)
        imgui.set_next_window_size(screen_width, screen_height)
        imgui.begin("wtf", flags=imgui.WINDOW_NO_TITLE_BAR | imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE | imgui.WINDOW_NO_BACKGROUND)
        imgui.set_cursor_pos((20,10))
        with imgui.font(font_small):
            imgui.text("THYRA GCS")
        #Creating a line for seperation
        draw_list = imgui.get_window_draw_list()
        start_x, start_y = 430, 0  # Starting point of the line (x, y)
        end_x, end_y = 430, 2000      # Ending point of the line (x, y)
        color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5) 
        draw_list.add_line(start_x,start_y, end_x, end_y, color, 5.0)
        #Running different widgets
        
        drone_visualization()
        grid()
        Arm_Button(node)
        Kill_command(node)
        Goto_field(node)
        speed_field(node)
        Dropdown_Menu()
        XYZ_Text_Field(msg=drone_data)
        RPY_Text_Field()
        probe_Field(node)
        XYZVelocity_Text_Field()
        batteryGraph()
        motor_speed()
        #Arrows()   
        #takeoff_button(node)
        #land_button(node)
        return_to_home_button(node)
        #drone_image(image_path,texture_id,img_width,img_height)
        GuiConsoleLogger(node)
        GUIButton.button1(876, 635, font_small, "Land", "land", node)
        GUIButton.button2( 492, 635, font_small, "Takeoff", "takeoff", node, -1.0)
        GUIButton.button1(1068, 635, font_small, "Set Origin", "set_origin", node)
        
        manual(node) #to be continued
        send_map_pos(node)
        route_planner(node)
        execute_route(node)
        plan_card.goto_card(1288, 190, node, flight_plan[0], flight_plan_coord[0], 0)
        plan_card.goto_card(1288, 242, node, flight_plan[1], flight_plan_coord[1], 1)
        plan_card.goto_card(1288, 294, node, flight_plan[2], flight_plan_coord[2], 2)
        plan_card.goto_card(1288, 346, node, flight_plan[3], flight_plan_coord[3], 3)
        plan_card.goto_card(1288, 398, node, flight_plan[4], flight_plan_coord[4], 4)
        plan_card.goto_card(1288, 450, node, flight_plan[5], flight_plan_coord[5], 5)
        plan_card.goto_card(1288, 502, node, flight_plan[6], flight_plan_coord[6], 6)
        effect_class.arrow_effect(execute_route_numb)
     

        #Todo move to function
    
        dot_position_x = map_value(position_x, 10, -10, 467, 1228) #e 1107
        dot_position_y = map_value(position_y, 10, -10, 158, 599)
        if 455 < dot_position_x < 1250 and 165 < dot_position_y < 610:  
            if texture_id:

                img = Image.open(image_path).convert("RGBA")

                # Apply a dead zone to yaw_velocity to prevent sudden jumps between 6.28 and 0
                if abs(yaw_velocity) < 0.05 or abs(yaw_velocity - 2 * math.pi) < 0.05:
                    angle = 0
                else:
                    angle = yaw_velocity * 57.4358  # Convert 0-2π to degrees
                rotated_img = img.rotate(angle, expand=False, center=(img.width/2, img.height/2))
                pixels = rotated_img.tobytes()
                gl.glBindTexture(gl.GL_TEXTURE_2D, texture_id)
                gl.glTexImage2D(
                    gl.GL_TEXTURE_2D, 0, gl.GL_RGBA, img_width, img_height, 0,
                    gl.GL_RGBA, gl.GL_UNSIGNED_BYTE, pixels
                )
                gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
                imgui.set_cursor_pos((dot_position_x-50, dot_position_y-50))  # Position the image
                imgui.image(texture_id, 100, 100)
            else:
                imgui.text("Failed to load droneimage.png")   


        imgui.end()

        
        # Render
        
        imgui.render()
        impl.render(imgui.get_draw_data())
        glfw.swap_buffers(window)
    
    # Cleanup
    if texture_id:
        gl.glDeleteTextures([texture_id])
    impl.shutdown()
    glfw.terminate()
    node.destroy_node()

if __name__ == "__main__":
    main()
    


    