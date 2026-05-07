#!/usr/bin/env python3

import glfw
import warnings
from glfw import GLFWError

from px4_msgs.msg._distance_sensor import DistanceSensor
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
from asr_comms.msg import TelemetryPosition, TelemetryAttitude, TelemetryBattery, TelemetryGPS, TelemetryStatus
from asr_comms.msg import GcsHeartbeat
from asr_comms.msg import ServoCommand
from asr_comms.msg import ProbeGlobalLocations
from asr_comms.action import DroneCommand
from dataclasses import dataclass
from decimal import Decimal
import os
import ament_index_python.packages

import rclpy
from rclpy.node import Node
from asr_comms.msg import ManualControlInput
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

from px4_msgs.msg import VehicleLocalPosition

#filename = "probe_data.txt"

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
    def button_save(x,y,font,label,filename,probes, transformed_probes):
        
        os.makedirs(os.path.dirname(filename), exist_ok=True)

        imgui.set_window_font_scale(0.75)
        imgui.set_cursor_pos((x, y))
        imgui.push_style_var(imgui.STYLE_FRAME_ROUNDING, 12.0)
        imgui.push_style_color(imgui.COLOR_BUTTON, *(0.0, 0.5, 0.0))
        imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, *(0.0, 0.8, 0.0))
        imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, *(0.0, 0.2, 0.0)) 
        
   

        with imgui.font(font):
            if imgui.button(label, width=120, height=40):
                try:
                    with open(filename, "w") as f:
                    
                        f.write(f"Data recorded on: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")

                        # Write each probe's data
                        for i, probe in enumerate(probes):
                            probes_grid_coords = probeCoordinatesToGridCoordinates(transformed_probes[i][0],transformed_probes[i][1],transformed_probes[i][2])
                            f.write(f'Our Coordinates:\n')
                            f.write(f"Probe {i+1}:\n")
                            f.write(f"  x: {probe['x']}\n")
                            f.write(f"  y: {probe['y']}\n")
                            f.write(f"  z: {probe['z']}\n")
                            f.write(f"  confidence: {probe['confidence']}\n")
                            f.write(f"  contribution: {probe['contribution']}\n\n")

                            f.write(f'ERC Coordinates:\n')
                            f.write(f"Probe {i+1}:\n")
                            f.write(f" x: {transformed_probes[i][0]}\n")
                            f.write(f" y: {transformed_probes[i][1]}\n")
                            f.write(f" z: {transformed_probes[i][2]}\n\n")
                            
                            f.write(f"Closest box coordinates of probe {i+1}:\n")
                            f.write(f" x: {probes_grid_coords[0]}\n")
                            f.write(f" y: {probes_grid_coords[1]}\n")
                            f.write(f" z: {probes_grid_coords[2]}\n\n")
                           
                        print(f"Saved to {filename}")
                        
                except Exception as e:
                    print(f"Failed to save: {e}")
        imgui.pop_style_color(3)
        imgui.pop_style_var() 
        imgui.set_window_font_scale(1.0)

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
text_buffer, text_buffer_plan, text_buffer_spin, text_buffer_plan_spin = "", "", "", ""
speed_buffer = ""
current_item = 0
position_x, position_y, position_z = 0, 0, 0
target_position_x, target_position_y, target_position_z = 0, 0, 0
roll, pitch, yaw_velocity = 0, 0, 0
velocity_x, velocity_y, velocity_z = 0, 0, 0
thrust = 0
drone_kill = True
gimbal_slider = 0
battery_state_timestamp = 0
position_timestamp = 0
velocity_timestamp = 0
probe_timestamp = 0
takeoff_time = 0
battery_voltage, battery_current, battery_percentage = 0.0, 0.0, 0.0
battery_discharge_rate, battery_average_current = 0.0, 0.0
arming_state = 0
estop = 0
drone_state = False
GUI_console_logs = [""]
GUI_Heartbeat = 0
actuator_speeds = [0, 0, 0, 0] # Placeholder for actuator speeds
yaw = -1.94
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
probe_classification = [0.0,0.0,0.0,0.0,0.0]  # Placeholder for probe classification confidence
probe_numb = 0
temp_y_arrow, start_y_arrow = 0, 0
flight_mode = -1  # Default to a safe value, e.g., "Standby"
flight_time = 0.0
trajectory_mode = 0  # Default trajectory mode
last_trajectory_mode = None  
rotated_x, rotated_y = 0.0, 0.0
angle = 0.0
erc_yaw = 3.14
longitude, latitude, satellites_used = 0.0, 0.0, 0
distance = 0.0
distance_timestamp = 0.0
gimbal_manual = True
last_gimbal_send_time = 0.0

def probeCoordinatesToGridCoordinates(x, y, z):
    GRID_SIZE = 1  # Define the grid size
    # Find the closest grid center by rounding to nearest center (centers at 0.5, 1.5, ...)
    grid_x = round((x - 0.5) / GRID_SIZE) * GRID_SIZE + 0.5
    grid_y = round((y - 0.5) / GRID_SIZE) * GRID_SIZE + 0.5
    #grid_z = round((z - 0.5) / GRID_SIZE) * GRID_SIZE + 0.5
    grid_z = 0
    return (grid_x, grid_y, grid_z)

def construct_drone_to_global(erc_yaw, translation=np.zeros(3)):
    # Fixed rotation around X by 180 degrees
    Rx = np.array([
        [1, 0, 0],
        [0, -1, 0],
        [0, 0, -1]
    ])

    # Rotation around Z by erc_yaw
    cz, sz = np.cos(erc_yaw), np.sin(erc_yaw)
    Rz = np.array([
        [cz, -sz, 0],
        [sz, cz, 0],
        [0, 0, 1]
    ])

    # Combined rotation: Z * X
    R = Rz @ Rx

    # Build homogeneous transformation
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = translation

    # Return as tuple like before: (matrix, np.array([0,0,0,1]))
    return T, np.array([0, 0, 0, 1])


droneGlobal_to_ERC_global = construct_drone_to_global(erc_yaw)


# Variables for rotating the ERC coordinates

class DroneGuiNode(Node):
    def __init__(self):
        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        super().__init__('thyra_gui_node')
        self.position_sub = self.create_subscription(
            TelemetryPosition,
            "/asr/thyra/out/telemetry/position",
            self.position_callback,
            10
        )
        self.attitude_sub = self.create_subscription(
            TelemetryAttitude,
            "/asr/thyra/out/telemetry/attitude",
            self.attitude_callback,
            10
        )
        self.battery_sub = self.create_subscription(
            TelemetryBattery,
            "/asr/thyra/out/telemetry/battery",
            self.battery_callback,
            10
        )
        self.gps_sub = self.create_subscription(
            TelemetryGPS,
            "/asr/thyra/out/telemetry/gps",
            self.gps_callback,
            10
        )
        self.status_sub = self.create_subscription(
            TelemetryStatus,
            "/asr/thyra/out/telemetry/status",
            self.status_callback,
            10
        )
        
        self.subscription = self.create_subscription(
           ProbeGlobalLocations,
           '/asr/probe_detector/global_probe_locations',
           self.probe_callback,
           qos
           
        )

        self.subscription = self.create_subscription(
            DistanceSensor,
            "/fmu/out/distance_sensor",
            self.distance_sensor_callback,
            qos
        )
        self.publisher_ = self.create_publisher(
           GcsHeartbeat, 
           "/asr/thyra/in/gcs_heartbeat",
           qos
        )
        self.gimbal_publisher = self.create_publisher(
            ServoCommand,
             "/asr/thyra/in/servo_command",
                qos
        )


        

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.heartbeat_timer = self.create_timer(0.5, self.send_heartbeat)  # 2 Hz
        #self.gimbal_timer = self.create_timer(0.1, self.send_gimbal_command)
        self.counter = 0.0
        self.get_logger().info('GUI Publisher Started')
        self.imgui_logger = ImGuiLogger()
        self.manual_control_publisher = self.create_publisher(ManualControlInput, '/asr/thyra/in/manual_input',qos)
        self._action_client = ActionClient(self, DroneCommand, '/asr/thyra/in/drone_command')   
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
            


    def position_callback(self, msg):
        global position_x, position_y, position_z, position_timestamp
        global target_position_x, target_position_y, target_position_z
        global velocity_x, velocity_y, velocity_z, velocity_timestamp

        position_timestamp = msg.timestamp
        if len(msg.position) >= 3:
            position_x = msg.position[0]
            position_y = msg.position[1]
            position_z = msg.position[2]
        if len(msg.velocity) >= 3:
            velocity_x = msg.velocity[0]
            velocity_y = msg.velocity[1]
            velocity_z = msg.velocity[2]
        if len(msg.target_position) >= 3:
            target_position_x = msg.target_position[0]
            target_position_y = msg.target_position[1]
            target_position_z = msg.target_position[2]

    def attitude_callback(self, msg):
        global roll, pitch, yaw_velocity

        if len(msg.orientation) >= 3:
            roll = msg.orientation[0]
            pitch = msg.orientation[1]
            yaw_velocity = msg.orientation[2]

    def battery_callback(self, msg):
        global battery_voltage, battery_state_timestamp, battery_current, battery_percentage, battery_discharge_rate, battery_average_current

        battery_state_timestamp = msg.timestamp
        battery_voltage = msg.voltage
        battery_current = msg.current
        battery_percentage = msg.percentage
        battery_discharge_rate = msg.discharged_mah
        battery_average_current = msg.average_current

    def gps_callback(self, msg):
        global longitude, latitude, satellites_used

        latitude = msg.latitude
        longitude = msg.longitude
        satellites_used = msg.satellites_used

    def status_callback(self, msg):
        global arming_state, flight_mode, flight_time, trajectory_mode
        global GUI_console_logs
        global actuator_speeds

        if hasattr(self, 'last_arming_state') and self.last_arming_state != msg.arming_state:
            self.imgui_logger.warn(f"Arming state changed: {self.last_arming_state} -> {msg.arming_state}")
        if hasattr(self, 'last_flight_mode') and self.last_flight_mode != msg.flight_mode:
            self.imgui_logger.warn(f"Flight mode changed: {self.last_flight_mode} -> {msg.flight_mode}")

        self.last_arming_state = msg.arming_state
        self.last_flight_mode = msg.flight_mode

        arming_state = msg.arming_state
        flight_mode = msg.flight_mode
        trajectory_mode = msg.trajectory_mode
        flight_time = msg.flight_time

        if len(msg.actuator_speeds) >= 4:
            actuator_speeds[0] = msg.actuator_speeds[0]
            actuator_speeds[1] = msg.actuator_speeds[1]
            actuator_speeds[2] = msg.actuator_speeds[2]
            actuator_speeds[3] = msg.actuator_speeds[3]

        GUI_console_logs[0] = str(self.get_logger())
    def probe_callback(self, msg):
        global probes
        global probe_numb, probe_classification
        global probe_timestamp

        # Extract the number of probes
        

        # Extract classification confidence (assuming it's derived from the confidence list or a separate field)
        # If classification_confidence is meant to be the average of msg.confidence, we can compute it
        

        # Extract probe data into a structured format (list of dictionaries)
        probes = []
        for i in range(msg.probe_count):
            probe = {
                'x': msg.x[i],
                'y': msg.y[i],
                'z': msg.z[i],
                'confidence': msg.confidence[i],
                'contribution': msg.contribution[i]
            }
            probes.append(probe)
        #probe_classification = sum(msg.confidence) / len(msg.confidence) if msg.confidence else 0.0
        
        probe_numb = msg.probe_count
        #print(f"Probe count: {probe_numb}")
        # Optionally, store the timestamp if needed
        probe_timestamp = msg.stamp
    def distance_sensor_callback(self, msg):
        global distance
        distance = msg.current_distance


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
    def send_manual_control(self, roll, pitch, yaw_velocity_m, thrust):
        msg = ManualControlInput()
        msg.roll = float(roll)
        msg.pitch = float(pitch)
        msg.yaw_velocity = float(yaw_velocity_m)
        msg.thrust = float(thrust)
        self.manual_control_publisher.publish(msg)
    def send_heartbeat(self):
        msg = GcsHeartbeat()
        msg.timestamp = float(self.get_clock().now().nanoseconds) / 1e9
        msg.gcs_nominal = 1
        self.publisher_.publish(msg)
        
    def send_gimbal_command(self, gimbal_slider, gimbal_id):
        # This function can be expanded to send gimbal commands if needed
        msg = ServoCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000) 
        msg.id = gimbal_id
        msg.value = float(gimbal_slider)
        self.gimbal_publisher.publish(msg)

       

def Arm_Button(node):
    global button_color, drone_kill, drone_state
    global arming_state, flight_time
    imgui.set_cursor_pos((415, 30))
    button_color = (0.0, 0.5, 0.0) if drone_kill else (1.0, 0.0, 0.0)
    button_text = " Disarmed " if drone_kill else "  Armed   "
    imgui.push_style_var(imgui.STYLE_FRAME_ROUNDING, 12.0)
    imgui.push_style_color(imgui.COLOR_BUTTON, *button_color)
    imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, *button_color)
    imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, *button_color)
    if arming_state == 1:
        drone_kill = False
        drone_state = True
        button_text = "  Armed   "
    elif arming_state == 0:
        drone_kill = True
        drone_state = False
        button_text = " Disarmed "
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
  
    draw_list = imgui.get_window_draw_list()
    start_x, start_y = 415, 70
    end_x, end_y = 1065, 70
    color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5)
    draw_list.add_line(start_x, start_y, end_x, end_y, color, 5.0)
    with imgui.font(font_small):
        imgui.set_cursor_pos((750, 30)); imgui.text(f"Flight time = {Decimal(flight_time).quantize(Decimal('0.000'))}")
    
def Kill_command(node):
    global killbutton_color, drone_kill
    imgui.set_cursor_pos((1075, 30))  # Changed x-coordinate to 1130
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
                node.send_command("velocity", [speed])  
                node.get_logger().info(f"KEK set to {speed}")
                node.imgui_logger.info(f"KEK set to {speed}")
            except ValueError as e:
                node.get_logger().warn(f"KEK speed input: {e}")
                node.imgui_logger.warn(f"KEK speed input: {e}")
    imgui.pop_style_color(3)
    imgui.pop_style_var()
 
def slider_field(node):
    global gimbal_slider, gimbal_manual, last_gimbal_send_time
    imgui.set_window_font_scale(1.0)

    imgui.set_cursor_pos((10, 935))
    with imgui.font(font_for_meter):

        if gimbal_manual:
            imgui.push_style_color(imgui.COLOR_BUTTON, 0.8, 0.2, 0.2, 1.0)  # Red = active
            label = "Stop"
        else:
            imgui.push_style_color(imgui.COLOR_BUTTON, 0.2, 0.6, 0.2, 1.0)  # Green = inactive
            label = "Activate"

        if imgui.button(label, width=100, height=30):
            gimbal_manual = not gimbal_manual  # Toggle on each press

        imgui.pop_style_color()

 
    imgui.set_cursor_pos((10, 995))
    with imgui.font(font_for_meter):
        imgui.text("Gimbal Slider:")

    imgui.set_cursor_pos((10, 975))
    imgui.set_next_item_width(300)
    changed, gimbal_slider = imgui.slider_float("##gimbal_slider", gimbal_slider, -1.0, 1.0)

    if gimbal_manual:
        now = time.time()
        if now - last_gimbal_send_time >= 0.1:
            node.send_gimbal_command(gimbal_slider, 1)
            last_gimbal_send_time = now
    
def XYZ_Text_Field(msg):
    # Drawing a square kek
    global position_x, position_y, position_z
    global target_position_x, target_position_y, target_position_z
    global position_timestamp
    global distance

    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5)

    # 420 -> 385 (35px shorter in x direction)
    draw_list.add_rect_filled(5, 90, 385, 235, color, rounding=10.0, flags=10)

    with imgui.font(font_small):
        imgui.set_cursor_pos((8, 50)); imgui.text("Positon:| Current:| Target: ")
        imgui.set_cursor_pos((45, 95)); imgui.text("X = ")
        imgui.set_cursor_pos((45, 145)); imgui.text("Y = ")
        imgui.set_cursor_pos((45, 195)); imgui.text("Z = ")

        imgui.set_cursor_pos((140, 93)); imgui.text(f"{Decimal(position_x).quantize(Decimal('0.000'))}")
        imgui.set_cursor_pos((140, 143)); imgui.text(f"{Decimal(position_y).quantize(Decimal('0.000'))}")
        imgui.set_cursor_pos((140, 193)); imgui.text(f"{(Decimal(position_z).quantize(Decimal('0.000')))}")

        # target column shifted 35px left: 313 -> 278
        imgui.set_cursor_pos((263, 93)); imgui.text(f"{Decimal(target_position_x).quantize(Decimal('0.000'))}")
        imgui.set_cursor_pos((263, 143)); imgui.text(f"{Decimal(target_position_y).quantize(Decimal('0.000'))}")
        imgui.set_cursor_pos((263, 193)); imgui.text(f"{(Decimal(target_position_z).quantize(Decimal('0.000')))}")
        imgui.set_cursor_pos((9, 240)); imgui.text(f"[m]")
        imgui.set_cursor_pos((180, 270)); imgui.text("|Height")
        imgui.set_cursor_pos((300, 270)); imgui.text(f"{Decimal(distance).quantize(Decimal('0.000'))}")
        # separator lines
        draw_list = imgui.get_window_draw_list()

        # left separator unchanged
        start_x, start_y = 120, 90
        end_x, end_y = 120, 235
        color_line = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 1.0)
        draw_list.add_line(start_x, start_y, end_x, end_y, color_line, 5.0)

        # right separator shifted 35px left: 287 -> 252
        draw_list = imgui.get_window_draw_list()
        start_x, start_y = 237, 90
        end_x, end_y = 237, 235
        color_line = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 1.0)
        draw_list.add_line(start_x, start_y, end_x, end_y, color_line, 5.0)

    imgui.set_cursor_pos((88, 250)); imgui.text(f" TS: {position_timestamp}")


def XYZVelocity_Text_Field():
    global velocity_x, velocity_y, velocity_z, velocity_timestamp

    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5)

    # this panel ends at 250 already, so NO CHANGE needed for "35px shorter"
    draw_list.add_rect_filled(5, 310, 250, 455, color, rounding=10.0, flags=10)

    with imgui.font(font_small):
        imgui.set_cursor_pos((8, 270)); imgui.text("Velocity:")
        imgui.set_cursor_pos((45, 315)); imgui.text("X   = ")
        imgui.set_cursor_pos((135, 313)); imgui.text(f"{Decimal(velocity_x).quantize(Decimal('0.000'))}")
        imgui.set_cursor_pos((45, 365)); imgui.text("Y   = ")
        imgui.set_cursor_pos((135, 363)); imgui.text(f"{Decimal(velocity_y).quantize(Decimal('0.000'))}")
        imgui.set_cursor_pos((45, 415)); imgui.text("Z   = ")
        imgui.set_cursor_pos((135, 413)); imgui.text(f"{-(Decimal(velocity_z).quantize(Decimal('0.000')))}")
        imgui.set_cursor_pos((9, 460)); imgui.text(f"[m/s]")

    imgui.set_cursor_pos((88, 470)); imgui.text(f" TS: {velocity_timestamp}")

def RPY_Text_Field():
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5)

    # this panel ends at 250 already, so NO CHANGE needed for "35px shorter"
    draw_list.add_rect_filled(5, 530, 250, 675, color, rounding=10.0, flags=10)

    with imgui.font(font_small):
        imgui.set_cursor_pos((8, 490)); imgui.text("Orientation:")
        imgui.set_cursor_pos((15, 540)); imgui.text("Roll  = ")
        imgui.set_cursor_pos((135, 538)); imgui.text(f"{Decimal(roll).quantize(Decimal('0.00'))}")
        imgui.set_cursor_pos((15, 590)); imgui.text("Pitch = ")
        imgui.set_cursor_pos((135, 588)); imgui.text(f"{Decimal(pitch).quantize(Decimal('0.00'))}")
        imgui.set_cursor_pos((15, 640)); imgui.text("Yaw   = ")
        imgui.set_cursor_pos((135, 638)); imgui.text(f"{Decimal(yaw_velocity).quantize(Decimal('0.00'))}")
        imgui.set_cursor_pos((8, 680)); imgui.text(f"[°]")


def probe_Field(node, filename):
    global probes
    global probe_classification, probe_numb
    global probe_timestamp
    global transformed_erc_probes

    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5)

    # convert probes into numpy array
    try:
        probe_array = np.array([[probe['x'], probe['y'], probe['z'], 1] for probe in probes]).T
        transformed_probes = droneGlobal_to_ERC_global[0] @ probe_array
        transformed_probes = transformed_probes[:3, :].T
    except Exception as e:
        transformed_probes = np.array([])

    draw_list.add_rect_filled(5, 750, 385, 895, color, rounding=10.0, flags=10)

    # Column separator lines
    start_x, start_y = 65, 750
    end_x, end_y = 65, 895
    color_line = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 1.0)
    draw_list.add_line(start_x, start_y, end_x, end_y, color_line, 5.0)

    start_x, start_y = 150, 750
    end_x, end_y = 150, 895
    draw_list.add_line(start_x, start_y, end_x, end_y, color_line, 5.0)

    start_x, start_y = 235, 750
    end_x, end_y = 235, 895
    draw_list.add_line(start_x, start_y, end_x, end_y, color_line, 5.0)

    start_x, start_y = 285, 750
    end_x, end_y = 285, 895
    draw_list.add_line(start_x, start_y, end_x, end_y, color_line, 5.0)

    with imgui.font(font_small):
        imgui.set_cursor_pos((8, 710)); imgui.text("Probe info:")
        imgui.set_cursor_pos((15, 755)); imgui.text("ID")
        imgui.set_cursor_pos((100, 755)); imgui.text("X")
        imgui.set_cursor_pos((185, 755)); imgui.text("Y")

        # These headers shifted slightly left because Z divider changed
        imgui.set_cursor_pos((255, 755)); imgui.text("Z")      
        imgui.set_cursor_pos((330, 755)); imgui.text("C")      

    try:
        i = 0
        draw_list = imgui.get_window_draw_list()

        # Always use map_value for drawing probe circles
        if probe_numb == 1:
            circle_x = map_value(probes[i]['x'], 10, -10, 467, 1228) 
            circle_y = map_value(probes[i]['y'], 10, -10, 158, 599)
            plan_card.probe_cards(0, 1, transformed_probes, i, probes, node)
            draw_list.add_circle_filled(circle_x, circle_y, 4, imgui.get_color_u32_rgba(0.0, 0.0, 1.0, 1.0))

        if probe_numb == 2:
            circle_x = map_value(probes[i]['x'], 10, -10, 467, 1228)
            circle_y = map_value(probes[i]['y'], 10, -10, 158, 599)
            plan_card.probe_cards(0, 1, transformed_probes, i, probes, node)
            draw_list.add_circle_filled(circle_x, circle_y, 4, imgui.get_color_u32_rgba(0.0, 0.0, 1.0, 1.0))

            circle_x_one = map_value(probes[i+1]['x'], 10, -10, 467, 1228)
            circle_y_one = map_value(probes[i+1]['y'], 10, -10, 158, 599)
            plan_card.probe_cards(20, 2, transformed_probes, i+1, probes, node)
            draw_list.add_circle_filled(circle_x_one, circle_y_one, 4, imgui.get_color_u32_rgba(0.0, 0.0, 1.0, 1.0))

        if probe_numb == 3:
            circle_x = map_value(probes[i]['x'], 10, -10, 467, 1228)
            circle_y = map_value(probes[i]['y'], 10, -10, 158, 599)
            plan_card.probe_cards(0, 1, transformed_probes, i, probes, node)
            draw_list.add_circle_filled(circle_x, circle_y, 4, imgui.get_color_u32_rgba(0.0, 0.0, 1.0, 1.0))

            circle_x_one = map_value(probes[i+1]['x'], 10, -10, 467, 1228)
            circle_y_one = map_value(probes[i+1]['y'], 10, -10, 158, 599)
            plan_card.probe_cards(20, 2, transformed_probes, i+1, probes, node)
            draw_list.add_circle_filled(circle_x_one, circle_y_one, 4, imgui.get_color_u32_rgba(0.0, 0.0, 1.0, 1.0))

            circle_x_two = map_value(probes[i+2]['x'], 10, -10, 467, 1228)
            circle_y_two = map_value(probes[i+2]['y'], 10, -10, 158, 599)
            plan_card.probe_cards(40, 3, transformed_probes, i+2, probes, node)
            draw_list.add_circle_filled(circle_x_two, circle_y_two, 4, imgui.get_color_u32_rgba(0.0, 0.0, 1.0, 1.0))

        if probe_numb == 4:
            circle_x = map_value(probes[i]['x'], 10, -10, 467, 1228)
            circle_y = map_value(probes[i]['y'], 10, -10, 158, 599)
            plan_card.probe_cards(0, 1, transformed_probes, i, probes, node)
            draw_list.add_circle_filled(circle_x, circle_y, 4, imgui.get_color_u32_rgba(0.0, 0.0, 1.0, 1.0))

            circle_x_one = map_value(probes[i+1]['x'], 10, -10, 467, 1228)
            circle_y_one = map_value(probes[i+1]['y'], 10, -10, 158, 599)
            plan_card.probe_cards(20, 2, transformed_probes, i+1, probes, node)
            draw_list.add_circle_filled(circle_x_one, circle_y_one, 4, imgui.get_color_u32_rgba(0.0, 0.0, 1.0, 1.0))

            circle_x_two = map_value(probes[i+2]['x'], 10, -10, 467, 1228)
            circle_y_two = map_value(probes[i+2]['y'], 10, -10, 158, 599)
            plan_card.probe_cards(40, 3, transformed_probes, i+2, probes, node)
            draw_list.add_circle_filled(circle_x_two, circle_y_two, 4, imgui.get_color_u32_rgba(0.0, 0.0, 1.0, 1.0))

            circle_x_three = map_value(probes[i+3]['x'], 10, -10, 467, 1228)
            circle_y_three = map_value(probes[i+3]['y'], 10, -10, 158, 599)
            plan_card.probe_cards(60, 4, transformed_probes, i+3, probes, node)
            draw_list.add_circle_filled(circle_x_three, circle_y_three, 4, imgui.get_color_u32_rgba(0.0, 0.0, 1.0, 1.0))

        if probe_numb >= 5:
            circle_x = map_value(probes[i]['x'], 10, -10, 467, 1228)
            circle_y = map_value(probes[i]['y'], 10, -10, 158, 599)
            plan_card.probe_cards(0, 1, transformed_probes, i, probes, node)
            draw_list.add_circle_filled(circle_x, circle_y, 4, imgui.get_color_u32_rgba(0.0, 0.0, 1.0, 1.0))

            circle_x_one = map_value(probes[i+1]['x'], 10, -10, 467, 1228)
            circle_y_one = map_value(probes[i+1]['y'], 10, -10, 158, 599)
            plan_card.probe_cards(20, 2, transformed_probes, i+1, probes, node)
            draw_list.add_circle_filled(circle_x_one, circle_y_one, 4, imgui.get_color_u32_rgba(0.0, 0.0, 1.0, 1.0))

            circle_x_two = map_value(probes[i+2]['x'], 10, -10, 467, 1228)
            circle_y_two = map_value(probes[i+2]['y'], 10, -10, 158, 599)
            plan_card.probe_cards(40, 3, transformed_probes, i+2, probes, node)
            draw_list.add_circle_filled(circle_x_two, circle_y_two, 4, imgui.get_color_u32_rgba(0.0, 0.0, 1.0, 1.0))

            circle_x_three = map_value(probes[i+3]['x'], 10, -10, 467, 1228)
            circle_y_three = map_value(probes[i+3]['y'], 10, -10, 158, 599)
            plan_card.probe_cards(60, 4, transformed_probes, i+3, probes, node)
            draw_list.add_circle_filled(circle_x_three, circle_y_three, 4, imgui.get_color_u32_rgba(0.0, 0.0, 1.0, 1.0))

            circle_x_four = map_value(probes[i+4]['x'], 10, -10, 467, 1228)
            circle_y_four = map_value(probes[i+4]['y'], 10, -10, 158, 599)
            plan_card.probe_cards(80, 5, transformed_probes, i+4, probes, node)
            draw_list.add_circle_filled(circle_x_four, circle_y_four, 4, imgui.get_color_u32_rgba(0.0, 0.0, 1.0, 1.0))

    except:
        pass

    imgui.set_cursor_pos((53, 900)); imgui.text(f" TS: {velocity_timestamp}")
    GUIButton.button_save(275, 650, font, "save", filename, probes, transformed_probes)
def batteryGraph():
    global battery_voltage, battery_current, battery_percentage, battery_average_current
    battery_progressbar = map_value(battery_percentage, 0, 1, 109, 44)
    graphs.battery_graph(1800, 40, 1840, 110, 1807, 31, 1832, 38)
    if(battery_percentage > 0.5):
        battery_color = imgui.get_color_u32_rgba(0.0, 1.0, 0.0, 1.0)
    elif(battery_percentage > 0.25):
        battery_color = imgui.get_color_u32_rgba(1.0, 1.0, 0.0, 1.0)
    else:
        battery_color = imgui.get_color_u32_rgba(1.0, 0.0, 0.0, 1.0)
    imgui.set_cursor_pos((1799, 116)); imgui.text(f"{Decimal(100*battery_percentage).quantize(Decimal('0.00'))} %")
    draw_list = imgui.get_window_draw_list()
    draw_list.add_rect_filled(1803,106,1837,(battery_progressbar),battery_color, rounding=1.0,flags=15)
    imgui.set_cursor_pos((1595, 30)); imgui.text(f"Voltage:          {Decimal(battery_voltage).quantize(Decimal('0.0'))} V")
    imgui.set_cursor_pos((1595, 60)); imgui.text(f"Current:          {-(Decimal(battery_current).quantize(Decimal('0.0')))} A")
    imgui.set_cursor_pos((1595, 90)); imgui.text(f"Discharge rate:   {Decimal(battery_discharge_rate).quantize(Decimal('0.0'))} mAh")
    imgui.set_cursor_pos((1595, 120)); imgui.text(f"Average current:  {-(Decimal(battery_average_current).quantize(Decimal('0.0')))} A")
    draw_list = imgui.get_window_draw_list()
    start_x, start_y = 1785, 30  # Starting point of the line (x, y)
    end_x, end_y = 1785, 132      # Ending point of the line (x, y)
    color = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 1.0) 
    draw_list.add_line(start_x,start_y, end_x, end_y, color, 2.0)
    draw_list = imgui.get_window_draw_list()
    start_x, start_y = 1714, 30  # Starting point of the line (x, y)
    end_x, end_y = 1714, 132      # Ending point of the line (x, y)
    color = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 1.0) 
    draw_list.add_line(start_x,start_y, end_x, end_y, color, 2.0)
    draw_list = imgui.get_window_draw_list()
    start_x, start_y = 1585, 30  # Starting point of the line (x, y)
    end_x, end_y = 1585, 132      # Ending point of the line (x, y)
    color = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 1.0) 
    draw_list.add_line(start_x,start_y, end_x, end_y, color, 2.0)
    imgui.set_cursor_pos((1655, 175)); imgui.text(f" TS:  {battery_state_timestamp}")

def map_value(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def drone_visualization():
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5) 
    #flags is for rounding different corners
    draw_list.add_rect(415,140, 1225,620,color,rounding =10.0, flags=15,thickness=6)
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 0.9) 
    draw_list.add_rect_filled(418,143, 1221,624,color,rounding =10.0, flags=15)

def grid():
    global position_x, position_y
    dot_position_x = map_value(position_x, 10, -10, 432, 1193)
    dot_position_y = map_value(position_y, 10, -10, 158, 599)
    drawlist = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.2, 0.6, 0.9, 0.8)  # Light blue color
    for x in range(432, 1221, 38):
        drawlist.add_line(x, 145, x, 620, color, 1.5)
    for y in range(157, 618, 22):
        drawlist.add_line(420, y, 1219, y, color, 1.5)
    
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(1.0, 0.0, 0.0, 1.0)
    
    if 414 < dot_position_x < 1225 and 155 < dot_position_y < 620: 
        draw_list.add_circle_filled(dot_position_x, dot_position_y, 4, color)  # Red dot at the center Center of the grid
    elif 414 > dot_position_x:
        if 155 < dot_position_y < 620:
            draw_list.add_rect_filled(414, dot_position_y-2, 434, dot_position_y+2, color, rounding=2.0)
    elif dot_position_x > 1225:
        if 155 < dot_position_y < 620:
            draw_list.add_rect_filled(1225, dot_position_y-2, 1205, dot_position_y+2, color, rounding=2.0)
    elif 140 > dot_position_y:
        if 414 < dot_position_x < 1225:
            draw_list.add_rect_filled(dot_position_x-2, 140, dot_position_x+2, 160, color, rounding=2.0)
    elif dot_position_y > 620:
        if 414 < dot_position_x < 1225:
            draw_list.add_rect_filled(dot_position_x-2, 620, dot_position_x+2, 600, color, rounding=2.0)
    
    imgui.push_style_color(imgui.COLOR_TEXT, 0.0, 0.0, 0.0, 1.0)
    imgui.set_cursor_pos((427, 610)); imgui.text("10   9    8     7    6     5     4    3     2    1    0    -1    -2   -3   -4   -5    -6    -7    -8    -9   -10")
    j = 10
    for i in range(40, 482, 22):
        imgui.set_cursor_pos((1200, 105+i)); imgui.text(f"{j}")
        j -= 1
    imgui.pop_style_color()
    # max 414 1193 x
    # max 155 561 y

def rotate_x_and_y(position_x, position_y, angle):
    global rotated_x, rotated_y
    global probes
    rotated_x = math.cos(angle) * position_x - math.sin(angle) * position_y
    rotated_y = math.sin(angle) * position_x + math.cos(angle) * position_y

def return_to_home_button(node):
    imgui.set_cursor_pos((649, 635))
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
    global flight_mode
    imgui.set_cursor_pos((1527, 200))
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
                    node.send_command("manual_aided")

                    node.imgui_logger.info("Manual mode activated")
                else:
                    print("Joystick not connected or not initialized.")
            except AttributeError:
                print("Joystick not initialized or not available.")
              
   
    imgui.pop_style_color(3)
    imgui.pop_style_var()
    flight_mode_text = ""
    if flight_mode == -3:
        flight_mode_text = "E-Stop"
    elif flight_mode == -2:
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
    
    imgui.set_cursor_pos((435, 100))
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

        current_item = 0
        return
    
    node.joystick = pygame.joystick.Joystick(0)
    
    node.joystick.init()
    if node.joystick.get_name() == "Sony Interactive Entertainment Wireless Controller":
        current_item = 1
        print(f"Initialized joystick: {node.joystick.get_name()}")
        
    elif node.joystick.get_name() == "OpenTX RM TX16S Joystick":
        current_item = 2
    else:
        current_item = 3
        print(f"Initialized joystick: {node.joystick.get_name()}")


    
    

    clock = pygame.time.Clock()
    DEAD_ZONE = 0.05
    prev_axis_state = None
    
    match current_item:
        case 1:
            try:
                while rclpy.ok():
                    pygame.event.pump()  # Process internal queue
                    
                    left_trigger = ((node.joystick.get_axis(2) + 1.0)/2.0)
                    right_trigger = ((node.joystick.get_axis(5) + 1.0)/2.0)
                    yaw_value = (right_trigger - left_trigger) * 2.0
                    print(f"Left Trigger: {node.joystick.get_axis(2)}, Right Trigger: {node.joystick.get_axis(5)}")
                    #for i in range(node.joystick.get_numaxes()):
                    #    if node.joystick.get_axis(i) > 0.5:
                    #        print(f"Axis {i} activated")
                    roll_m = node.joystick.get_axis(0) if abs(node.joystick.get_axis(0)) > DEAD_ZONE else 0.0
                    pitch_m = -node.joystick.get_axis(1) if abs(node.joystick.get_axis(1)) > DEAD_ZONE else 0.0
                    yaw_velocity_m = yaw_value if abs(yaw_value) > DEAD_ZONE else 0.0
                    print(f"Yaw Value: {yaw_velocity_m}")
                    thrust = -node.joystick.get_axis(4) if abs(node.joystick.get_axis(4)) > DEAD_ZONE else 0.0
                    current_axis_state = int(abs(node.joystick.get_axis(4)))
                    hello = node.joystick.get_button(3) #command to test which buttons
     
                    if(node.joystick.get_button(2) == 1):
                        node.send_command("arm")
                        arming_state = 1
                        node.send_command("manual_aided")
                    if( node.joystick.get_button(1) == 1):
                        node.send_command("estop")
                        drone_kill = True

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
                    roll_m = node.joystick.get_axis(0) if abs(node.joystick.get_axis(0)) > DEAD_ZONE else 0.0
                    pitch_m = -node.joystick.get_axis(1) if abs(node.joystick.get_axis(1)) > DEAD_ZONE else 0.0
                    yaw_velocity_m = node.joystick.get_axis(2) if abs(node.joystick.get_axis(2)) > DEAD_ZONE else 0.0
                    thrust = -node.joystick.get_axis(1) if abs(node.joystick.get_axis(1)) > DEAD_ZONE else 0.0
                    
                    current_axis_state = int(abs(node.joystick.get_axis(4)))


                    if prev_axis_state is None or current_axis_state != prev_axis_state:
                        if current_axis_state == 1:
                            node.send_command("disarm")
   
                        elif current_axis_state == 0:
                            node.send_command("arm")
  
                        prev_axis_state = current_axis_state
                    arming_state = -int(abs(node.joystick.get_axis(4)))

                
                    if( node.joystick.get_button(3) == 1):
                        node.send_command("estop")
                        drone_kill = True

                    node.send_manual_control(roll_m, pitch_m, yaw_velocity_m, thrust)
                    clock.tick(20)  # 20 Hz update rate
            except KeyboardInterrupt:
                pass
            finally:
                pygame.quit()
        case 3:
            try:
                while rclpy.ok():
                    pygame.event.pump()  # Process internal queue
                    left_trigger = (node.joystick.get_button(4) + 1.0)/2.0
                    right_trigger = (node.joystick.get_button(5) + 1.0)/2.0
                    
                    yaw_value = right_trigger - left_trigger
                    roll_m = node.joystick.get_axis(0) if abs(node.joystick.get_axis(0)) > DEAD_ZONE else 0.0
                    pitch_m = -node.joystick.get_axis(1) if abs(node.joystick.get_axis(1)) > DEAD_ZONE else 0.0
                    yaw_velocity_m = yaw_value if abs(yaw_value) > DEAD_ZONE else 0.0
                    thrust = -node.joystick.get_axis(3) if abs(node.joystick.get_axis(3)) > DEAD_ZONE else 0.0
                    
                    if(node.joystick.get_button(3) == 1):
                        node.send_command("arm")
                        arming_state = 1
                        drone_state = True
                        node.send_command("manual_aided")
                    
          
                    if( node.joystick.get_button(2) == 1):
                        node.send_command("estop")
                        drone_kill = True
                    
                    if( node.joystick.get_button(0) == 1):
                        node.send_command("land", [-1.0])
        

                    node.send_manual_control(roll_m, pitch_m, yaw_velocity_m, thrust)
                        
                    
                    clock.tick(20)  # 20 Hz update rate
            except KeyboardInterrupt:
                pass
            finally:
                pygame.quit()

def GuiConsoleLogger(node):
    global font_small
    
    # Position the logger window in the bottom right area
    imgui.set_cursor_pos((415, 690))
    
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
    graphs.motor_speed_graph(1265, 50, 1325, 110) 
    graphs.motor_speed_graph(1345, 50, 1405, 110)
    graphs.motor_speed_graph(1425, 50, 1485, 110)
    graphs.motor_speed_graph(1505, 50, 1565, 110)
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
    draw_list.add_rect_filled(1268,106,1322,actuator_speeds_slider_bar1,color1, rounding=1.0,flags=15)   
    imgui.set_cursor_pos((1269, 120)); imgui.text(f"{Decimal(actuator_speeds[0]).quantize(Decimal('0.00'))}")
    draw_list = imgui.get_window_draw_list()
    draw_list.add_rect_filled(1348,106,1402,actuator_speeds_slider_bar2,color2, rounding=1.0,flags=15)
    imgui.set_cursor_pos((1349, 120)); imgui.text(f"{Decimal(actuator_speeds[1]).quantize(Decimal('0.00'))}")
    draw_list = imgui.get_window_draw_list()
    draw_list.add_rect_filled(1428,106,1482,actuator_speeds_slider_bar3,color3, rounding=1.0,flags=15)
    imgui.set_cursor_pos((1429, 120)); imgui.text(f"{Decimal(actuator_speeds[2]).quantize(Decimal('0.00'))}")
    draw_list = imgui.get_window_draw_list()
    draw_list.add_rect_filled(1508,106,1562,actuator_speeds_slider_bar4,color4, rounding=1.0,flags=15)
    imgui.set_cursor_pos((1509, 120)); imgui.text(f"{Decimal(actuator_speeds[3]).quantize(Decimal('0.00'))}")
    with imgui.font(font_small):
        imgui.set_cursor_pos((1285, 20)); imgui.text("M1")
        imgui.set_cursor_pos((1365, 20)); imgui.text("M2")
        imgui.set_cursor_pos((1445, 20)); imgui.text("M3")
        imgui.set_cursor_pos((1525, 20)); imgui.text("M4")

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
    global flight_plan, text_buffer_plan, flight_plan_numb, flight_plan_coord, execute_route_numb, text_buffer_plan_spin
    global effect2, current_step, temp_y_arrow, start_y_arrow 
    text_field = ""
    text_field_spin = ""
    mouse_x, mouse_y = imgui.get_mouse_pos()
    imgui.set_cursor_pos((1655, 260))
    imgui.set_window_font_scale(2.0)
    imgui.set_next_item_width(180)
    changed, text_field= imgui.input_text("##goto_input_plan", text_buffer_plan, 20)
    if changed:
        text_buffer_plan = text_field
    imgui.set_cursor_pos((1655, 440))
    imgui.set_window_font_scale(2.0)
    imgui.set_next_item_width(180)
    changed, text_field_spin= imgui.input_text("##spin_input_plan", text_buffer_plan_spin, 20)
    if changed:
        text_buffer_plan_spin = text_field_spin  
    imgui.set_window_font_scale(1.0) 
    color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5) 
    draw_list = imgui.get_window_draw_list()
    draw_list.add_rect_filled(1250,140, 1521, 624,color,rounding =10.0, flags=15) #e 1505
    draw_list.add_line(1250,180,1521, 180,imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 0.9), 5)

    with imgui.font(font_small):
        imgui.set_cursor_pos((1303,145)); imgui.text("Flight Plan")


    if GUIButton.button_Plan(1527, 316, font_small, "Takeoff##plan1"):
        
        for i, val in enumerate(flight_plan):
            if val == 0:
                flight_plan[i] = 1
                flight_plan_numb += 1
                break
    if GUIButton.button_Plan(1527, 376, font_small, "Land##plan2"):
        
        for i, val in enumerate(flight_plan):
            if val == 0:
                flight_plan[i] = 2
                flight_plan_numb += 1
                break
    if GUIButton.button_Plan(1527, 256, font_small, "Goto##plan3"):
        
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
    if GUIButton.button_Plan(1401,635,font_small, "Clear##plan4"):
        for i in range(7):
            flight_plan[i] = 0
            flight_plan_numb = 0
            flight_plan_coord[i][0] = ""
            current_step = 0
            temp_y_arrow = 0
            start_y_arrow = 0
            execute_route_numb = 0

    if GUIButton.button_Plan(1527,436,font_small, "Spin##plan4"):
        for i, val in enumerate(flight_plan):
            if val == 0:
                flight_plan[i] = 4
                try:
                    flight_plan_coord[flight_plan_numb] = [text_buffer_plan_spin]
                    flight_plan_numb += 1
                    
                    text_buffer_plan_spin = ""
                    text_field_spin = ""
                except ValueError:
                     node.imgui_logger.warn("Invalid input for Spin, please enter yaw rotations direction values")
                break
    

def is_T_close(target_pos_x, target_pos_y):
    global position_x, position_y
    dx = target_pos_x - position_x
    dy = target_pos_y - position_y
    distance = math.sqrt(dx * dx + dy * dy)
    return distance <= 0.5

def is_trajectory_complete():
    global trajectory_mode, last_trajectory_mode
    # Only return True if last_trajectory_mode is not 2 and current is 2
    result = False
    if last_trajectory_mode is not None and last_trajectory_mode != 2 and trajectory_mode == 2:
        result = True
    last_trajectory_mode = trajectory_mode
    return result

def gps_status():
    global longitude, latitude, satellites_used
    imgui.set_cursor_pos((425, 940))
    with imgui.font(font_small):
        imgui.text(f"GPS: Lat = {Decimal(latitude).quantize(Decimal('0.00000'))} |Lon = {Decimal(longitude).quantize(Decimal('0.00000'))} |Satellites = {satellites_used}")
    

def execute_route(node):
    global flight_plan, flight_plan_coord
    global start_time, plan_duration, begin_execute
    global execute_route_numb, effect3
    global position_x, position_y, position_z
    global current_step, step_start_time, command_sent, waiting_for_completion
    if GUIButton.button_Plan(1250, 635, font_small, "Excecute##plan4"):
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
                        if is_trajectory_complete():
                            effect3 = True

                            step_finished = True
                            execute_route_numb += 1
                except (ValueError, IndexError):
                    step_finished = True
                    execute_route_numb += 1
        case 4: # Spin
            if not command_sent:
                try:
                    if execute_route_numb < len(flight_plan_coord) and flight_plan_coord[execute_route_numb]:
                        yaw, rotations, distance = map(float, flight_plan_coord[execute_route_numb][0].strip().split())

                        node.send_command("spin", [yaw, rotations, distance], 0.0)
                        node.imgui_logger.info("Spinning weeee")
                        command_sent = True
                        waiting_for_completion = True
                    else:
                        step_finished = True
                except ValueError:
                    node.get_logger().warn("Invalid spin input")
                    node.imgui_logger.warn("Invalid spin input")
                    step_finished = True

            if waiting_for_completion and elapsed > 0.5:
                try:
                    if execute_route_numb < len(flight_plan_coord) and flight_plan_coord[execute_route_numb]:
                        yaw, rotations, distance = map(float, flight_plan_coord[execute_route_numb][0].strip().split())
                        if is_trajectory_complete():
                            effect3 = True

                            step_finished = True
                            execute_route_numb += 1
                except (ValueError, IndexError):
                    step_finished = True
                    execute_route_numb += 1



    # If the step finished, move to next one, but don’t process it this frame
    if step_finished:
        waiting_for_completion = False
        command_sent = False
        step_start_time = -1  
        current_step += 1  
        return
    
    
        
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
            if t >= 1.0:
                effect_begin3 = False
                start_time = None
                temp_y_arrow = start_y_arrow + 52

            draw_list.add_triangle_filled(1231, 201+(temp_y_arrow ), 1231, 231+(temp_y_arrow ), 1247, 216+(temp_y_arrow ), color)
        else:
            draw_list.add_triangle_filled(1231, 201+(temp_y_arrow), 1231, 231+(temp_y_arrow), 1247, 216+(temp_y_arrow), color)

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
            case 4:
                text = str(text)
                clean_text = re.sub(r"[^\d.\s\-+]", " ", text)
                clean_text = " ".join(clean_text.split())   
                # Draw card background
                color = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 0.9) 
                draw_list = imgui.get_window_draw_list()
                draw_list.add_rect_filled(card_pos_x, card_pos_y, card_pos_x + 265, card_pos_y + 50, color, rounding=10.0, flags=15)    
                imgui.push_style_color(imgui.COLOR_TEXT, 0.0, 0.0, 0.0, 1.0)
                imgui.set_cursor_pos((card_pos_x + 35, card_pos_y + 20))
                imgui.text(f"Spin : {clean_text}")   
                with imgui.font(font):
                    imgui.set_cursor_pos((card_pos_x + 230, card_pos_y - 6)); imgui.text("-")
                    imgui.set_cursor_pos((card_pos_x + 230, card_pos_y + 4)); imgui.text("-")
                    imgui.set_cursor_pos((card_pos_x + 230, card_pos_y + 14)); imgui.text("-")
                imgui.pop_style_color()
          

                
                
    def probe_cards(y_add, id, probes, i, og_probes, node):
        imgui.set_cursor_pos((30, 790+y_add)); imgui.text(f"{id}")
        imgui.set_cursor_pos((100, 790+y_add)); imgui.text(f"{Decimal(probes[i][0]).quantize(Decimal('0.00'))}")
        imgui.set_cursor_pos((185, 790+y_add)); imgui.text(f"{Decimal(probes[i][1]).quantize(Decimal('0.00'))}")
        imgui.set_cursor_pos((270, 790+y_add)); imgui.text(f"{Decimal(probes[i][2]).quantize(Decimal('0.00'))}")
        imgui.set_cursor_pos((365, 790+y_add)); imgui.text(f"{Decimal(og_probes[i]['confidence']).quantize(Decimal('0.00'))}")
               
def drone_image(image_path, texture_id, img_width, img_height):
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
    global position_x, position_y, yaw_velocity, current_item
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
            ament_index_python.packages.get_package_share_directory("asr_gcs"),
            "fonts", "source-code-pro"
        
        )
        image_dir = os.path.join(
            ament_index_python.packages.get_package_share_directory("asr_gcs"),
            "images"
        )
        script_dir = os.path.dirname(os.path.abspath(__file__))

        # Go up 3 levels to reach ~/drone-software
        project_root = os.path.expanduser("~/drone-software")

        # Folder for probe data
        log_dir = os.path.join(project_root, "probe_data")
        os.makedirs(log_dir, exist_ok=True)
        # Combine directory and timestamped filename directly
        filename = os.path.join(log_dir, f"probe_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
        print(filename)  # This now includes the full path

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
        start_x, start_y = 405, 0  # Starting point of the line (x, y)
        end_x, end_y = 405, 2000      # Ending point of the line (x, y)
        color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5) 
        draw_list.add_line(start_x,start_y, end_x, end_y, color, 5.0)
        #Running different widgets
        
        drone_visualization()
        grid()
        Arm_Button(node)
        Kill_command(node)
        Goto_field(node)
        speed_field(node)
        XYZ_Text_Field(msg=drone_data)
        RPY_Text_Field()
        gps_status()
        slider_field(node)
        probe_Field(node,filename)
        XYZVelocity_Text_Field()
        batteryGraph()
        motor_speed()
        return_to_home_button(node)

        GuiConsoleLogger(node)
        GUIButton.button1(841, 635, font_small, "Land", "land", node)
        GUIButton.button2(457, 635, font_small, "Takeoff", "takeoff", node, -1.0)
        GUIButton.button1(1033, 635, font_small, "Set Origin", "set_origin", node)
  
        manual(node) 
        send_map_pos(node)
        route_planner(node)
        execute_route(node)
        plan_card.goto_card(1253, 190, node, flight_plan[0], flight_plan_coord[0], 0)
        plan_card.goto_card(1253, 242, node, flight_plan[1], flight_plan_coord[1], 1)
        plan_card.goto_card(1253, 294, node, flight_plan[2], flight_plan_coord[2], 2)
        plan_card.goto_card(1253, 346, node, flight_plan[3], flight_plan_coord[3], 3)
        plan_card.goto_card(1253, 398, node, flight_plan[4], flight_plan_coord[4], 4)
        plan_card.goto_card(1253, 450, node, flight_plan[5], flight_plan_coord[5], 5)
        plan_card.goto_card(1253, 502, node, flight_plan[6], flight_plan_coord[6], 6)
        effect_class.arrow_effect(execute_route_numb)
     

        #Todo move to function
    
        dot_position_x = map_value(position_x, 10, -10, 432, 1193)  # Adjusted for 35px shift
        dot_position_y = map_value(position_y, 10, -10, 158, 599)

        if 420 < dot_position_x < 1215 and 165 < dot_position_y < 610:  
            if texture_id:
                img = Image.open(image_path).convert("RGBA")    

              # yaw_velocity is now in radians (-π to π)

                # Dead zone (~0.5 degrees ≈ 0.0087 radians)
                if abs(yaw_velocity) < 0.0087:
                    angle = 0
                else:
                    angle = math.degrees(yaw_velocity)  # convert radians → degrees
                
                rotated_img = img.rotate(angle, expand=False, center=(img.width/2, img.height/2))
                pixels = rotated_img.tobytes()
                gl.glBindTexture(gl.GL_TEXTURE_2D, texture_id)
                gl.glTexImage2D(
                    gl.GL_TEXTURE_2D, 0, gl.GL_RGBA, img_width, img_height, 0,
                    gl.GL_RGBA, gl.GL_UNSIGNED_BYTE, pixels
                )
                gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
                imgui.set_cursor_pos((dot_position_x-50, dot_position_y-50))  # Position the image (centered)
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
    


    
