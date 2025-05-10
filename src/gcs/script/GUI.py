#!/usr/bin/env python3

import glfw
import imgui
from imgui.integrations.glfw import GlfwRenderer
import OpenGL.GL as gl
from PIL import Image
from OpenGL.GL import *


import rclpy
from rclpy.node import Node
from threading import Thread
from interfaces.msg import DroneState 
from dataclasses import dataclass
from decimal import *

class DroneData:
    position: list = (0.0, 0.0, 0.0)
    velocity: list = (0.0, 0.0, 0.0)
    orientation: list = (0.0, 0.0, 0.0)
    battery_voltage: float = 0.0

drone_data = DroneData()

button_color = (0.0,0.5,0.0)
killbutton_color = (0.8,0.0,0.0)
text_buffer = ""
current_item = 0
position_x, position_y, position_z = 0, 0, 0
target_position_x, target_position_y, target_position_z = 0, 0, 0
roll, pitch, yaw = 0, 0, 0
velocity_x, velocity_y, velocity_z = 0, 0, 0
thrust = 0
drone_kill = False
test_slider = 0
battery_state_timestamp = 0
position_timestamp = 0
battery_voltage, battery_current, battery_percentage = 0.0, 0.0, 0.0
battery_discharge_rate, battery_average_current = 0.0, 0.0

drone_state = False




class DroneGuiNode(Node):
    def __init__(self):
        super().__init__('thyra_gui_node')
        self.subscription = self.create_subscription(
            DroneState,
            "drone/out/drone_state",
            self.state_callback,
            10
        )

    def state_callback(self, msg):
        global position_x, position_y, position_z, position_timestamp
        global target_position_x, target_position_y, target_position_z
        global roll, pitch, yaw
        global velocity_x, velocity_y, velocity_z
        global battery_voltage, battery_state_timestamp, battery_current, battery_percentage, battery_discharge_rate, battery_average_current
        position_timestamp = msg.position_timestamp
        #print(f"Position timestamp: {position_timestamp}")
        if len(msg.position) >= 3:
            position_x = msg.position[0]
            position_y = msg.position[1]
            position_z = msg.position[2]
            #print(f"Position: {position_x}, {position_y}, {position_z}")
            #print(Decimal(position_x).quantize(Decimal('0.00')))
        if len(msg.velocity) >= 3:
            velocity_x = msg.velocity[0]
            velocity_y = msg.velocity[1]
            velocity_z = msg.velocity[2]
        #print(f"Velocity: {velocity_x}, {velocity_y}, {velocity_z}")
        if len(msg.orientation) >= 3:
            roll = msg.orientation[0]
            pitch = msg.orientation[1]
            yaw = msg.orientation[2]
        
        if len(msg.target_position) >= 3:
            target_position_x = msg.target_position[0]
            target_position_y = msg.target_position[1]
            target_position_z = msg.target_position[2]
        #print(f"Target Position: {target_position_x}, {target_position_y}, {target_position_z}")
        battery_state_timestamp = msg.battery_state_timestamp
        
        battery_voltage = msg.battery_voltage
        
        battery_current = msg.battery_current
        
        battery_percentage = msg.battery_percentage
        
        battery_discharge_rate = msg.battery_discharged_mah
        #print(f'Battery discharge rate: {battery_discharge_rate}')
        battery_average_current = msg.battery_average_current
        

def Arm_Button():
        global button_color, drone_kill
        global font
        global drone_state
        # Change button color on press
        drone_kill = Kill_command()
        #button_color = (0.0,0.5,0.0)   if Kill_command() else (1.0,0.0,0.0)
         
        imgui.set_cursor_pos((450,30))
        button_color = (0.0, 0.5, 0.0) if drone_kill else (1.0, 0.0, 0.0)#here
        button_text = "Press to Arm!  "  if drone_kill else "Press to Disarm"

        imgui.push_style_var(imgui.STYLE_FRAME_ROUNDING, 12.0)
        imgui.push_style_color(imgui.COLOR_BUTTON, *button_color)
        imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, *button_color)  # Match hover color to button color
        imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, *button_color)  # Match active color to button color
       
        with imgui.font(font_small):
            if imgui.button(button_text):
                if drone_kill:
                    # If armed, allow disarming
                    # Replace this with your own disarm function
                    drone_kill = False
                    drone_state = True  # Or call a function that disarms it
                else:
                    # If disarmed, try to arm (if allowed)
                    # If arming not allowed, keep it disarmed
                    if not Kill_command():  # If Kill_command still returns False, stay disarmed
                        drone_kill = True  # Or call a function that arms it
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
        start_x, start_y = 450, 70  # Starting point of the line (x, y)
        end_x, end_y = 1100, 70      # Ending point of the line (x, y)
        color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5) 
        draw_list.add_line(start_x,start_y, end_x, end_y, color, 5.0)
       
   
       
        
def Kill_command(): 
    global killbutton_color, drone_kill
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.8, 0.0, 0.0, 1.0) 
                                                                #flags is for rounding different corners
    #draw_list.add_rect_filled(1200,27, 1350,80,color,rounding =10.0, flags=15)
    imgui.set_cursor_pos((1150,30))

    imgui.push_style_var(imgui.STYLE_FRAME_ROUNDING, 12.0)
    imgui.push_style_color(imgui.COLOR_BUTTON, *killbutton_color)
    imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, *(1.0,0.0,0.0))
    imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, *(0.2,0.0,0.0)) 
    with imgui.font(font_large):
        if imgui.button("Kill",width=150, height=100):
            drone_kill = True
    imgui.pop_style_color(3)
    imgui.pop_style_var()

    if(killbutton_color == (1.0,0.0,0.0)):
        color = imgui.get_color_u32_rgba(1.0, 0.0, 0.0, 1.0) 
    else:
        color = imgui.get_color_u32_rgba(0.8, 0.0, 0.0, 1.0) 

    if drone_kill == True:
        return True
    else:
        return False


def Text_field():

    global text_buffer
    text_field = ""
    #changed = False
    
    imgui.set_cursor_pos((450,670))
    with imgui.font(font):
        imgui.text("Input field")
    imgui.set_cursor_pos((450,715))
    imgui.set_next_item_width(300)
    imgui.set_window_font_scale(2.0) 
    changed, text_field= imgui.input_text("", text_buffer, 64)
    if changed:
        text_buffer = text_field
    imgui.set_window_font_scale(1.0)
    imgui.set_cursor_pos((450,750))
    with imgui.font(font):
        imgui.text(f"You typed: {text_buffer}")

    imgui.set_cursor_pos((770,705))
    imgui.push_style_var(imgui.STYLE_FRAME_ROUNDING, 12.0)
    imgui.push_style_color(imgui.COLOR_BUTTON, *(0.0, 0.5, 0.0))
    imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, *(0.0,0.8,0.0))
    imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, *(0.0,0.2,0.0)) 
    with imgui.font(font_small):
        if imgui.button("Add",width=70, height=50):
            drone_kill = True
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
        imgui.set_cursor_pos((170,193)); imgui.text(f"{-(Decimal(position_z).quantize(Decimal('0.000')))}")

        imgui.set_cursor_pos((313,93)); imgui.text(f"{Decimal(target_position_x).quantize(Decimal('0.000'))}")
        imgui.set_cursor_pos((313,143)); imgui.text(f"{Decimal(target_position_y).quantize(Decimal('0.000'))}")
        imgui.set_cursor_pos((313,193)); imgui.text(f"{-(Decimal(target_position_z).quantize(Decimal('0.000')))}")

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

    
def RPY_Text_Field():
    
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5) 
                                                                #flags is for rounding different corners
    draw_list.add_rect_filled(20,305, 250,450,color,rounding =10.0, flags=10)
    thrust = -(int(test_slider))

    with imgui.font(font_small):
        imgui.set_cursor_pos((23,265)); imgui.text("Orientation:")
    with imgui.font(font_small):
        # imgui.set_cursor_pos((30,255)); imgui.text("Thrust = ")
        imgui.set_cursor_pos((30,310)); imgui.text("Roll  = ")
        imgui.set_cursor_pos((30,365)); imgui.text("Pitch = ")
        imgui.set_cursor_pos((30,415)); imgui.text("Yaw   = ")
        #imgui.set_cursor_pos((150,255)); imgui.text(str(thrust))
        imgui.set_cursor_pos((150,308)); imgui.text(f"{Decimal(roll).quantize(Decimal('0.00'))}")
        imgui.set_cursor_pos((150,363)); imgui.text(f"{Decimal(pitch).quantize(Decimal('0.00'))}")
        imgui.set_cursor_pos((150,413)); imgui.text(f"{Decimal(yaw).quantize(Decimal('0.00'))}")



    
    
def XYZVelocity_Text_Field():
    
    global velocity_x, velocity_y, velocity_z  
    
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5) 
                                                                #flags is for rounding different corners
    draw_list.add_rect_filled(20,520, 250,665,color,rounding =10.0, flags=10)

    with imgui.font(font_small):
        imgui.set_cursor_pos((23,480)); imgui.text("Velosity:")
        imgui.push_style_color(imgui.COLOR_TEXT, 1.0, 0.0, 0.0, 0.9)
        imgui.set_cursor_pos((30,525)); imgui.text("X = ")
        imgui.set_cursor_pos((120,523)); imgui.text(f"{Decimal(velocity_x).quantize(Decimal('0.000'))}")
        imgui.pop_style_color()
        imgui.push_style_color(imgui.COLOR_TEXT, 0.0, 1.0, 0.0, 0.9)
        imgui.set_cursor_pos((30,575)); imgui.text("Y = ")
        imgui.set_cursor_pos((120,573)); imgui.text(f"{Decimal(velocity_y).quantize(Decimal('0.000'))}")
        imgui.pop_style_color()
        imgui.push_style_color(imgui.COLOR_TEXT, 0.0, 0.0, 0.5, 0.9)
        imgui.set_cursor_pos((30,625)); imgui.text("Z = ")
        imgui.set_cursor_pos((120,623)); imgui.text(f"{-(Decimal(velocity_z).quantize(Decimal('0.000')))}")
        imgui.pop_style_color()
        
        
    




def batteryGraph():
    global battery_voltage, battery_current, battery_percentage, battery_average_current
    battery_progressbar = map_value(battery_percentage, 0, 1, 109, 44)
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.8, 0.8, 0.8, 1.0) 
    draw_list.add_rect(1525,40,1565,110,color, rounding=1.0,flags=15,thickness=3)   
    draw_list.add_rect(1532,31,1557,38,color, rounding=1.0,flags=3,thickness=3)  
    if(battery_percentage > 0.5):
        battery_color = imgui.get_color_u32_rgba(0.0, 1.0, 0.0, 1.0)
        
    elif(battery_percentage > 0.25):
        battery_color = imgui.get_color_u32_rgba(1.0, 1.0, 0.0, 1.0)
    else:
        battery_color = imgui.get_color_u32_rgba(1.0, 0.0, 0.0, 1.0)
    imgui.set_cursor_pos((1524, 116)); imgui.text(f"{Decimal(100*battery_percentage).quantize(Decimal('0.00'))} %")
    #draw_list.add_rect_filled(1465,31,1432+(battery_percentage*25),38,color, rounding=1.0,flags=3)
    #print(battery_voltage)
    draw_list.add_rect_filled(1528,106,1562,(battery_progressbar),battery_color, rounding=1.0,flags=15)

    imgui.set_cursor_pos((1320, 30)); imgui.text(f"Voltage:          {Decimal(battery_voltage).quantize(Decimal('0.0'))} V")
    imgui.set_cursor_pos((1320, 60)); imgui.text(f"Current:          {-(Decimal(battery_current).quantize(Decimal('0.0')))} A")
    imgui.set_cursor_pos((1320, 90)); imgui.text(f"Discharge rate:   {Decimal(battery_discharge_rate).quantize(Decimal('0.0'))} mAh")
    imgui.set_cursor_pos((1320, 120)); imgui.text(f"Average current:  {-(Decimal(battery_average_current).quantize(Decimal('0.0')))} A")

    draw_list = imgui.get_window_draw_list()
    start_x, start_y = 1510, 30  # Starting point of the line (x, y)
    end_x, end_y = 1510, 132      # Ending point of the line (x, y)
    color = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 1.0) 
    draw_list.add_line(start_x,start_y, end_x, end_y, color, 2.0)

    draw_list = imgui.get_window_draw_list()
    start_x, start_y = 1439, 30  # Starting point of the line (x, y)
    end_x, end_y = 1439, 132      # Ending point of the line (x, y)
    color = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 1.0) 
    draw_list.add_line(start_x,start_y, end_x, end_y, color, 2.0)

    draw_list = imgui.get_window_draw_list()
    start_x, start_y = 1310, 30  # Starting point of the line (x, y)
    end_x, end_y = 1310, 132      # Ending point of the line (x, y)
    color = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 1.0) 
    draw_list.add_line(start_x,start_y, end_x, end_y, color, 2.0)

    imgui.set_cursor_pos((1320, 150)); imgui.text(f"Timestamp {battery_state_timestamp}")

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
    changed, slider_value = imgui.slider_float("Scale Me", slider_value, 0.0, 100.0)

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
        draw_list.add_rect_filled(670, 325, 675, (-velocity_z * 10)+345, color, rounding=2.0)

    
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
    


def start_ros():
    rclpy.init()
    node = DroneGuiNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main():
    # Start ROS in background thread
    Thread(target=start_ros, daemon=True).start()

    # Initialize GLFW
    if not glfw.init():
        print("Could not initialize GLFW")
        return
    
    # Set window hints for borderless full-screen mode

    
    # Get primary monitor size
    #monitor = glfw.get_primary_monitor()
    #mode = glfw.get_video_mode(monitor)
    #width, height = mode.size.width, mode.size.height
    
    # Create borderless window
    window = glfw.create_window(1600, 800, "THYRA", None, None)
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
    #impl.process_inputs()
    io = imgui.get_io()
    global font,font_large, font_small
    font = io.fonts.add_font_from_file_ttf("/home/dksor/drone-software/src/gcs/fonts/source-code-pro/SourceCodePro-Black.otf", 40)  # <-- bigger font size
    font_large = io.fonts.add_font_from_file_ttf("/home/dksor/drone-software/src/gcs/fonts/source-code-pro/SourceCodePro-Black.otf", 50)  # <-- bigger font size
    font_small = io.fonts.add_font_from_file_ttf("/home/dksor/drone-software/src/gcs/fonts/source-code-pro/SourceCodePro-Black.otf", 30)  # <-- bigger font size
    impl.refresh_font_texture()


    texture_id = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, texture_id)
    
    # Set texture parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
    # Load image data (e.g., via PIL)
    image = Image.open("/home/dksor/drone-software/src/gcs/images/droneImage.png").convert("RGBA")
    image_data = image.tobytes()
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.width, image.height, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, image_data)
    glBindTexture(GL_TEXTURE_2D, 0)
    
    
    # Main loop
    while not glfw.window_should_close(window):
        glfw.poll_events()
        impl.process_inputs()
        clear_color = (0.0, 0.0, 0.12, 0.0)  # RGBA 
        gl.glClearColor(*clear_color)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)
        imgui.new_frame()
      
        # Create UI without windowed mode
        imgui.set_next_window_position(0, 0)
        imgui.set_next_window_size(1600, 800)
        imgui.begin("wtf", flags=imgui.WINDOW_NO_TITLE_BAR | imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE | imgui.WINDOW_NO_BACKGROUND)
        imgui.set_cursor_pos((20,10))
        with imgui.font(font_small):
            imgui.text("THYRA state monitor")
        #Creating a line for seperation
        draw_list = imgui.get_window_draw_list()
        start_x, start_y = 430, 0  # Starting point of the line (x, y)
        end_x, end_y = 430, 800      # Ending point of the line (x, y)
        color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5) 
        draw_list.add_line(start_x,start_y, end_x, end_y, color, 5.0)
        #Running different widgets
        drone_visualization()
        imgui.set_cursor_pos((550,320));imgui.image(texture_id, 250, 250)
        
        Arm_Button()
        Text_field()
        Dropdown_Menu()
        XYZ_Text_Field(msg=drone_data)
        RPY_Text_Field()
        XYZVelocity_Text_Field()
        
        batteryGraph()
        Arrows()

        imgui.end()

        
        # Render
        imgui.render()
        impl.render(imgui.get_draw_data())
        glfw.swap_buffers(window)
    
    # Cleanup
    impl.shutdown()
    glfw.terminate()

if __name__ == "__main__":
    main()
    


    