#!/usr/bin/env python3

import glfw
import imgui
from imgui.integrations.glfw import GlfwRenderer
import OpenGL.GL as gl
from PIL import Image


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
roll, pitch, yaw = 0, 0, 0
velocity_x, velocity_y, velocity_z = 0, 0, 0
thrust = 0
drone_kill = False
test_slider = 0
battery_voltage = 0.0
drone_state = False

class DroneGuiNode(Node):
    def __init__(self):
        super().__init__('thyra_gui_node')
        self.subscription = self.create_subscription(
            DroneState,
            "drone/out/state",
            self.state_callback,
            10
        )

    def state_callback(self, msg):
        global position_x, position_y, position_z
        global roll, pitch, yaw
        global velocity_x, velocity_y, velocity_z
        global battery_voltage
        if len(msg.position) >= 3:
            position_x = msg.position[0]
            position_y = msg.position[1]
            position_z = msg.position[2]
            #print(f"Position: {position_x}, {position_y}, {position_z}")
            #print(Decimal(position_x).quantize(Decimal('0.00')))
        if len(msg.orientation) >= 3:
            roll = msg.orientation[0]
            pitch = msg.orientation[1]
            yaw = msg.orientation[2]
        if len(msg.velocity) >= 3:
            velocity_x = msg.velocity[0]
            velocity_y = msg.velocity[1]
            velocity_z = msg.velocity[2]
        battery_voltage = msg.battery_voltage
        #print(f"Battery Voltage: {battery_voltage}")

def Arm_Button():
        global button_color, drone_kill
        global font
        global drone_state
        # Change button color on press
        drone_kill = Kill_command()
        #button_color = (0.0,0.5,0.0)   if Kill_command() else (1.0,0.0,0.0)
         
        imgui.set_cursor_pos((500,30))
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
        imgui.set_cursor_pos((760, 30))
        with imgui.font(font):
            if drone_kill:

                    imgui.text("THYRA IS DISARMED!")
            else:

                    imgui.text("THYRA IS ARMED!")
       
   
       
        
def Kill_command(): 
    global killbutton_color, drone_kill
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.8, 0.0, 0.0, 1.0) 
                                                                #flags is for rounding different corners
    #draw_list.add_rect_filled(1200,27, 1350,80,color,rounding =10.0, flags=15)
    imgui.set_cursor_pos((1227,30))

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
    imgui.set_window_font_scale(1.0) 
    imgui.set_cursor_pos((20,670))
    with imgui.font(font):
        imgui.text("Input field")
    imgui.set_cursor_pos((20,715))
    imgui.set_next_item_width(300)
    imgui.set_window_font_scale(2.0) 
    changed, text_field= imgui.input_text("", text_buffer, 64)
    if changed:
        text_buffer = text_field
    imgui.set_window_font_scale(1.0)
    imgui.set_cursor_pos((20,750))
    with imgui.font(font):
        imgui.text(f"You typed: {text_buffer}")

    imgui.set_cursor_pos((330,705))
    imgui.push_style_var(imgui.STYLE_FRAME_ROUNDING, 12.0)
    imgui.push_style_color(imgui.COLOR_BUTTON, *killbutton_color)
    imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, *(1.0,0.0,0.0))
    imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, *(0.2,0.0,0.0)) 
    with imgui.font(font_small):
        if imgui.button("Add",width=70, height=50):
            drone_kill = True
    imgui.pop_style_color(3)
    imgui.pop_style_var()
 


def Dropdown_Menu():
    global current_item

    items = ["No Controller", "PS4", "TX16S"]
    imgui.set_cursor_pos((500,630))
    imgui.set_next_item_width(300)

    with imgui.font(font_small):
        changed, current_item = imgui.combo(
            "Controller", current_item, items)
    #imgui.set_cursor_pos((10,570))
    #imgui.text(f"{str(items[current_item])}")


def XYZ_Text_Field(msg):
    #Drawing a square kek
    global position_x, position_y, position_z
    #position_x, position_y, position_z = drone_data.position
    #position_x = int(msg.position[0])
    #print(f"Position: {position_x}, {position_y}, {position_z}")
   
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5) 
                                                                #flags is for rounding different corners
    draw_list.add_rect_filled(20,90, 250,235,color,rounding =10.0, flags=10)
    
    with imgui.font(font_small):
        imgui.set_cursor_pos((23,50)); imgui.text("Positon")
    with imgui.font(font):
        imgui.set_cursor_pos((30,90)); imgui.text("X = ")
        imgui.set_cursor_pos((30,140)); imgui.text("Y = ")
        imgui.set_cursor_pos((30,190)); imgui.text("Z = ")
    with imgui.font(font_large):
        imgui.set_cursor_pos((120,83)); imgui.text(f"{int(position_x)}")
        imgui.set_cursor_pos((120,133)); imgui.text(f"{int(position_y)}")
        imgui.set_cursor_pos((120,183)); imgui.text(f"{int(position_z)}")

    
def RPY_Text_Field():
    
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5) 
                                                                #flags is for rounding different corners
    draw_list.add_rect_filled(20,305, 250,430,color,rounding =10.0, flags=10)
    thrust = -(int(test_slider))

    with imgui.font(font_small):
        imgui.set_cursor_pos((23,265)); imgui.text("Orientation")
    with imgui.font(font_small):
        # imgui.set_cursor_pos((30,255)); imgui.text("Thrust = ")
        imgui.set_cursor_pos((30,305)); imgui.text("Roll  = ")
        imgui.set_cursor_pos((30,355)); imgui.text("Pitch = ")
        imgui.set_cursor_pos((30,395)); imgui.text("Yaw   = ")
    with imgui.font(font_small):
        #imgui.set_cursor_pos((150,255)); imgui.text(str(thrust))
        imgui.set_cursor_pos((150,305)); imgui.text(f"{Decimal(roll).quantize(Decimal('0.00'))}")
        imgui.set_cursor_pos((150,355)); imgui.text(f"{Decimal(pitch).quantize(Decimal('0.00'))}")
        imgui.set_cursor_pos((150,395)); imgui.text(f"{Decimal(yaw).quantize(Decimal('0.00'))}")



    
    
def XYZVelocity_Text_Field():
    
    global position_x, position_y, position_z
    
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5) 
                                                                #flags is for rounding different corners
    draw_list.add_rect_filled(20,500, 250,645,color,rounding =10.0, flags=10)

    with imgui.font(font_small):
        imgui.set_cursor_pos((23,460)); imgui.text("Velosity")
    with imgui.font(font):
        imgui.set_cursor_pos((30,500)); imgui.text("X = ")
        imgui.set_cursor_pos((30,550)); imgui.text("Y = ")
        imgui.set_cursor_pos((30,600)); imgui.text("Z = ")

    with imgui.font(font_large):
        imgui.set_cursor_pos((120,493)); imgui.text(f"{int(velocity_x)}")
        imgui.set_cursor_pos((120,543)); imgui.text(f"{int(velocity_y)}")
        imgui.set_cursor_pos((120,593)); imgui.text(f"{int(velocity_z)}")
    




def batteryGraph():
    global battery_voltage
    battery_procentage = battery_voltage * 100 / 12.0
    battery_progressbar = map_value(battery_voltage, 0, 12.0, 109, 44)
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.8, 0.8, 0.8, 1.0) 
    draw_list.add_rect(1425,40,1465,110,color, rounding=1.0,flags=15,thickness=3)   
    draw_list.add_rect(1432,31,1457,38,color, rounding=1.0,flags=3,thickness=3)  
    if(battery_procentage > 50):
        battery_color = imgui.get_color_u32_rgba(0.0, 1.0, 0.0, 1.0)
        
    elif(battery_procentage > 25):
        battery_color = imgui.get_color_u32_rgba(1.0, 1.0, 0.0, 1.0)
    else:
        battery_color = imgui.get_color_u32_rgba(1.0, 0.0, 0.0, 1.0)
    imgui.set_cursor_pos((1434, 116)); imgui.text(f"{int(battery_procentage)} %")
    #draw_list.add_rect_filled(1465,31,1432+(battery_procentage*25),38,color, rounding=1.0,flags=3)
    #print(battery_voltage)
    draw_list.add_rect_filled(1428,106,1462,(battery_progressbar),battery_color, rounding=1.0,flags=15)

def map_value(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def drone_visualization(path):
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5) 
                                                                #flags is for rounding different corners
    draw_list.add_rect(490,150, 1200,600,color,rounding =10.0, flags=15,thickness=6)
    draw_list = imgui.get_window_draw_list()
    color = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 0.9) 
                                                                #flags is for rounding different corners
    draw_list.add_rect_filled(493,153, 1197,597,color,rounding =10.0, flags=15)

    image = Image.open(path).convert("RGBA")
    image_data = image.tobytes()

    width, height = image.size
    texture_id = gl.glGenTextures(1)
    gl.glBindTexture(gl.GL_TEXTURE_2D, texture_id)

    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)

    gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, gl.GL_RGBA, width, height, 0, gl.GL_RGBA, gl.GL_UNSIGNED_BYTE, image_data)
    

    return texture_id, width, height




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
    window = glfw.create_window(1500, 800, "THYRA", None, None)
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
    font = io.fonts.add_font_from_file_ttf("/home/dksor/drone-software/Fonts/source-code-pro/SourceCodePro-Black.otf", 40)  # <-- bigger font size
    font_large = io.fonts.add_font_from_file_ttf("/home/dksor/drone-software/Fonts/source-code-pro/SourceCodePro-Black.otf", 50)  # <-- bigger font size
    font_small = io.fonts.add_font_from_file_ttf("/home/dksor/drone-software/Fonts/source-code-pro/SourceCodePro-Black.otf", 30)  # <-- bigger font size
    impl.refresh_font_texture()
    
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
        imgui.set_next_window_size(1500, 800)
        imgui.begin("wtf", flags=imgui.WINDOW_NO_TITLE_BAR | imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE | imgui.WINDOW_NO_BACKGROUND)
        imgui.set_cursor_pos((20,10))
        with imgui.font(font_small):
            imgui.text("THYRA state monitor")
        #Creating a line for seperation
        draw_list = imgui.get_window_draw_list()
        start_x, start_y = 460, 0  # Starting point of the line (x, y)
        end_x, end_y = 460, 800      # Ending point of the line (x, y)
        color = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.5) 
        draw_list.add_line(start_x,start_y, end_x, end_y, color, 5.0)
        #Running different widgets
        texture_id, w, h = drone_visualization("droneImage.png")
        
        Arm_Button()
        Text_field()
        Dropdown_Menu()
        XYZ_Text_Field(msg=drone_data)
        RPY_Text_Field()
        XYZVelocity_Text_Field()
        imgui.set_cursor_pos((700, 230)); imgui.image(texture_id, 300, 300)
        batteryGraph()
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


    