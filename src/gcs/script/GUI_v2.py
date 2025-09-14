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
import sys
import threading
from threading import Thread
from dataclasses import dataclass
from decimal import Decimal
import os
import ament_index_python.packages
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

import GUI
import GUI_Objects
drone_kill = False
show_popup = True

def load_fonts(io, font_path, scale):
    """Load fonts with sizes scaled by the DPI factor."""
    io.fonts.clear()  # Clear existing fonts
    font = io.fonts.add_font_from_file_ttf(font_path, 40 * scale)
    font_large = io.fonts.add_font_from_file_ttf(font_path, 45 * scale)
    font_small = io.fonts.add_font_from_file_ttf(font_path, 30 * scale)
    font_for_meter = io.fonts.add_font_from_file_ttf(font_path, 20 * scale)

    return font, font_large, font_small, font_for_meter
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
   

def main(args=None):
    #imgui.set_cursor_pos((100, 30))
    #if imgui.button("Open Popup", 100, 40):
    #    show_popup = True
    #    imgui.open_popup("MyModalPopup")
    if not glfw.init():
        print("Could not initialize GLFW")
        return
    
    glfw.init()
    # To set the windows size in beginning
    monitor = glfw.get_primary_monitor()
    video_mode = glfw.get_video_mode(monitor)
    screen_width = video_mode.size.width
    screen_height = video_mode.size.height

    # Set window hints for no border
    glfw.window_hint(glfw.SCALE_TO_MONITOR, glfw.TRUE)
    glfw.window_hint(glfw.RESIZABLE, glfw.TRUE)
    glfw.window_hint(glfw.DECORATED, glfw.TRUE)
    glfw.window_hint(glfw.MAXIMIZED, glfw.TRUE)
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
    glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, gl.GL_TRUE)

    window = glfw.create_window(screen_width, screen_height, "THYRA", None, None)
    
    if not window:
        print("Could not create GLFW window")
        glfw.terminate()  
        return
    glfw.make_context_current(window)
    glfw.set_window_pos(window, 0, 0)
    
    def framebuffer_size_callback(window, width, height):
       gl.glViewport(0, 0, width, height)
       glfw.set_framebuffer_size_callback(window, framebuffer_size_callback)

    imgui.create_context()
    io = imgui.get_io()

    try:
        impl = GlfwRenderer(window)
    except Exception as e:
        print(f"Failed to initialize GlfwRenderer: {e}")
        glfw.terminate()
        sys.exit(1)

    x_sc, y_sc = glfw.get_window_size(window)
    scale = max(x_sc / screen_width, y_sc / screen_height)
    
    io.fonts.clear()
    io.fonts.add_font_default()
    io.font_global_scale = scale  # Simple scaling; can load custom fonts if needed
    impl.refresh_font_texture()
    
  
    try:
        font_dir = os.path.join(
            ament_index_python.packages.get_package_share_directory("gcs"),
            "fonts", "source-code-pro"
        
        )
        image_dir = os.path.join(
            ament_index_python.packages.get_package_share_directory("gcs"),
            "images"
        )
        font_path = os.path.join(font_dir, "SourceCodePro-Black.otf")
        image_path = os.path.join(image_dir, "droneImage.png")
        image_path_logo = os.path.join(image_dir, "AAU_Space_Robotics_Logo_Black.png")
    except Exception as e:
        print(f"Could not find gcs package share directory: {e}")
        glfw.terminate()
        return
    if not os.path.isfile(font_path):
        print(f"Font file not found: {font_path}")
        glfw.terminate()
        return
    if image_path and os.path.exists(image_path):
        texture_id, img_width, img_height = load_image_to_texture(image_path)
        texture_id_logo, img_width_logo, img_height_logo = load_image_to_texture(image_path_logo)
    else:
        print(f"Image file not found at: {image_path}")
        texture_id, img_width, img_height = None, 0, 0

    font, font_large, font_small, font_for_meter = load_fonts(io, font_path, scale)
    impl.refresh_font_texture()
    show_popup = True  
    start_time = time.time()
    elapsed_time = 0
    duration = 3
    # main loop
    background_color = (0.0, 0, 0.218, 1)
    #(0.0, 0.0, 0.12, 0.0)
    
    image_size = [200 * scale, 200 * scale]
    while not glfw.window_should_close(window):
        glfw.poll_events()
        impl.process_inputs()
        
    
        x_sc, y_sc = glfw.get_window_size(window)
        scale = max(x_sc / screen_width, y_sc / screen_height)
        io.font_global_scale = scale  # Update global scale if window size changes

        imgui.new_frame()
        if show_popup:
            imgui.open_popup(" ")
            
            imgui.set_next_window_position(0,0, 2)
            imgui.set_next_window_size(screen_width, screen_height)
           
             # RGBA values (0.0-1.0)
    
            # Push the background color style
            imgui.push_style_color(imgui.COLOR_POPUP_BACKGROUND, *background_color)
            if imgui.begin_popup_modal(" ",show_popup, imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE | imgui.WINDOW_NO_TITLE_BAR | imgui.WINDOW_NO_COLLAPSE)[0]:
                imgui.push_font(font_small)
                imgui.set_cursor_pos((((screen_width * 0.5)-(image_size[0]/2)) * scale, ((screen_height * 0.5)-(image_size[1]/2) -100) * scale))
                
                imgui.image(texture_id_logo, image_size[0], image_size[1])
                if elapsed_time < duration:
                    elapsed_time = time.time() - start_time
                    if elapsed_time < 1.5:
                        image_size[0] += elapsed_time * 2 * scale
                        image_size[1] += elapsed_time * 2 * scale
                    if elapsed_time >= 1.5:
                        image_size[0] -= elapsed_time * 1 * scale
                        image_size[1] -= elapsed_time * 1 * scale
                    
                

                else:
                    show_popup = False
      
               
                imgui.pop_font()
                imgui.end_popup()

            imgui.pop_style_color(1)
        imgui.set_next_window_position(0, 0)
        imgui.set_next_window_size(screen_width, screen_height)
        imgui.begin("wtf", flags=imgui.WINDOW_NO_TITLE_BAR | imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE | imgui.WINDOW_NO_BACKGROUND)
        imgui.set_cursor_pos((20, 10))
        imgui.push_font(font_small)
        imgui.text("")
        imgui.pop_font()

        # All main objects are called here
        if not show_popup:
            GUI_Objects.info_field.seperation_line(scale)
            GUI_Objects.info_field.XYZ_Text_Field(1.234, 2.345, 3.456, 4.567, 5.678, 6.789, "12:34:56", font_small, scale)
            GUI_Objects.info_field.XYZVelocity_Text_Field(0.123, 0.234, 0.345, "12:34:56", font_small, scale)
            GUI_Objects.info_field.RPY_Text_Field(0.12, 0.23, 0.34,font_small, scale)
            GUI_Objects.buttons.button_circle(font_large,scale)

        imgui.end()

         # Background color
        gl.glClearColor(*background_color)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        # Render
        
        imgui.render()
        impl.render(imgui.get_draw_data())
        glfw.swap_buffers(window)

    impl.shutdown()
    glfw.terminate()

if __name__ == "__main__":
    main()
    