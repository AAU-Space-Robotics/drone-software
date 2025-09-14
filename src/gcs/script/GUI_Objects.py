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


class info_field:
    def XYZ_Text_Field(position_x, position_y, position_z, target_position_x, target_position_y, target_position_z, position_timestamp, font_small,scale):
             
        info_field.squares.colored_square(20,90,420,235,scale)

        with imgui.font(font_small):
            imgui.set_cursor_pos((23*scale, 50*scale)); imgui.text("Positon:| Current:| Target: ")
            imgui.set_cursor_pos((60*scale, 95*scale)); imgui.text("X = ")
            imgui.set_cursor_pos((60*scale, 145*scale)); imgui.text("Y = ")
            imgui.set_cursor_pos((60*scale, 195*scale)); imgui.text("Z = ")
            imgui.set_cursor_pos((170*scale, 93*scale)); imgui.text(f"{Decimal(position_x).quantize(Decimal('0.000'))}")
            imgui.set_cursor_pos((170*scale, 143*scale)); imgui.text(f"{Decimal(position_y).quantize(Decimal('0.000'))}")
            imgui.set_cursor_pos((170*scale, 193*scale)); imgui.text(f"{(Decimal(position_z).quantize(Decimal('0.000')))}")

            imgui.set_cursor_pos((313*scale, 93*scale)); imgui.text(f"{Decimal(target_position_x).quantize(Decimal('0.000'))}")
            imgui.set_cursor_pos((313*scale, 143*scale)); imgui.text(f"{Decimal(target_position_y).quantize(Decimal('0.000'))}")
            imgui.set_cursor_pos((313*scale, 193*scale)); imgui.text(f"{(Decimal(target_position_z).quantize(Decimal('0.000')))}")
            imgui.set_cursor_pos((23*scale, 240*scale)); imgui.text(f"[m]")
  
            draw_list = imgui.get_window_draw_list()
            start_x, start_y = 145*scale, 90*scale  # Starting point of the line (x, y)
            end_x, end_y = 145*scale, 235*scale      # Ending point of the line (x, y)
            color = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 1.0) 
            draw_list.add_line(start_x, start_y, end_x, end_y, color, 5.0*scale)

            draw_list = imgui.get_window_draw_list()
            start_x, start_y = 287*scale, 90*scale  # Starting point of the line (x, y)
            end_x, end_y = 287*scale, 235*scale      # Ending point of the line (x, y)
            color = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 1.0) 
            draw_list.add_line(start_x, start_y, end_x, end_y, color, 5.0*scale)
        imgui.set_cursor_pos((103*scale, 250*scale)); imgui.text(f" TS: {position_timestamp}")
    def XYZVelocity_Text_Field(velocity_x, velocity_y, velocity_z, velocity_timestamp, font_small,scale):
        
        info_field.squares.colored_square(20,335,250,480,scale)

        with imgui.font(font_small):
            imgui.set_cursor_pos((23*scale, 295*scale)); imgui.text("Velocity:")
            imgui.set_cursor_pos((60*scale, 340*scale)); imgui.text("X   = ")
            imgui.set_cursor_pos((150*scale, 338*scale)); imgui.text(f"{Decimal(velocity_x).quantize(Decimal('0.000'))}")
            imgui.set_cursor_pos((60*scale, 390*scale)); imgui.text("Y   = ")
            imgui.set_cursor_pos((150*scale, 388*scale)); imgui.text(f"{Decimal(velocity_y).quantize(Decimal('0.000'))}")
            imgui.set_cursor_pos((60*scale, 440*scale)); imgui.text("Z   = ")
            imgui.set_cursor_pos((150*scale, 438*scale)); imgui.text(f"{-(Decimal(velocity_z).quantize(Decimal('0.000')))}")
            imgui.set_cursor_pos((23*scale, 485*scale)); imgui.text(f"[m/s]")
        imgui.set_cursor_pos((103*scale, 495*scale)); imgui.text(f" TS: {velocity_timestamp}")
    def RPY_Text_Field(roll, pitch, yaw_velocity, font_small,scale):
        
            #draw_list.add_rect_filled(20, 530, 250, 675, color, rounding=10.0, flags=10)
            info_field.squares.colored_square(20,580,250,725,scale)

            with imgui.font(font_small):
                imgui.set_cursor_pos((23*scale, 540*scale)); imgui.text("Orientation:")
                imgui.set_cursor_pos((30*scale, 590*scale)); imgui.text("Roll  = ")
                imgui.set_cursor_pos((150*scale, 588*scale)); imgui.text(f"{Decimal(roll).quantize(Decimal('0.00'))}")
                imgui.set_cursor_pos((30*scale, 640*scale)); imgui.text("Pitch = ")
                imgui.set_cursor_pos((150*scale, 638*scale)); imgui.text(f"{Decimal(pitch).quantize(Decimal('0.00'))}")
                imgui.set_cursor_pos((30*scale, 690*scale)); imgui.text("Yaw   = ")
                imgui.set_cursor_pos((150*scale, 688*scale)); imgui.text(f"{Decimal(yaw_velocity).quantize(Decimal('0.00'))}")
                imgui.set_cursor_pos((23*scale, 730*scale)); imgui.text(f"[r]")
    
    def seperation_line(scale):

        draw_list = imgui.get_window_draw_list()
        x_start, y_start = 435, 0  # Starting point of the line (x, y)
        end_x, end_y = 440, 2000      # Ending point of the line (x, y)
        color_TL = imgui.get_color_u32_rgba(0.0, 1.0, 1.0, 0.8) 
        color_TR = imgui.get_color_u32_rgba(0.0, 1.0, 1.0, 0.8) 
        color_BR = imgui.get_color_u32_rgba(0.0, 0.0, 1.0, 0.8)
        color_BL = imgui.get_color_u32_rgba(0.0, 0.0, 1.0, 0.8)
        draw_list.add_rect_filled_multicolor(x_start*scale, y_start*scale,end_x*scale,end_y*scale,color_TL, color_TR, color_BR, color_BL)


        x_start, y_start = 440, 90
        end_x, end_y = 1100, 95
        
        draw_list.add_rect_filled_multicolor(x_start*scale, y_start*scale,end_x*scale,end_y*scale,color_TL, color_BL, color_BR,color_TR )

    class squares:
        def colored_square(x,y,x_end,y_end,scale):
            draw_list = imgui.get_window_draw_list()
            color_behind = imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 1)
            color_TL = imgui.get_color_u32_rgba(0.0, 0.8, 1.0, 0.8) 
            color_TR = imgui.get_color_u32_rgba(0.0, 0.5, 1.0, 0.8) 
            color_BR = imgui.get_color_u32_rgba(0.0, 0.2, 1.0, 0.8)
            color_BL = imgui.get_color_u32_rgba(0.0, 0.0, 1.0, 0.9)
            
            flags = 10
            draw_list.add_rect_filled(x*scale, y*scale,x_end*scale+8,y_end*scale, color_behind, rounding=10*scale, flags=flags)
            draw_list.add_rect_filled_multicolor(x*scale, y*scale,x_end*scale,y_end*scale, color_TL, color_TR, color_BR, color_BL)
class buttons:
    def estop_button(font_large,scale,drone_kill):
        
        estop_color = (0.8,0.0,0.0,1)
        imgui.set_cursor_pos((1110*scale, 30*scale))  
        imgui.push_style_var(imgui.STYLE_FRAME_ROUNDING, 12.0)
        imgui.push_style_color(imgui.COLOR_BUTTON, *estop_color)
        imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, 1.0, 0.0, 0.0, 1)  
        imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, 0.2, 0.0, 0.0, 1)   

        with imgui.font(font_large):
            if imgui.button("E-stop", width=150*scale, height=100*scale):
            #    #node.send_command("estop")
                drone_kill = True
        imgui.pop_style_color(3)
        imgui.pop_style_var()
        draw_list = imgui.get_window_draw_list()
        ##for i in range(0,15):
        ## More compact gradient effect in the y direction
        ## First gradient: increasing alpha from top to bottom
        #for i, alpha in enumerate([0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.5,0.6,0.7,0.8,0.9,1.0]):
        #    y_offset = 37 + i * 1
        #    draw_list.add_rect_filled(1110*scale, y_offset*scale, 1260*scale, (y_offset+2)*scale, imgui.get_color_u32_rgba(1.0, 1.0, 1.0, alpha))
        ## Second gradient: decreasing alpha from where the first stops
        #start_y = 37 + len([0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.5,0.6,0.7,0.8,0.9,1.0]) * 1
        #for i, alpha in enumerate(reversed([0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.5,0.6,0.7,0.8,0.9,1.0])):
        #    y_offset = start_y + i * 1
        #    draw_list.add_rect_filled(1110*scale, y_offset*scale, 1260*scale, (y_offset+2)*scale, imgui.get_color_u32_rgba(1.0, 1.0, 1.0, alpha))
        #    TO BE CONTINUED
           
        return drone_kill
    def button_circle(font, scale):
        draw_list = imgui.get_window_draw_list()
        
        circle_center_x = 1200*scale
        circle_center_y = 90*scale
        circle_radius = 80*scale
        
        label = "E-STOP"

        bbox_min_x = circle_center_x - circle_radius
        bbox_min_y = circle_center_y - circle_radius
        bbox_max_x = circle_center_x + circle_radius
        bbox_max_y = circle_center_y + circle_radius
        is_hovered = imgui.is_mouse_hovering_rect(bbox_min_x*scale, bbox_min_y*scale, bbox_max_x*scale, bbox_max_y*scale)
        is_active = is_hovered and imgui.is_mouse_down(0)
        white_color = (0.6, 0.2, 0.2, 0.3)
        estop_color = (1.0, 0.0, 0.0, 1.0)  # Normal
        hovered_color = (1.0, 0.5, 0.5, 1.0)  # Hovered
        active_color = (1.0, 0.0, 0.0, 1.0)  # Active
        base_color = (
            active_color if is_active else
            hovered_color if is_hovered else
            estop_color
            )
        num_segments = 50
        for i in range(num_segments):
            t = i / (num_segments - 1)  # Interpolation factor (0=center, 1=edge)
            r = white_color[0] + (base_color[0] - white_color[0]) * t
            g = white_color[1] + (base_color[1] - white_color[1]) * t
            b = white_color[2] + (base_color[2] - white_color[2]) * t
            a = white_color[3] + (base_color[3] - white_color[3]) * t
            radius = circle_radius * (1.0 - t)  # Decrease radius toward center
            draw_list.add_circle_filled(circle_center_x*scale, circle_center_y*scale,radius,imgui.get_color_u32_rgba(r, g, b, a),)
        draw_list.add_circle(circle_center_x*scale, circle_center_y*scale,circle_radius,imgui.get_color_u32_rgba(1.0, 1.0, 1.0, 0.5),64*scale,3.0*scale)
        imgui.set_cursor_pos(((circle_center_x-60) *scale, (circle_center_y-25)*scale))

        with imgui.font(font ):
            imgui.text(label)
    
