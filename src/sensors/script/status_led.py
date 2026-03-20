#!/usr/bin/env python3
import board
import neopixel
import time

try:
    # Adjust pin (D18 for GPIO18) and LED count
    pixels = neopixel.NeoPixel(board.D18, 8, auto_write=True)
    pixels.fill((0, 255, 0))  # Set all LEDs to green
    print("LEDs set to green")
    time.sleep(2)
    pixels.fill((0, 0, 0))  # Turn off
except Exception as e:
    print(f"Error: {e}")