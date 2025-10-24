#!/usr/bin/env python3

import time
from trilobot import Trilobot
import serial

print("Trilobot Example: Remote ESP-NOW Control")
print("Listening for commands from ESP32...")

# --- CONFIGURE THIS ---
# Find your port with: ls /dev/ttyUSB* or ls /dev/ttyACM*
SERIAL_PORT = '/dev/ttyUSB0'  # Change this to your Pi's ESP32 port
BAUD_RATE = 115200
# --------------------

# --- TUNE THIS ---
# You may need to adjust this, it's the distance between your robot's wheels in meters
WHEEL_BASE = 0.1  # Example: 10cm
# -----------------

tbot = Trilobot()

try:
    board = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to ESP32 on {SERIAL_PORT}.")
    
    while True:
        # Read a line from the ESP32
        line = board.readline()
        
        if line:
            try:
                # 1. Decode and strip whitespace
                data_str = line.decode('utf-8').strip()
                
                # 2. Parse the string: "linear.x,angular.z"
                parts = data_str.split(',')
                if len(parts) == 2:
                    linear_x = float(parts[0])
                    angular_z = float(parts[1])
                    
                    # 3. Calculate motor speeds (same as the ROS 2 driver)
                    # Clamping speed between -1.0 and 1.0
                    right_speed = max(min(linear_x + (angular_z * WHEEL_BASE / 2), 1.0), -1.0)
                    left_speed = max(min(linear_x - (angular_z * WHEEL_BASE / 2), 1.0), -1.0)

                    # 4. Send speeds to the motors
                    tbot.set_left_speed(left_speed)
                    tbot.set_right_speed(right_speed)
                    
                    print(f"In: (lin={linear_x:.2f}, ang={angular_z:.2f}) -> Out: (L={left_speed:.2f}, R={right_speed:.2f})")

            except Exception as e:
                print(f"Error parsing data: '{data_str}'. Error: {e}")

except serial.SerialException as e:
    print(f"Error: Could not open port {SERIAL_PORT}.")
except KeyboardInterrupt:
    print("\nProgram stopped.")
finally:
    # On exit, make sure to stop the motors
    print("Stopping motors...")
    tbot.stop()
    if 'board' in locals() and board.is_open:
        board.close()
    print("Serial port closed. Exiting.")