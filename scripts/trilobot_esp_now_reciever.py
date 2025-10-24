#!/usr/bin/env python3

import time
from trilobot import Trilobot
import serial

print("Trilobot Example: Remote ESP-NOW Control\n")
print("Listening for commands from ESP32...")

# --- CONFIGURE THIS ---
# You need to find the ESP32's port on your Raspberry Pi.
# Plug it in, then run 'ls /dev/tty*' in a terminal.
# It will probably be /dev/ttyUSB0 or /dev/ttyACM0.
SERIAL_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 115200
# --------------------

tbot = Trilobot()
speed = 0.5
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
WHITE = (255, 255, 255)

try:
    board = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    
    while True:
        # Read a line from the ESP32
        line = board.readline()
        
        if line:
            # Decode from bytes, remove newline characters, get first char
            try:
                char = line.decode('utf-8').strip()[0]
            except IndexError:
                continue # Skip if we got an empty line

            if (char == "p"):
                print("Exit command received")
                tbot.stop()
                tbot.clear_underlighting()
                break # Exit the while loop
 
            elif (char == "a"):
                print("Received: Left")
                tbot.turn_left(speed)
                tbot.fill_underlighting(BLUE)
 
            elif (char == "d"):
                print("Received: Right")
                tbot.turn_right(speed)
                tbot.fill_underlighting(GREEN)
 
            elif (char == "w"):
                print("Received: Forward")
                tbot.forward(speed)
                tbot.fill_underlighting(WHITE)
 
            elif (char == "s"):
                print("Received: Reverse")
                tbot.backward(speed)
                tbot.fill_underlighting(RED)
 
            elif (char == "x"):
                print("Received: Stop")
                tbot.stop()
                tbot.clear_underlighting()
 
            elif (char == "q"):
                speed = round(min(1.0, speed + 0.1), 1)
                print(f"Received: Increase Speed. New speed: {speed}")
 
            elif (char == "z"):
                speed = round(max(0.0, speed - 0.1), 1)
                print(f"Received: Decrease Speed. New speed: {speed}")
                
except serial.SerialException as e:
    print(f"Error: Could not open port {SERIAL_PORT}.")
except KeyboardInterrupt:
    print("\nProgram stopped.")
finally:
    tbot.stop()
    tbot.clear_underlighting()
    if 'board' in locals() and board.is_open:
        board.close()
    print("Exiting.")