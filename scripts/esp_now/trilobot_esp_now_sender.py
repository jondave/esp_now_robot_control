import serial
import time
from pynput import keyboard

# --- CONFIGURE THIS ---
SERIAL_PORT = 'COM4'  # Change this to your PC's ESP32 COM port
BAUD_RATE = 115200
# --------------------

# This will hold the serial connection
board = None

# We track which movement key is held so we don't spam 'x'
last_move_key = None

def on_press(key):
    global board, last_move_key
    
    try:
        char = key.char
    except AttributeError:
        # This handles special keys (like 'shift', 'ctrl')
        return

    # Only send valid command characters
    if char in ['w', 'a', 's', 'd', 'q', 'z', 'p', 'x']:
        
        # If it's a movement key, send it
        if char in ['w', 'a', 's', 'd']:
            last_move_key = char
            send_command(char)
        
        # If it's a non-movement key, just send it once
        elif char in ['q', 'z', 'p', 'x']:
            send_command(char)
            

def on_release(key):
    global last_move_key
    
    try:
        char = key.char
    except AttributeError:
        return
    
    # If the key we just released was a movement key, send STOP
    if char == last_move_key and char in ['w', 'a', 's', 'd']:
        send_command('x') # Send 'x' (stop) on release
        last_move_key = None
    
    # Exit program if 'p' is pressed
    if char == 'p':
        print("Exiting...")
        return False # This stops the listener

def send_command(cmd_char):
    if board and board.is_open:
        print(f"Sending: {cmd_char}")
        board.write(f"{cmd_char}\n".encode('utf-8'))

# --- Main Program ---
try:
    board = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to ESP32 on {SERIAL_PORT}.")
    print("Keyboard control is active.")
    print("w/a/s/d = move | q/z = speed | x = stop | p = exit")
    print("Press keys to send commands...")
    
    # Start the keyboard listener
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

except serial.SerialException as e:
    print(f"Error: Could not open port {SERIAL_PORT}.")
    print("Is the board plugged in and the port correct?")
except KeyboardInterrupt:
    print("\nProgram stopped.")
finally:
    if 'board' in locals() and 'board' in globals() and board.is_open:
        # Send a final stop command before closing
        send_command('x')
        board.close()
        print("Serial port closed.")