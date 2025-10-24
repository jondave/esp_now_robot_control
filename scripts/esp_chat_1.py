import serial
import threading
import time

# --- CONFIGURE THIS ---
SERIAL_PORT = 'COM3'  # Change this to your ESP32's COM port
BAUD_RATE = 115200
# --------------------

# This function runs in a separate thread to read data from the ESP32
def read_from_serial(board):
    try:
        while True:
            line = board.readline()
            if line:
                data = line.decode('utf-8').strip()
                # Print the message received from the other ESP32
                print(f"<< Received: {data}")
    except Exception as e:
        # This will happen when the port is closed
        return

try:
    # Connect to the ESP32 (your "gateway" board)
    board = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to ESP32 on {SERIAL_PORT}.")
    time.sleep(1) # Give the board a second to reset
    
    # Start the background thread for reading
    read_thread = threading.Thread(target=read_from_serial, args=(board,))
    read_thread.daemon = True # This lets the program exit when the main thread stops
    read_thread.start()

    print("Type a message and press Enter to send. Type 'quit' to exit.")

    while True:
        # Get user input from the main thread
        message = input(">> Send: ") 
        
        if message.lower() == 'quit':
            break
        
        # Send the message to the ESP32, ending with a newline
        board.write(f"{message}\n".encode('utf-8'))
        
except serial.SerialException as e:
    print(f"Error: Could not open port {SERIAL_PORT}.")
    print("Is the board plugged in and the port correct?")
except KeyboardInterrupt:
    print("\nStopping...")
finally:
    if 'board' in locals() and board.is_open:
        board.close()
        print("Serial port closed.")