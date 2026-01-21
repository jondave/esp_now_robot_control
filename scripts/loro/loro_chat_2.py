import serial
import threading
import time

# --- CONFIGURE THIS ---
SERIAL_PORT = '/dev/ttyACM1'    # Change to your RP2040/Arduino Port
BAUD_RATE = 9600                # IMPORTANT: LoRa code uses 9600, not 115200
# --------------------

# This function runs in a separate thread to read data from the LoRa module
def read_from_serial(board):
    try:
        while True:
            # Check if data is waiting (non-blocking is safer here)
            if board.in_waiting > 0:
                line = board.readline().decode('utf-8', errors='replace').strip()
                
                if line:
                    # Logic to clean up the output
                    if line.startswith("RX:"):
                        # Strip off the "RX: " part to show just the message
                        clean_msg = line[4:] 
                        print(f"\n<< Received: {clean_msg}")
                        # Re-print the input prompt so the UI looks clean
                        print(">> Send: ", end="", flush=True)
                    
                    elif line.startswith("TX_ERR"):
                        print(f"\n[!] Transmission Error: {line}")
                        print(">> Send: ", end="", flush=True)

                    # We ignore "TX_DONE" and "TX_QUEUED" to keep the screen clean

    except Exception:
        return

try:
    # Connect to the LoRa Module
    board = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to LoRa on {SERIAL_PORT}.")
    time.sleep(2) # Give the board 2 seconds to reset/stabilize
    
    # Start the background thread for reading
    read_thread = threading.Thread(target=read_from_serial, args=(board,))
    read_thread.daemon = True 
    read_thread.start()

    print("Type a message and press Enter to send. Type 'quit' to exit.")
    print("-" * 60)

    while True:
        # Get user input from the main thread
        message = input(">> Send: ") 
        
        if message.lower() == 'quit':
            break
        
        # Safety Check: LoRa cannot send more than ~250 chars at once
        if len(message) > 250:
            print(f"[!] Error: Message is too long ({len(message)} chars). Limit is 250.")
            continue

        if message:
            # Send the message to the Arduino, ending with a newline
            board.write(f"{message}\n".encode('utf-8'))
        
except serial.SerialException:
    print(f"Error: Could not open port {SERIAL_PORT}.")
    print("Is the board plugged in and the port correct?")
except KeyboardInterrupt:
    print("\nStopping...")
finally:
    if 'board' in locals() and board.is_open:
        board.close()
        print("Serial port closed.")