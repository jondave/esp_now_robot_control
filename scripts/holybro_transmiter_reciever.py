import serial
import time
import threading

# --- Configuration ---
PORT = '/dev/ttyUSB0'  # Change to your port, e.g. 'COM5' on Windows and '/dev/ttyUSB0' on Linux
BAUD = 57600           # Typical baud rate for SiK radios
SEND_INTERVAL = 5      # seconds between automatic sends

# --- Function to continuously read incoming data ---
def read_from_radio(ser):
    print("[Reader] Listening for incoming data...")
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8', errors='ignore').strip()
            if data:
                print(f"[Received] {data}")
        time.sleep(0.1)  # Small delay to reduce CPU load

# --- Main program ---
def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.5)
        print(f"[Connected] Serial port open on {ser.name}")

        # Start a background thread to read data continuously
        reader_thread = threading.Thread(target=read_from_radio, args=(ser,), daemon=True)
        reader_thread.start()

        # Main loop: periodically send data
        while True:
            message = f"Hello from Python Ubuntu at {time.strftime('%H:%M:%S')}\n"
            ser.write(message.encode('utf-8'))
            print(f"[Sent] {message.strip()}")
            time.sleep(SEND_INTERVAL)

    except KeyboardInterrupt:
        print("\n[Exiting] Script stopped by user.")
    except serial.SerialException as e:
        print(f"[Error] Serial connection failed: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("[Closed] Serial port closed.")

# --- Run ---
if __name__ == "__main__":
    main()
