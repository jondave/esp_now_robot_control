import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

# --- CONFIGURE THIS ---
# Find your port with: ls /dev/ttyUSB* or ls /dev/ttyACM*
SERIAL_PORT = '/dev/ttyUSB0'  # Change this to your PC's ESP32 port
BAUD_RATE = 115200
# --------------------

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('pc_bridge_node')
        self.get_logger().info(f"Connecting to ESP32 on {SERIAL_PORT}...")
        
        try:
            self.serial_port = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(1) # Give the board a second to reset
            self.get_logger().info("ESP32 connected. Subscribing to /cmd_vel.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open port {SERIAL_PORT}: {e}")
            raise e

        # Create a subscriber to the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # A Twist message has linear (x,y,z) and angular (x,y,z)
        # For a ground robot, we only care about linear.x and angular.z
        lin_x = msg.linear.x
        ang_z = msg.angular.z

        # Format as a simple string: "linear.x,angular.z\n"
        data_str = f"{lin_x},{ang_z}\n"
        
        # Send it to the ESP32
        self.serial_port.write(data_str.encode('utf-8'))
        self.get_logger().info(f'Sent to ESP32: "{data_str.strip()}"')

def main(args=None):
    rclpy.init(args=args)
    try:
        bridge_node = CmdVelBridge()
        rclpy.spin(bridge_node)
    except Exception as e:
        print(f"Node failed to start: {e}")
    finally:
        # Check if bridge_node was successfully initialized before trying to destroy
        if 'bridge_node' in locals() and bridge_node.serial_port.is_open:
            bridge_node.serial_port.close()
        if 'bridge_node' in locals():
            bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()