import rclpy
from rclpy.node import Node
import serial
import time
import json
import importlib  # For dynamic message type imports
from functools import partial  # To help with generic callbacks
from rosidl_runtime_py import convert  # The magic for Msg -> Dict
from rosidl_runtime_py import set_message  # <-- THE FIX IS HERE

# --- CONFIGURE THIS ---
SERIAL_PORT = '/dev/ttyUSB0'  # Linux: /dev/ttyUSB0, Windows: COM3
BAUD_RATE = 115200 # ESP NOW
# --------------------

# --- DEFINE YOUR CONFIGURATIONS ---

# This config runs on your PC
PC_CONFIG = {
    'node_name': 'pc_bridge_node',
    'send': {
        # 'ros_topic_name': 'message.type.string'
        'cmd_vel': 'geometry_msgs/Twist'
    },
    'receive': {
        # 'ros_topic_name': 'message.type.string'
        # '/odom': 'nav_msgs/Odometry'
        # '/ping': 'std_msgs/String'
    }
}

# This config runs on your Robot
ROBOT_CONFIG = {
    'node_name': 'robot_bridge_node',
    'send': {
        # 'ros_topic_name': 'message.type.string'
        # '/odom': 'nav_msgs/Odometry'
    },
    'receive': {
        # 'ros_topic_name': 'message.type.string'
        'cmd_vel': 'geometry_msgs/Twist'
    }
}

# --- CHOOSE WHICH CONFIG TO USE ---
# Change this to ROBOT_CONFIG when on the robot or PC_CONFIG when on your PC
ACTIVE_CONFIG = PC_CONFIG
# ----------------------------------


class EspNowBridge(Node):

    def __init__(self, config, serial_port, baud_rate):
        # Use the node name from the config dictionary
        super().__init__(config['node_name'])
        
        self.config = config
        self.publishers_ = {}
        self.subscriptions_ = {}
        
        self.get_logger().info(f"Starting node: {config['node_name']}")
        self.get_logger().info(f"Connecting to ESP32 on {serial_port}...")
        try:
            self.serial_port = serial.Serial(serial_port, baud_rate, timeout=1)
            time.sleep(1) # Give the board a second to reset
            self.get_logger().info("ESP32 connected.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open port {serial_port}: {e}")
            raise e

        # 1. Create all SUBSCRIBERS (for topics to send)
        for topic_name, msg_type_str in self.config['send'].items():
            msg_class = self.get_msg_class(msg_type_str)
            if msg_class:
                self.subscriptions_[topic_name] = self.create_subscription(
                    msg_class,
                    topic_name,
                    # Use partial to pass the topic_name to the callback
                    partial(self.listener_callback, topic_name=topic_name),
                    10)
                self.get_logger().info(f"Subscribing to '{topic_name}' ({msg_type_str})")

        # 2. Create all PUBLISHERS (for topics to receive)
        for topic_name, msg_type_str in self.config['receive'].items():
            msg_class = self.get_msg_class(msg_type_str)
            if msg_class:
                self.publishers_[topic_name] = self.create_publisher(
                    msg_class, 
                    topic_name, 
                    10)
                self.get_logger().info(f"Publishing to '{topic_name}' ({msg_type_str})")

        # 3. Create the serial port reader timer (for receiving)
        self.serial_read_timer = self.create_timer(0.01, self.read_serial_callback)

    def get_msg_class(self, msg_type_str):
        """Dynamically imports a ROS 2 message class from its string type."""
        try:
            pkg_name, msg_name = msg_type_str.split('/')
            module = importlib.import_module(f"{pkg_name}.msg")
            return getattr(module, msg_name)
        except Exception as e:
            self.get_logger().error(f"Failed to import msg type {msg_type_str}: {e}")
            return None

    def listener_callback(self, msg, topic_name):
        """Generic callback for all subscribed topics."""
        try:
            # 1. Convert ROS 2 message to a dictionary
            msg_dict = convert.message_to_ordereddict(msg)
            
            # 2. Convert dict to a JSON string
            payload = json.dumps(msg_dict)
            
            # 3. Create final packet and send over serial
            packet = f"{topic_name}|{payload}\n"
            self.serial_port.write(packet.encode('utf-8'))
            self.get_logger().info(f'Sent -> {topic_name} ({len(payload)} bytes)')
            
        except Exception as e:
            self.get_logger().warn(f"Error serializing message for {topic_name}: {e}")

    def read_serial_callback(self):
        """Timer callback to read from serial and publish to topics."""
        if self.serial_port.in_waiting > 0:
            try:
                raw = self.serial_port.readline()
                if not raw:
                    return

                packet = raw.decode('utf-8', errors='ignore').strip()
                if '|' not in packet or not packet:
                    return  # Skip invalid or empty packets

                topic_name, payload = packet.split('|', 1)

                if topic_name in self.publishers_:
                    if not payload.startswith('{') or not payload.endswith('}'):
                        self.get_logger().warn(f"Skipping malformed JSON from {topic_name}")
                        return

                    msg_dict = json.loads(payload)
                    msg_class = self.publishers_[topic_name].msg_type
                    msg_to_publish = msg_class()
                    set_message.set_message_fields(msg_to_publish, msg_dict)
                    self.publishers_[topic_name].publish(msg_to_publish)
                    self.get_logger().info(f'Received <- {topic_name} ({len(payload)} bytes)')
            except Exception as e:
                self.get_logger().warn(f"Error parsing serial data: {e}")


def main(args=None):
    rclpy.init(args=args)
    bridge_node = None # Define bridge_node here to ensure it's in scope for finally
    try:
        bridge_node = EspNowBridge(ACTIVE_CONFIG, SERIAL_PORT, BAUD_RATE)
        rclpy.spin(bridge_node)
    except Exception as e:
        print(f"Node failed to start or shut down: {e}")
    except KeyboardInterrupt:
        print("Shutdown requested.")
    finally:
        if bridge_node is not None:
            if 'serial_port' in bridge_node.__dict__ and bridge_node.serial_port.is_open:
                bridge_node.serial_port.close()
                bridge_node.get_logger().info("Serial port closed.")
            bridge_node.destroy_node()
        rclpy.shutdown()
        print("ROS 2 shutdown complete.")

if __name__ == '__main__':
    main()