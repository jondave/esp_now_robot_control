#!/usr/bin/env python3
"""
ROS 2 bridge for Holybro telemetry radios.
Supports sending and receiving arbitrary ROS 2 topics as JSON over serial.
This example publishes/subscribes to /cmd_vel.
"""

import rclpy
from rclpy.node import Node
import serial
import time
import json
import importlib
from functools import partial
from rosidl_runtime_py import convert, set_message

# -----------------------------
# SERIAL CONFIG
# -----------------------------
SERIAL_PORT = '/dev/ttyUSB0'  # Linux: /dev/ttyUSB0 | WSL: /dev/ttySx | Windows: COM3
BAUD_RATE = 57600

# -----------------------------
# ROS 2 TOPIC CONFIG
# -----------------------------
# PC side: sends cmd_vel
PC_CONFIG = {
    'node_name': 'pc_bridge_node',
    'send': {
        'cmd_vel': 'geometry_msgs/Twist'
    },
    'receive': {
        # Optional: receive other topics if needed
    }
}

# Robot side: receives cmd_vel
ROBOT_CONFIG = {
    'node_name': 'robot_bridge_node',
    'send': {
        # Optional: send topics from robot
    },
    'receive': {
        'cmd_vel': 'geometry_msgs/Twist'
    }
}

# -----------------------------
# SELECT ACTIVE CONFIG
# -----------------------------
ACTIVE_CONFIG = PC_CONFIG  # Switch to ROBOT_CONFIG on the robot

# -----------------------------
# TELEMETRY BRIDGE NODE
# -----------------------------
class HolybroBridge(Node):
    def __init__(self, config, serial_port, baud_rate):
        super().__init__(config['node_name'])
        self.config = config
        self.publishers_ = {}
        self.subscriptions_ = {}

        self.get_logger().info(f"Starting node: {config['node_name']}")
        self.get_logger().info(f"Connecting to telemetry radio on {serial_port} @ {baud_rate}...")

        try:
            self.serial_port = serial.Serial(serial_port, baud_rate, timeout=1)
            time.sleep(1)  # Give the radio time to initialize
            self.get_logger().info("✅ Telemetry radio connected.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open port {serial_port}: {e}")
            raise e

        # Create subscribers for "send" topics
        for topic_name, msg_type_str in self.config['send'].items():
            msg_class = self.get_msg_class(msg_type_str)
            if msg_class:
                self.subscriptions_[topic_name] = self.create_subscription(
                    msg_class,
                    topic_name,
                    partial(self.listener_callback, topic_name=topic_name),
                    10
                )
                self.get_logger().info(f"Subscribed to '{topic_name}' ({msg_type_str})")

        # Create publishers for "receive" topics
        for topic_name, msg_type_str in self.config['receive'].items():
            msg_class = self.get_msg_class(msg_type_str)
            if msg_class:
                self.publishers_[topic_name] = self.create_publisher(msg_class, topic_name, 10)
                self.get_logger().info(f"Publishing to '{topic_name}' ({msg_type_str})")

        # Timer to continuously read serial
        self.serial_timer = self.create_timer(0.01, self.read_serial_callback)

    def get_msg_class(self, msg_type_str):
        """Dynamically import a ROS 2 message class."""
        try:
            pkg, msg = msg_type_str.split('/')
            module = importlib.import_module(f"{pkg}.msg")
            return getattr(module, msg)
        except Exception as e:
            self.get_logger().error(f"Failed to import {msg_type_str}: {e}")
            return None

    def listener_callback(self, msg, topic_name):
        """Send ROS 2 messages over serial."""
        try:
            msg_dict = convert.message_to_ordereddict(msg)
            payload = json.dumps(msg_dict)
            packet = f"{topic_name}|{payload}\n"
            self.serial_port.write(packet.encode('utf-8'))
            self.get_logger().info(f"[TX] {topic_name} -> {payload}")
        except Exception as e:
            self.get_logger().warn(f"Error sending {topic_name}: {e}")

    def read_serial_callback(self):
        """Receive serial messages and publish to ROS 2."""
        if self.serial_port.in_waiting > 0:
            try:
                raw = self.serial_port.readline()
                if not raw:
                    return
                line = raw.decode('utf-8', errors='ignore').strip()
                if '|' not in line:
                    return

                topic_name, payload = line.split('|', 1)
                if topic_name in self.publishers_:
                    msg_dict = json.loads(payload)
                    msg_class = self.publishers_[topic_name].msg_type
                    msg_obj = msg_class()
                    set_message.set_message_fields(msg_obj, msg_dict)
                    self.publishers_[topic_name].publish(msg_obj)
                    self.get_logger().info(f"[RX] {topic_name} <- {payload}")
            except Exception as e:
                self.get_logger().warn(f"Error parsing serial: {e}")

    def destroy_node(self):
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()


# -----------------------------
# MAIN
# -----------------------------
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HolybroBridge(ACTIVE_CONFIG, SERIAL_PORT, BAUD_RATE)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Bridge interrupted by user.")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()
        print("ROS 2 shutdown complete.")


if __name__ == "__main__":
    main()
