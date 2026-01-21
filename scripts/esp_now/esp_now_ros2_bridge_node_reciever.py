import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trilobot import Trilobot

# --- TUNE THIS ---
# You may need to adjust this, it's the distance between your robot's wheels in meters
WHEEL_BASE = 0.1  # Example: 10cm
# -----------------

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.get_logger().info("Motor driver started. Subscribing to /cmd_vel.")
        self.tbot = Trilobot()
        
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

    def cmd_vel_callback(self, msg):
        # This is the core logic for a "differential drive" robot.
        # It converts linear.x (forward) and angular.z (rotate)
        # into left and right wheel speeds.
        
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Differential drive kinematics
        # Clamping speed between -1.0 and 1.0
        right_speed = max(min(linear_x + (angular_z * WHEEL_BASE / 2), 1.0), -1.0)
        left_speed = max(min(linear_x - (angular_z * WHEEL_BASE / 2), 1.0), -1.0)

        # Send speeds to the motors
        self.tbot.set_left_speed(left_speed)
        self.tbot.set_right_speed(right_speed)
        
        self.get_logger().info(f'Twist In: (lin_x={linear_x:.2f}, ang_z={angular_z:.2f}) -> Motor Out: (L={left_speed:.2f}, R={right_speed:.2f})')

def main(args=None):
    rclpy.init(args=args)
    try:
        driver_node = MotorDriver()
        rclpy.spin(driver_node)
    except KeyboardInterrupt:
        pass
    finally:
        # On exit, make sure to stop the motors
        driver_node.tbot.stop()
        driver_node.destroy_node()
        rclpy.shutdown()
        print("Motor driver shut down, motors stopped.")

if __name__ == '__main__':
    main()