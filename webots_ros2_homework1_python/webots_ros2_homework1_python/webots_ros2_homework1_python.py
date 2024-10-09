import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class RotateRobot(Node):
    def __init__(self):
        super().__init__('rotate_robot')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_yaw = None  # Store initial yaw angle
        self.current_yaw = None
        self.total_rotation = 0.0  # Track total rotation
        self.target_rotation = math.radians(360)  # Target rotation in radians (180 degrees)
        self.rotation_speed = math.radians(120)  # Rotation speed in radians/sec (30 degrees/sec)

        self.start_time = time.time()  # Record the start time for the simulation

    def quaternion_to_euler_yaw(self, x, y, z, w):
        # Convert quaternion to yaw (rotation around the z-axis)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def odom_callback(self, msg):
        # Extract orientation quaternion from odometry message
        orientation_q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_euler_yaw(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

        # Initialize the start yaw when rotation begins
        if self.start_yaw is None:
            self.start_yaw = self.current_yaw

    def timer_callback(self):
        if self.current_yaw is None or self.start_yaw is None:
            return  # Wait until odometry data is received

        # Calculate the amount of rotation between the previous and current yaw
        yaw_diff = self.current_yaw - self.start_yaw

        # Adjust for wraparound (-pi to pi)
        if yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        elif yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi

        # Accumulate the total rotation
        self.total_rotation += yaw_diff
        self.start_yaw = self.current_yaw  # Update start_yaw for the next calculation

        # Create the Twist message for rotation
        cmd = Twist()

        if abs(self.total_rotation) < abs(self.target_rotation):
            cmd.angular.z = self.rotation_speed  # Rotate at 30 degrees per second
            self.publisher_.publish(cmd)

            # Print the current time and rotation to the console
            current_time = time.time() - self.start_time
            print(f"Time: {current_time:.2f}s, Total Rotation: {math.degrees(self.total_rotation):.2f} degrees")

            # Log to the console as well
            self.get_logger().info(f"Rotating... Total Rotation: {math.degrees(self.total_rotation):.2f} degrees")
        else:
            # Stop the robot after reaching the target rotation
            cmd.angular.z = 0.0
            self.publisher_.publish(cmd)
            self.get_logger().info(f"Rotation complete. Robot has turned {math.degrees(self.total_rotation):.2f} degrees.")

            # Print completion to the console
            current_time = time.time() - self.start_time
            print(f"Rotation complete at {current_time:.2f} seconds.")

def main(args=None):
    rclpy.init(args=args)
    rotate_robot = RotateRobot()
    rclpy.spin(rotate_robot)
    rotate_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
