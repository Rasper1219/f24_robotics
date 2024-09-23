import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import re
import os



LINEAR_VEL = 0.22
STOP_DISTANCE = 0.5
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.7
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX=150
LEFT_SIDE_INDEX=90
MIN_RIGHT_WALL = 0.2
MAX_RIGHT_WALL = 1.0

def extract_turtlebot_start(file_path):
    translation = None
    rotation = None
    in_turtlebot_section = False

    with open(file_path, 'r') as file:
        for line in file:
            # Ignore commented-out lines
            line = line.strip()
            if line.startswith('#') or not line:
                continue

            # Check if entering TurtleBot3Burger section
            if 'TurtleBot3Burger {' in line:
                in_turtlebot_section = True

            # Exit TurtleBot3Burger section
            if in_turtlebot_section and '}' in line:
                break

            # Extract translation and rotation if within the TurtleBot3Burger block
            if in_turtlebot_section:
                translation_match = re.match(r'translation\s+([\d\.\-]+)\s+([\d\.\-]+)\s+([\d\.\-]+)', line)
                rotation_match = re.match(r'rotation\s+([\d\.\-]+)\s+([\d\.\-]+)\s+([\d\.\-]+)\s+([\d\.\-]+)', line)

                if translation_match:
                    translation = tuple(map(float, translation_match.groups()))
                elif rotation_match:
                    rotation = tuple(map(float, rotation_match.groups()))

    if translation and rotation:
        return {'translation': translation, 'rotation': rotation}
    else:
        return "TurtleBot3Burger start values not found."


class RandomWalk(Node):

    def __init__(self):
        # Initialize the node with the name 'random_walk_node'
        super().__init__('random_walk_node')
        
        # Initialize variables for storing sensor data and robot state
        self.scan_cleaned = []  # Cleaned LIDAR data (filtered and processed)
        self.stall = False  # Flag to detect if the robot is stalled
        self.turtlebot_moving = False  # Flag to indicate if the robot is currently moving
        self.first_pos_store = False  # Flag to store the first position of the robot
        
        # Create a publisher for controlling the robot's velocity
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribe to the LIDAR scan topic to receive distance measurements
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        # Subscribe to the odometry topic to receive position and orientation data
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        # Initialize additional variables for controlling the robot
        self.laser_forward = 0  # Variable to store forward LIDAR data
        self.odom_data = 0  # Variable to store odometry data
        self.pose_saved = ''  # Variable to save the robot's position
        self.cmd = Twist()  # Twist message used to control the robot's velocity
        
        # Create a timer to call the timer_callback function periodically (every 0.5 seconds)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def listener_callback1(self, msg1):
        """Callback function to process incoming LIDAR scan data."""
        # Store the received scan data in a temporary variable
        scan = msg1.ranges
        
        # Clear the cleaned scan data list before processing new data
        self.scan_cleaned = []
        
        # Process the raw scan data to handle infinite distances and NaN values
        for reading in scan:
            if reading == float('Inf'):
                # Replace 'Inf' readings with a maximum distance value
                self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                # Replace 'NaN' readings with zero
                self.scan_cleaned.append(0.0)
            else:
                # Keep valid readings as they are
                self.scan_cleaned.append(reading)

    def listener_callback2(self, msg2):
        """Callback function to process incoming odometry data."""
        # Extract position and orientation data from the odometry message
        position = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation
        
        self.pose_saved = position

        if not self.first_pos_store:
            file_path = '/home/ryan/Desktop/CS560/HW1/f24_robotics/webots_ros2_homework1_python/worlds/f23_robotics_1.wbt'
            turtlebot_start = extract_turtlebot_start(file_path)
            self.first_pos_store = True
            counter = 0
            # Grab the files in the directory, and check the highest number
            for file in os.listdir('Homework1/Data'):
                if file.startswith('robot_position') and str(turtlebot_start["translation"]) in file:
                    counter = max(counter, int(file.split('_')[-1].split('.')[0]))

            counter += 1

            self.file_name = f'Homework1/Data/robot_position_{turtlebot_start["translation"]}_{counter}.txt'

            with open(self.file_name, 'w') as f:
                if turtlebot_start != "TurtleBot3Burger start values not found.":
                    f.write(f'start -> translation: {turtlebot_start["translation"]}| rotation: {turtlebot_start["rotation"]}\n')

                f.write(f'x: {self.pose_saved.x}, y: {self.pose_saved.y}\n')
            self.store_timer = self.create_timer(5, self.store_timer_callback)

    def store_timer_callback(self):
        with open(self.file_name, 'a') as f:
            f.write(f'x: {self.pose_saved.x}, y: {self.pose_saved.y}\n')
        
    def timer_callback(self):
        if (len(self.scan_cleaned)==0):
    	    self.turtlebot_moving = False
    	    return
    	    
        #left_lidar_samples = self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX]
        #right_lidar_samples = self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX]
        #front_lidar_samples = self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX]
        
        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])

        #self.get_logger().info('left scan slice: "%s"'%  min(left_lidar_samples))
        #self.get_logger().info('front scan slice: "%s"'%  min(front_lidar_samples))
        #self.get_logger().info('right scan slice: "%s"'%  min(right_lidar_samples))
        # Need to get to wall on right side, and keep following it within the range

        if front_lidar_min < SAFE_STOP_DISTANCE:
            if self.turtlebot_moving == True:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.5 #Max rotate left
                self.publisher_.publish(self.cmd)
                self.get_logger().info('Stopping')
                return
        elif front_lidar_min < LIDAR_AVOID_DISTANCE:
                self.cmd.linear.x = 0.05 #slow it daaahn bbgirl
                if (right_lidar_min > left_lidar_min): #corner on front left, get back to right following
                   self.cmd.angular.z = -0.5 #max turn right
                #no turning left because we want to stay on right walls
                self.publisher_.publish(self.cmd)
                self.get_logger().info('Turning')
                self.turtlebot_moving = True
                return
        else:
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = 0.0
            if right_lidar_min > MAX_RIGHT_WALL: #if too far from right wall
                self.cmd.angular.z = -0.5
                self.cmd.linear.x = 0.1 # slow down a bit and turn towards the wall

            elif right_lidar_min < MIN_RIGHT_WALL: #too close, turn a little left
                self.cmd.angular.z = 0.5
                self.cmd.linear.x = 0.1
            elif right_lidar_min > (MAX_RIGHT_WALL / 2.0): #good enough, adjust angular properly
                
                adjustedAngularSpeed = (right_lidar_min - 0.5) / (1.0 - 0.5)

                
                self.cmd.angular.z = -0.20 * (adjustedAngularSpeed ** 0.5) #Adjust that thang
                self.cmd.linear.x = 0.15 #not too fast since we turning a bit
            

            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
            

        self.get_logger().info('Distance of the obstacle : %f' % front_lidar_min)
        self.get_logger().info('I receive: "%s"' %
                               str(self.odom_data))
        if self.stall == True:
           self.get_logger().info('Stall reported')
        
        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % self.cmd)
 


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    random_walk_node = RandomWalk()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(random_walk_node)
    # Explicity destroy the node
    random_walk_node.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()



if __name__ == '__main__':
    main()
