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
import time
import random

LINEAR_VEL = 0.15
ANGULAR_VEL = 1.0
STOP_DISTANCE = 0.4
LIDAR_ERROR = 0.05
WALL_DISTANCE = 0.35
LIDAR_AVOID_DISTANCE = .7
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
MAX_MOVE_DIST = 2
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX=150
LEFT_SIDE_INDEX=90

class RandomWalk(Node):

    def __init__(self):
        # Initialize the publisher
        super().__init__('random_walk_node')
        self.scan_cleaned = []
        self.stall = False
        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.laser_forward = 0
        self.odom_data = 0
        self.pose_saved = ''
        self.cmd = Twist()
        self.timer = self.create_timer(0.25, self.timer_callback)
        self.current_position = None
        self.orientation = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.distance_from_start = 0.0  # Track distance from start
        # Open a file for writing the position
        self.position_file = open('position.txt', 'w')

    def dist_from_start(self, pos):
        """Calculate distance from the starting position."""
        (x, y) = pos
        dx = x - self.start_x
        dy = y - self.start_y
        return math.sqrt(dx**2 + dy**2)
        
    def quaternion_to_yaw(self, q):
        # Quaternion to Euler angles (yaw)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

        
    def normalize_angle(self,angle):
        """Normalize the angle to be within [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))
        
    def turn_x_deg(self, x):
        x = math.radians(x)
        if self.orientation == None:
            self.get_logger().info('Turn x deg called with None for self.current_orientation')
            return
        turn_duration = (x / ANGULAR_VEL)
        self.cmd.angular.z = ANGULAR_VEL
        self.cmd.linear.x = 0.0
        self.publisher_.publish(self.cmd)
        self.get_logger().info('Turn x deg called for %f duration (s)' % turn_duration)
        time.sleep(turn_duration)
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        return
        
    def move_x_dist(self, x):
        if x > MAX_MOVE_DIST:
            x = MAX_MOVE_DIST
        elif x < STOP_DISTANCE:
            x = 0.0
        else:
            x = x - STOP_DISTANCE
        self.cmd.linear.x = LINEAR_VEL
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        self.turtlebot_moving = True
        time.sleep(x / LINEAR_VEL)
        self.cmd.linear.x = 0.0
        self.publisher_.publish(self.cmd)
        self.turtlebot_moving = False
        return
    
    def listener_callback1(self, msg1):
        #self.get_logger().info('scan: "%s"' % msg1.ranges)
        scan = msg1.ranges
        self.scan_cleaned = []
       
        #self.get_logger().info('scan: "%s"' % scan)
        # Assume 360 range measurements
        for reading in scan:
            if reading == float('Inf'):
                self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                self.scan_cleaned.append(0.0)
            else:
            	self.scan_cleaned.append(reading)
            
    def listener_callback2(self, msg2):
        """Odometry callback to track position and calculate distance from start."""
        position = msg2.pose.pose.position
        self.current_position = (position.x, position.y)
    
        # Update distance from start
        self.distance_from_start = self.dist_from_start(self.current_position)
    
        # Log the current position and distance from start
        self.get_logger().info('Current position: {}'.format(self.current_position))
        self.get_logger().info('Distance from start: {}'.format(self.distance_from_start))
    
        # Set initial position as the start position if it's the first odometry reading
        if self.start_x == 0.0 and self.start_y == 0.0:
            self.start_x = position.x
            self.start_y = position.y
            self.get_logger().info('Initial position saved as: {}, {}'.format(self.start_x, self.start_y))
        # Write the current position to the file
        self.position_file.write(f'{position.x}, {position.y}\n')
        self.position_file.flush()  # Ensure the data is written to the file
           
    def timer_callback(self):
        if len(self.scan_cleaned) == 0:
            self.turtlebot_moving = False
            return
        
        # Get the minimum distance from the laser scans on the front, right, and left
        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])
    
        # Log the distances for debugging
        self.get_logger().info('Left side min distance: %f' % left_lidar_min)
        self.get_logger().info('Front min distance: %f' % front_lidar_min)
        self.get_logger().info('Right side min distance: %f' % right_lidar_min)
        
        # If an obstacle is directly in front, turn left to avoid it
        if front_lidar_min < SAFE_STOP_DISTANCE:
            self.get_logger().info('Obstacle detected ahead, turning left.')
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = ANGULAR_VEL * .5
            self.publisher_.publish(self.cmd)
            return
    
        # Follow the right wall
        if right_lidar_min < WALL_DISTANCE - LIDAR_ERROR:
            # Too close to the right wall, turn left slightly
            self.get_logger().info('Too close to right wall, turning left.')
            self.cmd.angular.z = ANGULAR_VEL * 0.5  # Turn slightly left
            self.cmd.linear.x = LINEAR_VEL * 0.5    # Slow forward movement
        elif right_lidar_min > WALL_DISTANCE + LIDAR_ERROR and right_lidar_min < 3 * (WALL_DISTANCE + LIDAR_ERROR):
            # Too far from the right wall, turn right slightly
            self.get_logger().info('Too far from right wall, turning right.')
            self.cmd.angular.z = -ANGULAR_VEL * .5  # Turn slightly right
            self.cmd.linear.x = LINEAR_VEL * 0.5   # Slow forward movement
        else:
            # Maintain a straight path, but add a random angular velocity between -20 and 20 degrees
            random_angle_deg = random.uniform(-45, 45)  # Generate random angle between -20 and 20 degrees
            random_angle_rad = math.radians(random_angle_deg)  # Convert degrees to radians
    
            self.get_logger().info('Maintaining path with random direction change of %f degrees' % random_angle_deg)
            
            self.cmd.angular.z = random_angle_rad  # Set the random angular velocity
            self.cmd.linear.x = LINEAR_VEL  # Maintain forward movement
        
        # Publish the command
        self.publisher_.publish(self.cmd)
        
def __del__(self):
        # Ensure the file is closed when the node is destroyed
        self.position_file.close()    
    
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
