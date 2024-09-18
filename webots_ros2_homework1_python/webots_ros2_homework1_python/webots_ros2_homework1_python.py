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



LINEAR_VEL = 0.22
ANGULAR_VEL = 0.1
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.7
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
        timer_period = 0.5
        self.pose_saved=''
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.current_position = None
        self.current_orientation = None
        self.orientation = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        
    def turn_x_deg(self, x):
        start = self.current_orientation
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = ANGULAR_VEL
        self.publisher_.publish(self.cmd)
        self.get_logger().info('Step 1')
        self.turtlebot_moving = True
        while (abs(self.current_orientation - start - x)) > 0.01:
            continue
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        self.get_logger().info('Step 2')
        self.turtlebot_moving = False

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

    def dist_from_start(self, pos):
        (x,y) = pos
        dx = x - self.start_x
        dy = y - self.start_y
        return math.sqrt(dx**2 + dy**2)

    def get_proj_pos(self, dir, dist):
        if self.current_position == None:
            return 0,0
        curr_x, curr_y = self.current_position
        #current heading is self.orientation
        theta = self.orientation + math.radians(dir)
        proj_x = curr_x + math.cos(theta) * dist
        proj_y = curr_y + math.sin(theta) * dist
        proj_pos = proj_x, proj_y
        return proj_pos
        
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
        position = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation
        (posx, posy, posz) = (position.x, position.y, position.z)
        self.current_position = (position.x, position.y)
        self.orientation = orientation.z
        (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
        self.get_logger().info('self position: {},{},{}'.format(posx,posy,posz));
        # similarly for twist message if you need
        self.pose_saved=position
        
        #Example of how to identify a stall..need better tuned position deltas; wheels spin and example fast
        #diffX = math.fabs(self.pose_saved.x- position.x)
        #diffY = math.fabs(self.pose_saved.y - position.y)
        #if (diffX < 0.0001 and diffY < 0.0001):
           #self.stall = True
        #else:
           #self.stall = False
           
        return None
        
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
        best_dir = 0
        best_dist = 0
        best_tot_dist = 0
        for dir in range(len(self.scan_cleaned)):
            dist = self.scan_cleaned[dir]
            projected_pos = self.get_proj_pos(dir, self.scan_cleaned[dir])
            projected_tot_dist = self.dist_from_start(projected_pos)
            if projected_tot_dist > best_tot_dist:
                best_dir = dir
                best_dist = dist
                best_tot_dist = projected_tot_dist
        self.get_logger().info('Call to turn %f deg' % best_dir)
        self.turn_x_deg(best_dir)
        self.get_logger().info('Call to move %f m' % best_dist)
        self.move_x_dist(best_dist)
            
            
            
            

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
