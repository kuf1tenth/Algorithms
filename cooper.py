'''Node to win race
    Cooper Wright
    Created: sometime in 2024
    Last Edited: 10/22/2024
'''

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import time

class BrakeNode(Node):
    def __init__(self):
        super().__init__('brake_node')
        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scan_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.get_logger().info('BrakeNode has been started.')

        self.lap_count = 0
        self.lap_time = time.time()

        self.start_finish_line_x = 0.0  # Define the x-coordinate of the start/finish line
        self.previous_x_position = None  # To track the previous position of the car
        self.crossed_line = False  # To track whether the car has crossed the line
        
        self.position = None
        self.orientation = None
        
    def scan_callback(self, msg):
    
        leftSideMin = min(msg.ranges[541:680])
        leftSideMax = max(msg.ranges[541:810])
        rightSideMin = min(msg.ranges[400:539])
        rightSideMax = max(msg.ranges[270:539])
        forward = min(msg.ranges[530:550])
        
        
        lidar = list(msg.ranges)
        theGap = lidar.index(max(lidar[270:810]))
        print(theGap)
        
        if forward > 3:
            turnTHM = .5 #.5
        else:
            turnTHM = 1
        if rightSideMin <= 1.5 * turnTHM and leftSideMin <= 1.5 * turnTHM:
            if rightSideMax > leftSideMax:
                direction = -1
            else: 
                direction = 1
            speed = 2.0
            steer = .5 * direction
        else:
            if rightSideMin <= 1.5 * turnTHM and rightSideMin > 0:
                speed = 3.0
                steer = (.8/rightSideMin)
            elif leftSideMin <= 1.5 * turnTHM and leftSideMin > 0:
                speed = 3.0
                steer = -(.8/leftSideMin) 
            else:
                speed = 6.0 #6
                steer = 0.0
        
        # Publish a velo 0 message if wall is within .5 meters of the front of the car
        lookahead = min(msg.ranges[500:580])
        if lookahead <= .5:
            self.publish_ackermann_drive(speed/15, steer*5) 
        elif lookahead <= 2:
            self.publish_ackermann_drive(speed * (lookahead/2), steer)
        else:
            self.publish_ackermann_drive(speed, steer)
        
    def check_lap(self, current_position):
        # Check if we have a previous position to compare
        if self.previous_x_position is None:
            self.previous_x_position = current_position.x
            return
        # Check if the car crossed the start/finish line going forward
        if self.previous_x_position < self.start_finish_line_x <= current_position.x:
            if not self.crossed_line:
                self.lap_count += 1
                self.crossed_line = True
        
        self.get_logger().info(f'Lap {self.lap_count} completed!')

        # Reset the line crossing flag if the car moves away from the line
        if current_position.x < self.start_finish_line_x:
            self.crossed_line = False

        # Update previous x position for the next callback
        self.previous_x_position = current_position.x
        
        if self.lap_count == 1:
            self.get_logger().info(f"Lap time: {time.time() - self.lap_time}")
            self.destroy_node()
            quit()   
            
    def odom_callback(self, msg):
        self.get_logger().info('Received Odometry data')
        # Process Odometry data here
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        self.check_lap(self.position)

    def publish_ackermann_drive(self, speed, steering_angle):
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header = Header()
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        ackermann_msg.drive.speed = speed
        ackermann_msg.drive.steering_angle = steering_angle

        self.ackermann_publisher.publish(ackermann_msg)
        #self.get_logger().info(f'Published AckermannDriveStamped message: speed={speed}, steering_angle={steering_angle}')

def main(args=None):
    rclpy.init(args=args)
    node = BrakeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
