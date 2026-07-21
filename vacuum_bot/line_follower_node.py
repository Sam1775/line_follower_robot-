#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        
        self.bridge = CvBridge()
        
        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/model/vacuum_bot/image_raw',
            self.image_callback,
            1
        )
        
        # Publish speed commands
        self.cmd_pub = self.create_publisher(Twist, '/model/vacuum_bot/cmd_vel', 1)
        self.get_logger().info('Camera Line Follower Node Started!')

    def image_callback(self, msg):
        try:
            # 1. Convert ROS Image to OpenCV BGR Format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        h, w, _ = cv_image.shape

        # 2. Crop image to focus on the floor right in front (Bottom 30%)
        crop = cv_image[int(h * 0.7) : h, 0 : w]

        # 3. Convert to HSV & Filter for Black Line on Ground
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 80])  # Adjust brightness cutoff if needed
        mask = cv2.inRange(hsv, lower_black, upper_black)

        # 4. Find Line Centroid using Moments
        M = cv2.moments(mask)
        cmd = Twist()

        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00']) # X coordinate of line center
            
            # Calculate Error relative to image center
            image_center_x = w // 2
            error = cx - image_center_x

            # Proportional Steering Control
            cmd.linear.x = 0.2                     # Constant forward speed
            cmd.angular.z = -float(error) / 120.0  # Steering gain

            self.get_logger().info(f'Line Center: {cx} | Error: {error} | Steering: {cmd.angular.z:.2f}')
        else:
            # Line lost -> Slow rotate to search for track
            cmd.linear.x = 0.05
            cmd.angular.z = 0.3
            self.get_logger().warn('Line Lost! Searching for line...')

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()  