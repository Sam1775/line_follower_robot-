#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

class LidarScannerNode(Node):
    def __init__(self):
        super().__init__('lidar_scanner_node')
        
        # Subscribe to Gazebo raw LiDAR topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/model/vacuum_bot/scan',
            self.scan_callback,
            10
        )
        
        # Publisher for processed zone distances [Front, Left, Right]
        self.publisher = self.create_publisher(Float32MultiArray, '/processed_scan', 10)
        self.get_logger().info('Lidar Scanner Node Started!')

    def scan_callback(self, msg):
        ranges = msg.ranges
        num_samples = len(ranges)
        
        if num_samples == 0:
            return

        # Divide 360 degree scan into 3 zones
        # Front zone: center slice
        # Right zone: start of array
        # Left zone: end of array
        right_zone = ranges[0 : num_samples // 3]
        front_zone = ranges[num_samples // 3 : 2 * (num_samples // 3)]
        left_zone  = ranges[2 * (num_samples // 3) : ]

        # Helper function to get valid minimum distance, ignoring .inf
        def get_min_dist(zone):
            valid = [r for r in zone if not float('inf') == r and r > 0.15]
            return min(valid) if valid else 10.0

        min_front = get_min_dist(front_zone)
        min_left  = get_min_dist(left_zone)
        min_right = get_min_dist(right_zone)

        # Publish array: [Front, Left, Right]
        out_msg = Float32MultiArray()
        out_msg.data = [float(min_front), float(min_left), float(min_right)]
        self.publisher.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarScannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()