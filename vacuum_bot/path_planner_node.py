#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/processed_scan',
            self.planner_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/model/vacuum_bot/cmd_vel', 10)
        self.safe_distance = 0.8  
        
        # 🧠 STATE MEMORY: Prevents left/right flickering
        self.turning_direction = 0  # 0: None, 1: Turning Left, -1: Turning Right
        
        self.get_logger().info('Improved Path Planner Node Started!')

    def planner_callback(self, msg):
        front, left, right = msg.data[0], msg.data[1], msg.data[2]
        cmd = Twist()

        # 1. Clear Path Ahead -> Move Forward & Reset Turning Memory
        if front > (self.safe_distance + 0.2): # 1.0m buffer to prevent rapid switching
            self.turning_direction = 0
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
            self.get_logger().info(f'Forward | Front: {front:.2f}m')

        # 2. Obstacle Detected -> Commit to chosen direction until front clears!
        else:
            cmd.linear.x = 0.05  # Slight forward crawl or 0.0
            
            # If we haven't picked a turning direction yet, choose the wider side
            if self.turning_direction == 0:
                if left > right:
                    self.turning_direction = 1   # Lock in LEFT
                else:
                    self.turning_direction = -1  # Lock in RIGHT

            # Execute locked turning direction
            if self.turning_direction == 1:
                cmd.angular.z = 0.6  # Turn Left
                self.get_logger().warn(f'Locked Turning LEFT | Front: {front:.2f}m')
            else:
                cmd.angular.z = -0.6 # Turn Right
                self.get_logger().warn(f'Locked Turning RIGHT | Front: {front:.2f}m')

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()