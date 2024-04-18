#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
# from ackermann_msgs.msg import AckermannDriveStamped

class TestNode(Node):
    """ 
    Publish virtual scan data for debugging
    """
    def __init__(self):
        super().__init__('test_node')
        lidarscan_topic = '/scan'
        self.pub_ = self.create_publisher(
            LaserScan,
            lidarscan_topic,
            10
        )
        self.frame_id = "ego_racecar/laser"
        self.angle_increment = 0.04351851996034384
        self.angle_max = 2.3499999046325684
        self.angle_min = -2.3499999046325684
        self.time_increment = 0.0
        self.scan_time = 0.0
        self.range_min = 0.0
        self.range_max = 30.0
        # self.dt = None
        self.last_time = None
        timer_period = 0.5 
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = LaserScan()
        msg.scan_time = self.scan_time
        msg.angle_increment = self.angle_increment
        msg.range_max = self.range_max
        msg.range_min = self.range_min
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.time_increment = self.time_increment
        msg.header.frame_id = self.frame_id
        msg.ranges = self.create_ranges()
        self.pub_.publish(msg)
        
    def create_ranges(self):
        ranges_length = (self.angle_max - self.angle_min) / self.angle_increment
        print(ranges_length)
        ranges = [
            1.2, 1.2, 1.2, 1.2, 1.3, 1.2, 1.2, 1.2, 1.2, 1.2, 
            3.3, 3.4, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 
            1.3, 1.2, 1.2, 1.2, 1.1, 1.2, 1.3, 1.2, 2.0, 2.0,
            2.0, 2.0, 2.3, 2.5, 2.4, 2.4, 2.4, 2.4, 2.4, 2.4,
            3.3, 3.4, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5,
            3.3, 3.4, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5,
            3.3, 3.4, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5,
            3.3, 3.4, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5,
            1.2, 1.2, 1.2, 1.2, 1.3, 1.2, 1.2, 1.2, 1.2, 1.2,
            2.2, 2.2, 2.2, 2.2, 2.2, 2.2, 2.2,2.2, 2.2, 2.2,
            2.2, 2.2, 2.2, 2.2, 2.2, 2.2, 2.2,2.2
            ]
        return ranges


def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    test_node.get_logger().info(("TestNode Initialized"))
    rclpy.spin(test_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
     