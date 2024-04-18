#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation as R
import csv
from time import sleep


class VelTest(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('vel_test_node')
        
        
        # self.create_subscription(Odometry, '/ego_racecar/odom', self.pose_callback, 10)
        self.create_subscription(Odometry, '/pf/pose/odom', self.pose_callback, 10)

        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.start_time = self.get_clock().now().nanoseconds
        self.get_logger().info(str(self.start_time))
        
    
    def pose_callback(self, pose_msg):
        
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "ego_racecar/base_link"
        drive_msg.drive.steering_angle = 0.0
        pf_speed = np.linalg.norm(np.array([pose_msg.twist.twist.linear.x, pose_msg.twist.twist.linear.y]))
        curr_time = Time.from_msg(pose_msg.header.stamp).nanoseconds
        if curr_time - self.start_time < 2 * 1e9:
            drive_msg.drive.speed = 2.0
            desired = 2.0
        else:
            drive_msg.drive.speed = self.interpolate_vel(pf_speed, 7.0)
            desired = 7.0
        self.get_logger().info("pf_vel: {}  desired_speed: {} drive_msg: {}".format(pf_speed, desired, drive_msg.drive.speed))
        self.drive_publisher.publish(drive_msg)

    def interpolate_vel(self, current_vel, seg_vel):
        """
        param:
            current_vel : current velocity given by the particle filter
            seg_vel : velocity in current segment
        returns:
            command_vel : interpolated velocity
        """
        acc = max(0.2, current_vel ** 2 * 0.15)
        timestep = 1.0
        
        if current_vel < seg_vel: # if we are accelerating
            command_vel = current_vel + acc * timestep
            command_vel = min(command_vel, seg_vel)
        else: # decelrating
            command_vel = current_vel - acc * timestep
            command_vel = max(command_vel, seg_vel)
        return command_vel


def main(args=None):
    rclpy.init(args=args)
    print("VelocityTest Initialized")
    vel_test_node = VelTest()
    rclpy.spin(vel_test_node)

    vel_test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
