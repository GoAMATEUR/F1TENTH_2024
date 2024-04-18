#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        self.sub_ = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.scan_callback,
            10
        )
        self.pub_ = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10
        )
        # 1: set PID gains
        self.kp = 2.5
        self.ki = 0.0000
        self.kd = 0.75
        self.dist = 0.7
        self.look_ahead = 0.4
        self.theta = np.pi / 18 * 7.5
        # 2: store history
        self.integral = 0
        self.prev_error = 0
        self.error = 0
        # 3: store any necessary values you think you'll need
        self.lidar_angle_increment = None
        self.lidar_angle_max = None
        self.lidar_angle_min = None


    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
        # assert self.lidar_angle_min <= angle <= self.lidar_angle_max, "Invalid lidar angle {}".format(angle) 
        idx = int((angle - self.lidar_angle_min) / self.lidar_angle_increment)
        return range_data[idx]

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        a = self.get_range(range_data, np.pi / 2 - self.theta)
        b = self.get_range(range_data, np.pi / 2)
        
        alpha = np.arctan2(a * np.cos(self.theta) - b, a * np.sin(self.theta))
        D_t = b * np.cos(alpha)
        D_t_ = D_t + self.look_ahead * np.sin(alpha)
        # self.get_logger().info("{}, {}, al: {}, D: {}, D_: {}".format(a, b, alpha, D_t, D_t_))
        return dist - D_t_

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        deriv = error - self.prev_error
        self.prev_error = error
        self.integral += error
        angle = - (self.kp * error + self.ki * self.integral + self.kd * deriv)
        if abs(angle) < np.pi / 18:
            speed = 1.5
        elif abs(angle) < np.pi / 18 * 2:
            speed = 1.0
        else:
            speed = 0.5
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = angle
        self.pub_.publish(drive_msg)

    def scan_callback(self, msg: LaserScan):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        if self.lidar_angle_increment is None:
            self.lidar_angle_increment = msg.angle_increment
            self.lidar_angle_max = msg.angle_max
            self.lidar_angle_min = msg.angle_min
        range_data = msg.ranges
        
        error = self.get_error(range_data, self.dist) # TODO
        self.get_logger().info(f"err: {error}")
        velocity = 0.0 # TODO: calculate desired car velocity based on error
        self.pid_control(error, velocity) # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    wall_follow_node = WallFollow()
    wall_follow_node.get_logger().info(("WallFollow Initialized"))
    rclpy.spin(wall_follow_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
