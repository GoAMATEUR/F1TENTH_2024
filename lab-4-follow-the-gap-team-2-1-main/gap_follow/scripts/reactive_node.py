#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from scipy import ndimage
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

#  Constants from xacro
CHASSIS_WIDTH = 0.2032  # (m)
TIRE_WIDTH = 0.0381  # (m)
CAR_WIDTH = CHASSIS_WIDTH + 2 * TIRE_WIDTH
MAX_STEER = 0.36  # (rad)


class ReactiveFollowGap(Node):
    """
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """

    def __init__(self):
        super().__init__('reactive_node')

        # Params
        self.declare_parameter('window_size', 5)
        self.declare_parameter('rejection_thres', 2.5)
        self.declare_parameter('distance_thres', 2.0)

        self.declare_parameter('consecutive_hits', 5)
        self.declare_parameter('bubble_radius_factor', 0.7)
        self.declare_parameter('disparity_thres', 0.15)

        self.declare_parameter('k_p', 1.0)
        self.declare_parameter('k_i', 0.0)
        self.declare_parameter('k_d', 0.1)
        self.declare_parameter("alpha", 0.4)

        # self.declare_parameter("low_vel", 0.5)
        # self.declare_parameter("high_vel", 3.0)
        # self.declare_parameter("velocity_attenuation", 1.5)

        self.range_min = None
        self.range_max = None
        self.ranges_length = None       #length of ranges
        self.angle_min = None        # angle range
        self.angle_max = None
        self.angle_increment = None
        self.front_idx_start = None    # idx of scans in -90~90 degrees ahead
        self.front_idx_end = None

        # PID Control Params
        self.prev_angle = 0.0
        self.integral = 0.0
        self.prev_steer = 0.0

        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        self.sub_ = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        self.pub_ = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

    def angle_to_index(self, angle: float):
        return int(round((np.clip(angle, self.angle_min, self.angle_max) - self.angle_min) / self.angle_increment))

    def index_to_angle(self, index: int):
        return self.angle_min + index * self.angle_increment

    def preprocess_lidar(self, ranges: np.ndarray):
        window_size = self.get_parameter('window_size').get_parameter_value().integer_value
        rejection_thres = self.get_parameter('rejection_thres').get_parameter_value().double_value
        # Remove invalid readings
        ranges = np.clip(ranges, self.range_min, self.range_max) 
        # Average over window
        ranges = np.convolve(ranges, np.ones(window_size, dtype=float) / window_size, 'same')
        # Clip high values
        ranges[[0, -1]] *= 1.67
        ranges[[1, -2]] *= 1.25
        ranges = np.clip(ranges, 0, rejection_thres)
        return ranges

    def find_disparities(self, ranges):
        disparity_thres = self.get_parameter('disparity_thres').get_parameter_value().double_value
        disparities = np.diff(ranges) # [i] = [i+1] - [i], if 
        left_indices = np.where(disparities > disparity_thres)[0] # bubble spread right
        right_indices = np.where(disparities < -disparity_thres)[0] + 1
        # self.get_logger().info("Num disparities: %0.2f" % len(indices))
        return left_indices, right_indices

    def create_bubbles(self, ranges: np.ndarray):
        bubble_radius = self.get_parameter('bubble_radius_factor').get_parameter_value().double_value * CAR_WIDTH
        closest_indices = [np.argmin(ranges)]
        left_indices, right_indices = self.find_disparities(ranges)
        # indices = np.hstack((closest_indices, disparity_indices))
        self.get_logger().info("Closest {}, left {}, right {}".format(closest_indices, left_indices, right_indices))
        new_ranges = ranges.copy()
        
        for center_idx in closest_indices:
            center_angle = self.index_to_angle(center_idx)
            if bubble_radius < ranges[center_idx]:
                theta = abs(np.arcsin(bubble_radius / ranges[center_idx]))
            else:
                theta = np.pi / 6
                self.get_logger().warning("CLOSE TO WALL; Closest")
            bubble_index_start  = self.angle_to_index(center_angle - theta)
            bubble_index_end    = self.angle_to_index(center_angle + theta)
            new_ranges[bubble_index_start:bubble_index_end] = 0.0
            
        for center_idx in left_indices:
            center_angle = self.index_to_angle(center_idx)
            if bubble_radius < ranges[center_idx]:
                theta = abs(np.arcsin(bubble_radius / ranges[center_idx]))
            else:
                theta = np.pi / 6
                self.get_logger().warning("CLOSE TO WALL; dis")
            bubble_index_start  = center_idx
            bubble_index_end    = self.angle_to_index(center_angle + theta)
            new_ranges[bubble_index_start:bubble_index_end] = ranges[center_idx]
        
        for center_idx in right_indices:
            center_angle = self.index_to_angle(center_idx)
            if bubble_radius < ranges[center_idx]:
                theta = abs(np.arcsin(bubble_radius / ranges[center_idx]))
            else:
                theta = np.pi / 6
                self.get_logger().warning("CLOSE TO WALL; dis")
            bubble_index_start  = self.angle_to_index(center_angle - theta)
            bubble_index_end    = center_idx
            new_ranges[bubble_index_start:bubble_index_end] = ranges[center_idx]
            
        return new_ranges

    def find_max_gap(self, ranges, limited_range=True):
        consecutive_hits = self.get_parameter("consecutive_hits").get_parameter_value().integer_value
        distance_thres = self.get_parameter("distance_thres").get_parameter_value().double_value
        if limited_range:
            ranges = ranges[self.front_idx_start:self.front_idx_end+1]
            offset = self.front_idx_start
        else:
            offset = 0
        extend_ranges = np.hstack((0, ranges, 0))
        zero_indices = np.where(extend_ranges <  distance_thres)[0]
        gap_angles = np.hstack((0, np.diff(zero_indices) - 1))
        right_bound = np.where(gap_angles > 0)[0]
        left_bound = right_bound - 1
        gap_bounds = zero_indices[np.vstack((left_bound, right_bound)).T]
        gap_bounds = gap_bounds + np.array([1, -1])
        gap_bounds = gap_bounds - 1
        print("GAP_BUNDS gap_bounds", gap_bounds)
        gap_lengths = (np.diff(gap_bounds, axis=-1) + 1).flatten()
        
        # max length gap
        # TODO: Max gap selection criteria
        
        gap_bound = gap_bounds[np.argmax(gap_lengths)] + offset
        # print(gap_lengths)
        return gap_bound

    def find_best_point(self, gap_bound, ranges):
        start_i = gap_bound[0]
        end_i = gap_bound[1]
        gap = ranges[start_i:end_i+1]
        max_range = np.max(gap)
        max_ids = np.where(gap == max_range)[0]
        avg_mid = int(np.mean(max_ids))
        return start_i + avg_mid

    def get_steer(self, angle):
        kp = self.get_parameter('k_p').get_parameter_value().double_value
        ki = self.get_parameter('k_i').get_parameter_value().double_value
        kd = self.get_parameter('k_d').get_parameter_value().double_value
        alpha = self.get_parameter('alpha').get_parameter_value().double_value
        d_error = angle - self.prev_angle
        self.prev_angle = angle
        self.integral += angle
        steer = kp * angle + ki * self.integral + kd * d_error
        new_steer = np.clip(steer, -MAX_STEER, MAX_STEER)
        new_steer = alpha * new_steer + (1 - alpha) * self.prev_steer
        self.prev_steer = new_steer
        return new_steer

    # def get_velocity(self, error):
    #     """ Get desired velocity based on current error
    #     """
    #     # Speed is exponential w.r.t error
    #     low_vel = self.get_parameter('low_vel').get_parameter_value().double_value
    #     high_vel = self.get_parameter('high_vel').get_parameter_value().double_value
    #     atten = self.get_parameter('velocity_attenuation').get_parameter_value().double_value

    #     return (high_vel - low_vel) * np.exp(-abs(error) * atten) + low_vel
    def get_velocity(self, angle):
        if abs(angle) < np.pi / 18:
            speed = 1.5
        elif abs(angle) < np.pi / 18 * 2:
            speed = 1.0
        else:
            speed = 0.5
        return speed

    def lidar_callback(self, data: LaserScan):
        ranges = np.array(data.ranges, dtype=float)
        if self.range_min == None:
            self.range_min = data.range_min
            self.range_max = data.range_max
            self.ranges_length = len(ranges)       #length of ranges
            self.angle_min = data.angle_min        # angle range
            self.angle_max = data.angle_max
            self.angle_increment = data.angle_increment
            self.front_idx_start = self.angle_to_index(-np.pi/2)    # idx of scans in -90~90 degrees ahead
            self.front_idx_end = self.ranges_length - self.front_idx_start
        # 2. Preprocess ranges data
        
        # print(list(ranges))
        ranges = self.preprocess_lidar(ranges)
        # print("==================pro")
        # print(list(ranges))
        # 3. create bubble:
        ranges = self.create_bubbles(ranges)
        # print(">>>>>>>>>>>bubbles")
        # print(list(ranges))        
        # Find max length gap
        gap_bound= self.find_max_gap(ranges, limited_range=True)
        self.get_logger().info("Gap: ({}, {})".format(gap_bound[0], gap_bound[1]))

        # Find the best point in the gap
        best_idx = self.find_best_point(gap_bound, ranges)

        self.get_logger().info("best_idx: {}".format(best_idx))

        # Get speed and steer
        angle = self.index_to_angle(best_idx)
        steer = self.get_steer(angle)
        speed = self.get_velocity(steer)
        # steer = self.get_steer(angle)

        # Publish Drive message
        self.get_logger().info("Error: %0.2f,\t Steer: %0.2f,\t Vel: %0.2f" % (np.rad2deg(angle),
                                                                               np.rad2deg(steer),
                                                                               speed))
        message = AckermannDriveStamped()
        message.drive.speed = speed
        message.drive.steering_angle = steer
        self.pub_.publish(message)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
